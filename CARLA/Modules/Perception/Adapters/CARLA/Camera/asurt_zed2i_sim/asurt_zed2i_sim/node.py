"""CARLA stereo observer exposing legacy ASURT ZED ROS interfaces.

Two CARLA RGB sensors represent the two lenses of one ZED 2i. The process
NEVER ticks the world, creates a hero, edits weather, or sends driving commands.
"""
from array import array as byte_array
from concurrent.futures import ThreadPoolExecutor
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import Object, ObjectsStamped

from .detection import CONE_LABEL_IDS, detect_stereo, draw_overlay
from .clock_alignment import ClockOffsetCalibrator
from .optics import Calibration
from .overlay_sync import next_overlay_frame
from .stereo_sync import StereoSynchronizer
from .tracking import ConeTracker
from .weather import WeatherEffects, camera_ground_plane, decode_scene_depth, needs_scene_depth


def load_carla():
    """Import matching CARLA binding without replacing system NumPy/OpenCV."""
    try:
        import carla
        return carla
    except ImportError:
        site = os.environ.get(
            "ASURT_CARLA_SITE",
            "/home/hassan/ASURT/.venvs/asurt-native/lib/python3.10/site-packages",
        )
        if site not in sys.path:
            sys.path.append(site)  # append, never prepend NumPy 2 from venv
        import carla
        return carla


def stamp_seconds(t):
    sec = math.floor(t)
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1000000000:
        sec += 1
        nsec -= 1000000000
    return Time(sec=sec, nanosec=nsec)


def image_message(array, stamp, frame):
    msg = Image()
    msg.header = Header(stamp=stamp, frame_id=frame)
    msg.height, msg.width = array.shape[:2]
    msg.encoding = "bgr8"
    msg.step = msg.width * 3
    msg.is_bigendian = 0
    # ROS2 Humble Python's Image.data setter iterates individual bytes when
    # assigned `bytes` (~hundreds of ms for 640x360). array('B') converts in C
    # and serializes efficiently, which matters for live stereo throughput.
    msg.data = byte_array('B', array.tobytes())
    return msg


def optical_rotation():
    """camera_link (FLU) -> optical (right/down/forward)."""
    # Standard ROS static optical transform: Euler(-pi/2, 0, -pi/2)
    return (-0.5, 0.5, -0.5, 0.5)


class Zed2iStereoNode(Node):
    def __init__(self):
        super().__init__("zed2i_stereo_node")
        reference_cal = Calibration()
        defaults = {
            "host": "127.0.0.1", "port": 2000, "role_name": "hero",
            "image_width": reference_cal.width, "image_height": reference_cal.height,
            "horizontal_fov_deg": reference_cal.hfov_deg,
            "baseline_m": reference_cal.baseline_m,
            "mount_x_m": reference_cal.mount_x,
            "mount_y_m": reference_cal.mount_y,
            "mount_z_m": reference_cal.mount_z, "mount_pitch_deg": 0.0,
            "mount_yaw_deg": 0.0, "publish_rate_hz": 10.0,
            "enable_weather_effects": True, "glare_scale": 0.8,
            "droplets_scale": 0.75, "noise_std": 1.5,
            "enable_legacy_detection": True,
            "vision_backend": "yolo26",
            "yolo_model_path": "",
            "yolo_confidence": 0.30,
            "yolo_cpu_threads": 2,
            "yolo_template_threshold": 0.72,
            "sensor_rate_hz": 10.0,
            "enable_temporal_tracking": True,
            "max_stereo_skew_s": 0.05,
            "legacy_objects_topic": "/zed/zed_node/obj_det/objects",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        if self.p["mount_pitch_deg"] != 0.0 or self.p["mount_yaw_deg"] != 0.0:
            raise ValueError("Nonzero mount orientation requires calibrated TF extrinsics")
        self.cal = Calibration(
            width=self.p["image_width"], height=self.p["image_height"],
            hfov_deg=self.p["horizontal_fov_deg"],
            baseline_m=self.p["baseline_m"],
            mount_x=self.p["mount_x_m"], mount_y=self.p["mount_y_m"],
            mount_z=self.p["mount_z_m"],
        )
        self.effects = WeatherEffects(
            enabled=self.p["enable_weather_effects"],
            glare_scale=self.p["glare_scale"],
            droplets_scale=self.p["droplets_scale"],
            noise_std=self.p["noise_std"],
        )
        self.lock = threading.Lock()
        self.stereo = StereoSynchronizer(
            max_skew_s=self.p["max_stereo_skew_s"], max_pending_per_eye=10)
        self.capture_counts = {"left": 0, "right": 0}
        self.capture_prev_wall = {"left": None, "right": None}
        self.capture_max_gap = {"left": 0.0, "right": 0.0}
        self.capture_skew_max = 0.0
        self.capture_pair_count = 0
        self.capture_report_before = (time.monotonic(), 0, 0, 0, 0)
        self.clock_calibration = ClockOffsetCalibrator()
        self.clock_offset = None
        self.weather = None
        self.camera_yaw = 0.0
        self.camera_pitch = 0.0
        self.actors = {}
        self.depth_actors = {}
        self.depth_frames = {"left": {}, "right": {}}
        self.destroyed = False
        self.received_pairs = 0
        self.tracker = ConeTracker()
        self.yolo_detector = None
        self.detection_pool = None
        self.detection_future = None
        self.skipped_detection_frames = 0
        self.latest_left = None
        self.latest_cones = None
        self.processed_left = None
        self.displayed_frame = -1
        self.latest_clock_seconds = None
        self.last_display_age = None
        self.next_pair_wall = 0.0
        if self.p["vision_backend"] not in ("hsv", "yolo26"):
            raise ValueError("vision_backend must be 'hsv' or 'yolo26'")
        if self.p["vision_backend"] == "yolo26" and self.p["enable_legacy_detection"]:
            from .yolo26 import Yolo26Stereo
            self.yolo_detector = Yolo26Stereo(
                model_path=self.p["yolo_model_path"],
                confidence=self.p["yolo_confidence"],
                cpu_threads=self.p["yolo_cpu_threads"],
                template_threshold=self.p["yolo_template_threshold"],
            )
            # Keep ROS image/clock callbacks responsive while the CPU runs
            # the ONNX model. There is never more than one pending inference.
            self.detection_pool = ThreadPoolExecutor(max_workers=1,
                                                      thread_name_prefix="asurt-yolo26")
        self.last_marker_ids = set()
        self.marker_bootstrap_clear = True
        self.pose = None
        self.pose_time = 0.0
        self.carla = load_carla()
        self.client = self.carla.Client(self.p["host"], self.p["port"])
        self.client.set_timeout(6.0)
        self.world = self.client.get_world()
        self.world.wait_for_tick(15)
        heroes = [a for a in self.world.get_actors().filter("vehicle.*")
                  if a.attributes.get("role_name") == self.p["role_name"]]
        if len(heroes) != 1 or heroes[0].type_id != "vehicle.vehicle.asurt_fsai":
            raise RuntimeError("Expected exactly one existing ASURT FSAI hero; no vehicle created")
        self.hero = heroes[0]
        if not self.world.get_settings().synchronous_mode:
            self.get_logger().warning("World is asynchronous; node still will not tick it")
        others = [a.id for a in self.world.get_actors().filter("sensor.camera.*")
                  if a.attributes.get("role_name", "").startswith("asurt_zed2i_sim_")]
        if others:
            raise RuntimeError(f"Found previous ASURT ZED sensor actor(s) {others}; inspect before restart")

        self.images = {
            eye: self.create_publisher(
                Image, f"/zed/zed_node/{eye}/image_rect_color", qos_profile_sensor_data)
            for eye in ("left", "right")
        }
        self.camera_infos = {
            eye: self.create_publisher(
                CameraInfo, f"/zed/zed_node/{eye}/camera_info", qos_profile_sensor_data)
            for eye in ("left", "right")
        }
        self.overlay_pub = self.create_publisher(
            Image, "/zed/zed_node/rgb/cones_overlay", qos_profile_sensor_data)
        self.object_pub = self.create_publisher(
            ObjectsStamped, self.p["legacy_objects_topic"], 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, "/zed/zed_node/cone_markers", 10)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odometry, qos_profile_sensor_data)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.publish_tf()
        try:
            self.spawn_cameras()
        except Exception:
            self.cleanup()
            raise
        self.refresh_weather()
        self.create_timer(0.75, self.refresh_weather)
        self.create_timer(0.02, self.publish_pair)
        self.create_timer(0.02, self.publish_live_view)
        self.create_timer(4.0, self.report_capture)
        self.get_logger().info(
            f"ZED 2i stereo: hero={self.hero.id} sensors={[(k,v.id) for k,v in self.actors.items()]}; "
            f"{self.cal.width}x{self.cal.height}, HFOV {self.cal.hfov_deg}, "
            f"baseline {self.cal.baseline_m}m, "
            f"CARLA hero-relative camera-center mount=({self.cal.mount_x:.4f}, "
            f"{-self.cal.mount_y:.4f}, {self.cal.mount_z:.4f})m; "
            f"vision={self.p['vision_backend']}; passive observer")

    def on_clock(self, msg):
        self.latest_clock_seconds = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.monotonic()
        if not self.clock_calibration.due(now):
            return
        try:
            snap = self.world.get_snapshot()
        except Exception as exc:
            self.clock_calibration.defer(now)
            self.get_logger().warning(f"ZED clock calibration snapshot unavailable: {exc}")
            return
        epoch = self.latest_clock_seconds
        if self.clock_calibration.observe(
            epoch, snap.timestamp.elapsed_seconds, now, snap.frame
        ):
            self.clock_offset = self.clock_calibration.offset
            self.get_logger().info(
                f"ZED clock epoch offset locked to {self.clock_offset:.6f}s "
                f"from {len(self.clock_calibration.samples)} paired samples "
                f"over {self.clock_calibration.window_seconds:.2f}s"
            )

    def on_odometry(self, msg):
        # Only compensate motion using the actual world/odom vehicle pose.
        # This is never treated as a ground-truth cone observation.
        if msg.header.frame_id.lstrip("/") not in ("map", "odom", "world"):
            return
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1-2*(q.y*q.y+q.z*q.z))
        p = msg.pose.pose.position
        if all(math.isfinite(x) for x in (p.x, p.y, yaw)):
            self.pose = (float(p.x), float(p.y), float(yaw))
            self.pose_time = time.monotonic()

    def publish_tf(self):
        transforms = []
        qx, qy, qz, qw = optical_rotation()
        for eye, sign in (("left", 1.0), ("right", -1.0)):
            parent = f"zed_{eye}_camera_frame"
            t = TransformStamped()
            t.header.frame_id = "base_link"
            t.header.stamp = self.get_clock().now().to_msg()
            t.child_frame_id = parent
            t.transform.translation.x = self.cal.mount_x
            t.transform.translation.y = self.cal.mount_y + sign*self.cal.baseline_m/2
            t.transform.translation.z = self.cal.mount_z
            t.transform.rotation.w = 1.0
            transforms.append(t)
            optical = TransformStamped()
            optical.header = Header(stamp=t.header.stamp, frame_id=parent)
            optical.child_frame_id = parent.replace("_frame", "_optical_frame")
            optical.transform.rotation.x = qx
            optical.transform.rotation.y = qy
            optical.transform.rotation.z = qz
            optical.transform.rotation.w = qw
            transforms.append(optical)
        self.tf_pub.sendTransform(transforms)

    def spawn_cameras(self):
        library = self.world.get_blueprint_library()
        commands = []
        eye_order = []
        for eye, sign in (("left", -1.0), ("right", 1.0)):
            blueprint = library.find("sensor.camera.rgb")
            params = {
                "image_size_x": str(self.cal.width),
                "image_size_y": str(self.cal.height),
                "fov": str(self.cal.hfov_deg),
                # Camera render rate MUST match the actual perception budget.
                # 0.0 rendered BOTH eyes on every 30 Hz world tick even though
                # we only used 10 pairs/s: huge needless GPU/CPU overhead.
                "sensor_tick": str(1.0/max(1.0, self.p["sensor_rate_hz"])),
                "role_name": f"asurt_zed2i_sim_{eye}",
                "enable_postprocess_effects": "true",
                "gamma": "2.2",
                "bloom_intensity": "0.4",
                "lens_flare_intensity": "0.3",
            }
            for k, v in params.items():
                if blueprint.has_attribute(k):
                    blueprint.set_attribute(k, v)
            eye_order.append(eye)
            commands.append(self.carla.command.SpawnActor(
                blueprint, self.camera_mount(sign), self.hero.id
            ))

        # Two separate spawn_actor RPCs can land on different world ticks.
        # When the cameras are rate-limited, that capture-phase offset can
        # persist indefinitely and exceed the strict epipolar skew budget.
        # Spawn both sensors in ONE CARLA command batch, without ticking the
        # synchronous world; the existing bridge remains sole tick owner.
        responses = self.client.apply_batch_sync(commands, False)
        if len(responses) != len(eye_order):
            raise RuntimeError("CARLA did not report both stereo spawn results")
        spawned = []
        for eye, response in zip(eye_order, responses):
            if not response.error:
                spawned.append((eye, response.actor_id))
        try:
            if len(spawned) != 2:
                errors = [f"{eye}: {response.error}" for eye, response in
                          zip(eye_order, responses) if response.error]
                raise RuntimeError(f"Stereo spawn batch failed: {errors}")
            for eye, actor_id in spawned:
                sensor = self.world.get_actor(actor_id)
                if sensor is None or sensor.type_id != "sensor.camera.rgb":
                    raise RuntimeError(f"Stereo actor {actor_id} is missing/incorrect")
                if sensor.attributes.get("role_name") != f"asurt_zed2i_sim_{eye}":
                    raise RuntimeError(f"Stereo actor {actor_id} has unexpected ownership")
                self.actors[eye] = sensor
                sensor.listen(lambda img, e=eye: self.on_image(e, img))
        except Exception:
            # Destroy ONLY the IDs returned by this batch, even if a later
            # validation/listener call fails. Never touch other CARLA actors.
            for eye, actor_id in spawned:
                sensor = self.world.get_actor(actor_id)
                if sensor is not None and sensor.type_id == "sensor.camera.rgb" and (
                    sensor.attributes.get("role_name") == f"asurt_zed2i_sim_{eye}"
                ):
                    try:
                        sensor.stop()
                        sensor.destroy()
                    except RuntimeError:
                        pass
            self.actors.clear()
            raise

    def on_image(self, eye, img):
        if self.destroyed:
            return
        try:
            arrived = time.monotonic()
            bgr = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(
                img.height, img.width, 4)[:, :, :3].copy()
            with self.lock:
                self.capture_counts[eye] += 1
                previous = self.capture_prev_wall[eye]
                if previous is not None:
                    self.capture_max_gap[eye] = max(
                        self.capture_max_gap[eye], arrived - previous)
                self.capture_prev_wall[eye] = arrived
                self.stereo.add(eye, img.frame, img.timestamp, bgr, arrived)
                if eye == "left" and (self.latest_left is None or img.frame > self.latest_left[0]):
                    self.latest_left = (img.frame, bgr, img.timestamp, arrived)
        except Exception as exc:
            self.get_logger().warning(f"Stereo callback dropped {eye}: {exc}")

    def report_capture(self):
        with self.lock:
            now = time.monotonic()
            prev_time, prev_l, prev_r, prev_pairs, prev_dropped = self.capture_report_before
            left, right = self.capture_counts["left"], self.capture_counts["right"]
            pairs = self.capture_pair_count
            discarded = self.stereo.dropped_unpaired
            pending_l = len(self.stereo.frames["left"])
            pending_r = len(self.stereo.frames["right"])
            late = self.stereo.dropped_late
            gaps = dict(self.capture_max_gap)
            self.capture_max_gap = {"left": 0.0, "right": 0.0}
            skew = self.capture_skew_max
            self.capture_skew_max = 0.0
            self.capture_report_before = (now, left, right, pairs, discarded)
        interval = max(0.001, now - prev_time)
        self.get_logger().info(
            f"Capture window={interval:.1f}s left={(left-prev_l)/interval:.1f}Hz "
            f"right={(right-prev_r)/interval:.1f}Hz pairs={(pairs-prev_pairs)/interval:.1f}Hz "
            f"unpaired_pruned={discarded-prev_dropped} late_total={late} "
            f"pending_left={pending_l} pending_right={pending_r} "
            f"max_intereye_skew={skew:.3f}s max_callback_gaps={gaps} "
            f"last_display_age_ms={self.last_display_age}")

    def refresh_weather(self):
        try:
            self.weather = self.world.get_weather()
            transform = self.actors["left"].get_transform()
            self.camera_yaw = transform.rotation.yaw
            self.camera_pitch = transform.rotation.pitch
            depth_needed = self.effects.enabled and needs_scene_depth(self.weather)
            if depth_needed and not self.depth_actors:
                self.start_depth_cameras()
            elif not depth_needed and self.depth_actors:
                self.stop_depth_cameras()
        except RuntimeError as exc:
            self.get_logger().warning(f"Weather/pose query failed: {exc}", throttle_duration_sec=10.0)

    def start_depth_cameras(self):
        blueprint = self.world.get_blueprint_library().find("sensor.camera.depth")
        settings = {
            "image_size_x": str(self.cal.width),
            "image_size_y": str(self.cal.height),
            "fov": str(self.cal.hfov_deg),
            "sensor_tick": str(1.0 / max(1.0, self.p["sensor_rate_hz"])),
        }
        for attr, value in settings.items():
            blueprint.set_attribute(attr, value)
        try:
            for eye, sign in (("left", -1.0), ("right", 1.0)):
                blueprint.set_attribute("role_name", f"asurt_zed2i_sim_depth_{eye}")
                actor = self.world.spawn_actor(blueprint, self.camera_mount(sign), attach_to=self.hero)
                self.depth_actors[eye] = actor
                actor.listen(lambda img, e=eye, ident=actor.id: self.on_depth(e, ident, img))
        except RuntimeError:
            self.stop_depth_cameras()
            raise

    def camera_mount(self, sign):
        # CARLA y right, ROS base_link y left. Cameras parallel & rectified.
        return self.carla.Transform(
            self.carla.Location(x=self.cal.mount_x,
                                y=-self.cal.mount_y + sign * self.cal.baseline_m / 2,
                                z=self.cal.mount_z),
            self.carla.Rotation(pitch=self.p["mount_pitch_deg"], yaw=self.p["mount_yaw_deg"]),
        )

    def on_depth(self, eye, actor_id, image):
        depth = decode_scene_depth(image)
        plane = camera_ground_plane(image.transform, self.hero.get_location().z)
        with self.lock:
            actor = self.depth_actors.get(eye)
            if actor is None or actor.id != actor_id:
                return
            frames = self.depth_frames[eye]
            frames[image.frame] = (image.timestamp, depth, plane)
            if len(frames) > 8:
                del frames[min(frames)]

    def scene_depth(self, eye, frame, timestamp):
        with self.lock:
            frames = self.depth_frames[eye]
            if not frames:
                return None, None
            closest = min(frames, key=lambda key: abs(key - frame))
            captured, depth, plane = frames[closest]
            if abs(captured - timestamp) <= 0.20:
                return depth, plane
        return None, None

    def stop_depth_cameras(self):
        with self.lock:
            actors = self.depth_actors.copy()
            self.depth_actors.clear()
            for frames in self.depth_frames.values():
                frames.clear()
        for eye, actor in actors.items():
            try:
                actor.stop()
                actor.destroy()
            except RuntimeError as exc:
                self.get_logger().warning(f"Failed to remove {eye} depth camera: {exc}")

    def info_for(self, eye, stamp):
        c = self.cal
        info = CameraInfo()
        info.header = Header(stamp=stamp, frame_id=f"zed_{eye}_camera_optical_frame")
        info.width, info.height = c.width, c.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5  # rectified virtual pinhole, not factory intrinsics
        info.k = [c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        tx = -c.fx*c.baseline_m if eye == "right" else 0.0
        info.p = [c.fx, 0.0, c.cx, tx,
                  0.0, c.fy, c.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def process_eye(self, eye, sample):
        frame, bgr, timestamp, _ = sample
        depth, plane = self.scene_depth(eye, frame, timestamp)
        return self.effects.process(
            bgr, self.weather, self.camera_yaw, self.camera_pitch, frame,
            0 if eye == "left" else 1, self.cal.hfov_deg, depth, plane, timestamp=timestamp,
        )

    def process_left(self, sample):
        frame = sample[0]
        if self.processed_left is None or self.processed_left[0] != frame:
            image = self.process_eye("left", sample)
            self.processed_left = (frame, image)
        return self.processed_left[1]

    def publish_live_view(self):
        # The overlay must display the image actually analyzed by the detector.
        # Publishing newer raw captures between asynchronous YOLO completions
        # makes boxes blink and can drop every result when inference is slower
        # than capture. Raw left/right image topics remain independent.
        if self.clock_offset is None or not self.overlay_pub.get_subscription_count():
            return
        with self.lock:
            sample = self.latest_left
            detection = self.latest_cones
        enabled = self.p["enable_legacy_detection"]
        selected = next_overlay_frame(self.displayed_frame, sample, detection, enabled)
        if selected is None:
            return
        if enabled:
            frame, image, stamp, arrived, cones = selected
            image = draw_overlay(image, cones)
            epoch = stamp.sec + stamp.nanosec * 1e-9
        else:
            frame, _, captured, arrived = selected
            image = self.process_left(selected)
            epoch = captured + self.clock_offset
            stamp = stamp_seconds(epoch)
        self.overlay_pub.publish(image_message(image, stamp,
                                               "zed_left_camera_optical_frame"))
        self.displayed_frame = frame
        sim_age = self.latest_clock_seconds - epoch if self.latest_clock_seconds is not None else 0
        self.last_display_age = {"sim": round(1000 * max(0, sim_age)),
                                 "after_capture": round(1000 * (time.monotonic() - arrived))}

    def publish_pair(self):
        # Buffer bounded stereo frames until a capture-time epoch is known.
        # Never mislabel an old captured pair with the current /clock value.
        now = time.monotonic()
        if self.clock_offset is None or now < self.next_pair_wall:
            return
        timing_start = time.perf_counter()
        with self.lock:
            selected = self.stereo.pop_latest()
            if selected is None:
                return
            left, right, skew = selected
            frame = left[0]
            self.capture_pair_count += 1
            self.capture_skew_max = max(self.capture_skew_max, skew)
        self.next_pair_wall = now + 1.0 / max(1.0, self.p["publish_rate_hz"])
        # Use the source-frame stamps rather than wall-time callback arrival.
        # Approximate synchronization is strictly bounded to the configured
        # skew; its effect on moving-scene depth still requires validation.
        stamp = stamp_seconds(left[2] + self.clock_offset)
        images = {}
        for eye, img in (("left", self.process_left(left)), ("right", self.process_eye("right", right))):
            images[eye] = img
            optical_frame = f"zed_{eye}_camera_optical_frame"
            # Topic contracts remain available, but avoid serializing a
            # 0.7 MB ROS Image when no consumer has subscribed.
            if self.images[eye].get_subscription_count():
                self.images[eye].publish(image_message(img, stamp, optical_frame))
            if self.camera_infos[eye].get_subscription_count():
                self.camera_infos[eye].publish(self.info_for(eye, stamp))
        timing_images = time.perf_counter()
        image_ms = 1000.0 * (timing_images - timing_start)
        if self.detection_pool is None:
            self.publish_detections(images, stamp, left, image_ms)
            return
        # Drop detection work while one inference is outstanding, rather than
        # building stale-frame latency. Raw stereo images continue to publish.
        if self.detection_future is not None:
            if not self.detection_future.done():
                self.skipped_detection_frames += 1
                return
            try:
                self.detection_future.result()
            except Exception as exc:
                self.get_logger().error(f"YOLO26 inference failed: {exc}")
            self.detection_future = None
        self.detection_future = self.detection_pool.submit(
            self.publish_detections, images, stamp, left, image_ms)

    def publish_detections(self, images, stamp, source, image_ms):
        frame = source[0]
        detector_start = time.perf_counter()
        objects = ObjectsStamped()
        objects.header = Header(stamp=stamp, frame_id="base_link")
        markers = MarkerArray()
        if self.marker_bootstrap_clear or not self.p["enable_temporal_tracking"]:
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            self.marker_bootstrap_clear = False
        cones = []
        pose = None
        if self.p["enable_legacy_detection"]:
            if self.yolo_detector is not None:
                cones = self.yolo_detector.detect_stereo(
                    images["left"], images["right"], self.cal)
            else:
                cones = detect_stereo(images["left"], images["right"], self.cal)
            if self.p["enable_temporal_tracking"]:
                pose = self.pose if time.monotonic()-self.pose_time < 1.5 else None
                # Age visualization holds from actual image arrival, not from
                # when a slow inference worker finishes the old frame.
                cones = self.tracker.update(cones, pose, observed_at=source[3])
        timing_detection = time.perf_counter()
        for i, cone in enumerate(cones):
            ob = Object()
            ob.label = cone.color + "_cone"
            ob.label_id = CONE_LABEL_IDS[cone.color]
            ob.confidence = float(cone.confidence)
            ob.position = [float(cone.x), float(cone.y), float(cone.z)]
            ob.tracking_state = 1
            ob.tracking_available = bool(self.p["enable_temporal_tracking"])
            if self.p["enable_temporal_tracking"]:
                ob.sublabel = f"asurt_track_{cone.id}"
            x, y, w, h = cone.box
            corners = [(x,y), (x+w,y), (x+w,y+h), (x,y+h)]
            for kp, (u,v) in zip(ob.bounding_box_2d.corners, corners):
                kp.kp = [int(u), int(v)]
            ob.dimensions_3d = ([0.285, 0.505, 0.285] if cone.color == "large_orange"
                                else [0.22, 0.325, 0.22])
            objects.objects.append(ob)
        # Keep short detection dropouts from erasing otherwise confirmed RViz
        # cylinders. Fresh-only ObjectsStamped (above) and the camera overlay
        # remain faithful to this inference; held tracks are display-only.
        marker_cones = list(cones)
        fresh_ids = {c.id for c in cones} if self.p["enable_temporal_tracking"] else set()
        if self.p["enable_temporal_tracking"]:
            held = self.tracker.held_visualization(self.last_marker_ids - fresh_ids,
                                                   pose=pose)
            for c in held:
                # A new confirmed detection takes precedence over the last
                # visual estimate, even if its colour or track ID changed.
                if not any(math.hypot(c.x - observed.x, c.y - observed.y) < 0.45
                           for observed in cones):
                    marker_cones.append(c)
        for i, cone in enumerate(marker_cones):
            marker = Marker()
            marker.header = objects.header
            marker.ns = "zed2i_detected_cones"
            marker.id = cone.id if self.p["enable_temporal_tracking"] else i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = cone.x
            marker.pose.position.y = cone.y
            marker.pose.position.z = cone.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 0.285 if cone.color == "large_orange" else 0.22
            marker.scale.z = 0.505 if cone.color == "large_orange" else 0.325
            # A stalled detection worker must not leave RViz ghosts forever.
            # Each fresh update refreshes this lifetime; explicit DELETE below
            # handles expired or genuinely missing tracks on the next result.
            marker.lifetime.sec = 1
            marker.lifetime.nanosec = 0
            color = {"blue": (0.03,0.18,1.0), "yellow": (1.0,0.9,0.02),
                     "orange": (1.0,0.35,0.02),
                     "large_orange": (1.0,0.25,0.01)}[cone.color]
            marker.color.r, marker.color.g, marker.color.b = color
            # Hold the same opacity during brief misses; otherwise changing
            # alpha on every reacquisition still looks like flicker in RViz.
            marker.color.a = 1.0
            markers.markers.append(marker)
        # Stable identifiers and explicit DELETE prevent RViz from confusing
        # one disappearing cone with the next cone in a reindexed array.
        current_ids = {m.id for m in markers.markers if m.action == Marker.ADD}
        for missing_id in self.last_marker_ids - current_ids:
            gone = Marker()
            gone.header = objects.header
            gone.ns = "zed2i_detected_cones"
            gone.id = missing_id
            gone.action = Marker.DELETE
            markers.markers.append(gone)
        self.last_marker_ids = current_ids
        self.object_pub.publish(objects)
        self.markers_pub.publish(markers)
        timing_messages = time.perf_counter()
        with self.lock:
            # Preserve the image, source timestamp and detections as one atomic
            # snapshot. A newer left capture is not grounds for discarding a
            # completed inference; its frame is often ahead of the worker.
            if self.latest_cones is None or frame > self.latest_cones[0]:
                self.latest_cones = (frame, images["left"], stamp, source[3], cones)
        self.received_pairs += 1
        if self.received_pairs % 10 == 0:
            self.get_logger().info(
                f"Stereo pairs={self.received_pairs} frame={frame} detected_cones={len(cones)} "
                f"vision={self.p['vision_backend']} "
                f"skipped_busy={self.skipped_detection_frames} "
                f"latency_ms=images:{image_ms:.0f} "
                f"detection:{1000*(timing_detection-detector_start):.0f} "
                f"objects:{1000*(timing_messages-timing_detection):.0f} "
                f"overlay:{1000*(time.perf_counter()-timing_messages):.0f}")

    def cleanup(self):
        if self.destroyed:
            return
        self.destroyed = True
        self.stop_depth_cameras()
        if self.detection_pool is not None:
            # Drain at most one active inference before releasing publishers
            # and owned sensor actors. This does not affect the CARLA ticker.
            self.detection_pool.shutdown(wait=True, cancel_futures=True)
            self.detection_pool = None
        # Lifetime-zero markers otherwise survive a clean observer shutdown.
        # This acts only on our own marker namespace and never touches
        # the existing simulator, hero, cone actors or bridge publishers.
        # SIGINT may already have invalidated the ROS context by the time
        # the finally block runs. Marker cleanup must never prevent release
        # of the actual CARLA sensor actors. The next observer clears stale
        # visualization markers on its first published MarkerArray.
        if self.last_marker_ids and rclpy.ok():
            try:
                markers = MarkerArray()
                for ident in self.last_marker_ids:
                    gone = Marker()
                    gone.header.frame_id = "base_link"
                    gone.ns = "zed2i_detected_cones"
                    gone.id = ident
                    gone.action = Marker.DELETE
                    markers.markers.append(gone)
                self.markers_pub.publish(markers)
            except Exception:
                pass
        self.last_marker_ids.clear()
        for eye, actor in list(self.actors.items()):
            try:
                actor.stop()
                actor.destroy()
                self.get_logger().info(f"Removed owned {eye} sensor {actor.id}")
            except Exception as exc:
                self.get_logger().error(f"Failed to remove {eye} sensor: {exc}")
        self.actors.clear()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Zed2iStereoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
