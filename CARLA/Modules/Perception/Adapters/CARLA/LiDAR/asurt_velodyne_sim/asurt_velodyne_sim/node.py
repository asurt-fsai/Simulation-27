"""Passive Velodyne LiDAR simulation: CARLA -> /velodyne_points.

One logical scanner is attached to the EXISTING hero. No world tick, vehicle spawn,
weather edit, course modification, or /drive publication is performed.
Future Velodyne hardware or an updated processor can replace this publisher
without changing the downstream ROS PointCloud2 contract.
"""
from array import array as byte_array
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from .clock_alignment import ClockOffsetCalibrator
from .scan import decode_carla_lidar, RevolutionAccumulator
from .ultra_puck import beam_banks, rotate_to_sensor


def load_carla():
    try:
        import carla
        return carla
    except ImportError:
        site = os.environ.get(
            "ASURT_CARLA_SITE",
            "/home/hassan/ASURT/.venvs/asurt-native/lib/python3.10/site-packages",
        )
        if site not in sys.path:
            sys.path.append(site)
        import carla
        return carla


def ros_stamp(seconds):
    seconds = max(0.0, float(seconds))
    sec = math.floor(seconds)
    nanosec = round((seconds - sec) * 1e9)
    if nanosec >= 1000000000:
        sec, nanosec = sec + 1, nanosec - 1000000000
    return Time(sec=sec, nanosec=nanosec)


def to_cloud(points, stamp, frame_id):
    if points.dtype != np.float32 or points.ndim != 2 or points.shape[1] != 4:
        raise ValueError("LiDAR cloud must be contiguous float32 Nx4")
    points = np.ascontiguousarray(points, dtype="<f4")
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name=name, offset=4 * i,
                   datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(("x", "y", "z", "intensity"))
    ]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = 16 * len(points)
    cloud.is_dense = bool(np.isfinite(points).all())
    # Humble's Python byte-sequence setter is very slow for plain bytes.
    cloud.data = byte_array("B", points.tobytes())
    return cloud


class VelodyneSimNode(Node):
    def __init__(self):
        super().__init__("asurt_velodyne_sim_node")
        defaults = {
            "host": "127.0.0.1", "port": 2000, "role_name": "hero",
            "mount_x_m": 0.3755355179, "mount_y_m": 0.0100316303,
            "mount_z_m": 0.6965517998,
            "frame_id": "velodyne", "base_frame_id": "base_link",
            "topic": "/velodyne_points",
            "model": "vlp32c",
            "channels": 32, "range_m": 200.0,
            "upper_fov_deg": 15.0, "lower_fov_deg": -25.0,
            "rotation_hz": 10.0, "points_per_second": 600000,
            "noise_stddev_m": 0.02, "atmosphere_attenuation": 0.004,
            "dropoff_general_rate": 0.02,
            "dropoff_intensity_limit": 0.15, "dropoff_zero_intensity": 0.1,
            "publish_tf": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.validate()
        self.carla = load_carla()
        self.client = self.carla.Client(self.p["host"], self.p["port"])
        self.client.set_timeout(8.0)
        self.world = self.client.get_world()
        self.world.wait_for_tick(15)
        heroes = [
            actor for actor in self.world.get_actors().filter("vehicle.*")
            if actor.attributes.get("role_name") == self.p["role_name"]
        ]
        if len(heroes) != 1 or heroes[0].type_id != "vehicle.vehicle.asurt_fsai":
            raise RuntimeError("Require exactly one existing ASURT FSAI hero")
        self.hero = heroes[0]
        if not self.world.get_settings().synchronous_mode:
            self.get_logger().warning("World is not synchronous; still never ticking it")
        # An existing bridge or previous adapter owns the LiDAR until its
        # process is stopped. Never create a second lidar without inspection.
        lidar_actors = [a for a in self.world.get_actors()
                        if a.type_id.startswith("sensor.lidar.")]
        if lidar_actors:
            raise RuntimeError(f"Live LiDAR actor(s) already exist: {[a.id for a in lidar_actors]}")

        self.publisher = self.create_publisher(PointCloud2, self.p["topic"], 5)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.clock_calibration = ClockOffsetCalibrator()
        self.clock_offset = None
        self.accumulator = RevolutionAccumulator(self.p["rotation_hz"])
        self.lock = threading.Lock()
        self.queue = []
        self.sensors = []
        self.sensor_ids = []
        self.pending = {}
        self.last_frame = -1
        self.banks = (beam_banks() if self.p["model"] == "vlp32c" else
                      ((0.0, tuple(np.linspace(self.p["lower_fov_deg"],
                                                self.p["upper_fov_deg"], self.p["channels"]))),))
        self.destroyed = False
        self.segments_received = 0
        self.scans_received = 0
        self.scans_published = 0
        self.scans_dropped = 0
        self.points_last = 0
        self.last_report = time.monotonic()
        self.last_report_scans = 0
        self.last_report_segments = 0
        self.last_report_published = 0
        self.tf_broadcaster = StaticTransformBroadcaster(self) if self.p["publish_tf"] else None
        if self.tf_broadcaster:
            self.publish_mount_tf()
        try:
            self.spawn_sensor()
        except Exception:
            self.cleanup()
            raise
        self.create_timer(0.025, self.publish_queue)
        self.create_timer(4.0, self.report)
        self.get_logger().info(
            f"Velodyne {self.p['model']} source: existing hero={self.hero.id}, "
            f"{len(self.banks)} ray-cast banks, {self.p['channels']} channels, "
            f"{self.p['rotation_hz']} Hz, {self.p['points_per_second']} pts/s, "
            f"output={self.p['topic']} frame={self.p['frame_id']}, "
            f"CARLA hero-relative mount=({self.p['mount_x_m']:.4f}, "
            f"{-self.p['mount_y_m']:.4f}, {self.p['mount_z_m']:.4f})m; no world tick")

    def validate(self):
        p = self.p
        if p["model"] not in ("vlp32c", "uniform"):
            raise ValueError("unknown LiDAR model; use vlp32c or uniform")
        if p["model"] == "vlp32c" and (
            p["channels"] != 32 or (p["lower_fov_deg"], p["upper_fov_deg"]) != (-25.0, 15.0)
        ):
            raise ValueError("VLP-32C has 32 fixed beams covering -25 to +15 degrees")
        if p["channels"] < 1 or p["channels"] > 128:
            raise ValueError("invalid LiDAR channel count")
        if p["range_m"] <= 0 or p["rotation_hz"] <= 0 or p["points_per_second"] <= 0:
            raise ValueError("invalid LiDAR range, rotation, or point rate")
        if not -90 <= p["lower_fov_deg"] < p["upper_fov_deg"] <= 90:
            raise ValueError("invalid LiDAR elevation FOV")
        if not p["topic"].startswith("/") or not p["frame_id"] or not p["base_frame_id"]:
            raise ValueError("invalid ROS LiDAR topic/frame")
        if p["frame_id"] == p["base_frame_id"]:
            raise ValueError("LiDAR frame cannot equal base frame")

    def publish_mount_tf(self):
        tr = TransformStamped()
        tr.header.frame_id = self.p["base_frame_id"]
        tr.child_frame_id = self.p["frame_id"]
        tr.transform.translation.x = float(self.p["mount_x_m"])
        # Config stores ROS +y-left, while CARLA uses +y-right.
        tr.transform.translation.y = float(self.p["mount_y_m"])
        tr.transform.translation.z = float(self.p["mount_z_m"])
        tr.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tr)

    def spawn_sensor(self):
        settings = {
            "range": self.p["range_m"],
            "rotation_frequency": self.p["rotation_hz"],
            "noise_stddev": self.p["noise_stddev_m"],
            "atmosphere_attenuation_rate": self.p["atmosphere_attenuation"],
            "dropoff_general_rate": self.p["dropoff_general_rate"],
            "dropoff_intensity_limit": self.p["dropoff_intensity_limit"],
            "dropoff_zero_intensity": self.p["dropoff_zero_intensity"],
            "sensor_tick": 0.0,  # every server tick => real scanning sectors
            "ros_publish_tf": "false",  # this adapter is the single TF authority
        }
        library = self.world.get_blueprint_library()
        commands = []
        location = self.carla.Location(x=float(self.p["mount_x_m"]), y=-float(self.p["mount_y_m"]),
                                       z=float(self.p["mount_z_m"]))
        allocated = 0
        channels = 0
        for index, (yaw, elevations) in enumerate(self.banks):
            bp = library.find("sensor.lidar.ray_cast")
            channels += len(elevations)
            budget = self.p["points_per_second"] * channels // self.p["channels"]
            values = dict(settings, channels=len(elevations), points_per_second=budget - allocated,
                          upper_fov=max(elevations), lower_fov=min(elevations),
                          # Preserve the established readiness role for old/local supervisors.
                          role_name="asurt_velodyne_vlp16" if index == 0 else f"asurt_vlp32c_bank_{index}")
            allocated = budget
            for key, value in values.items():
                if bp.has_attribute(key):
                    bp.set_attribute(key, str(value))
            transform = self.carla.Transform(location, self.carla.Rotation(yaw=float(yaw)))
            commands.append(self.carla.command.SpawnActor(bp, transform, self.hero.id))
        responses = self.client.apply_batch_sync(commands, False)
        errors = []
        for response in responses:
            if response.error:
                errors.append(response.error)
            else:
                self.sensor_ids.append(response.actor_id)
        if errors or len(self.sensor_ids) != len(self.banks):
            raise RuntimeError(f"Could not create all LiDAR banks: {errors}")
        by_id = {actor.id: actor for actor in self.world.get_actors(self.sensor_ids)}
        self.sensors = [by_id[actor_id] for actor_id in self.sensor_ids]
        for index, sensor in enumerate(self.sensors):
            sensor.listen(lambda measurement, bank=index: self.on_scan_segment(measurement, bank))

    def on_clock(self, msg):
        # Target 21 distinct-frame pairs, spaced by ~0.1 s. No reads after lock.
        now = time.monotonic()
        if not self.clock_calibration.due(now):
            return
        try:
            snapshot = self.world.get_snapshot()
        except Exception as exc:
            self.clock_calibration.defer(now)
            self.get_logger().warning(f"LiDAR clock calibration snapshot unavailable: {exc}")
            return
        epoch = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock_calibration.observe(
            epoch, snapshot.timestamp.elapsed_seconds, now, snapshot.frame
        ):
            self.clock_offset = self.clock_calibration.offset
            self.get_logger().info(
                f"LiDAR clock epoch offset locked to {self.clock_offset:.6f}s "
                f"from {len(self.clock_calibration.samples)} paired samples "
                f"over {self.clock_calibration.window_seconds:.2f}s"
            )

    def on_scan_segment(self, measurement, bank):
        if self.destroyed:
            return
        try:
            points = decode_carla_lidar(measurement.raw_data)
            if self.banks[bank][0]:
                rotate_to_sensor(points, self.banks[bank][0])
            with self.lock:
                frame = measurement.frame
                if frame <= self.last_frame:
                    return
                parts = self.pending.setdefault(frame, {})
                parts[bank] = points
                if len(parts) != len(self.banks):
                    if len(self.pending) > 4:
                        del self.pending[min(self.pending)]
                    return
                self.last_frame = frame
                self.pending = {key: value for key, value in self.pending.items() if key > frame}
                self.segments_received += 1
                points = np.concatenate([parts[index] for index in range(len(self.banks))])
                result = self.accumulator.add(measurement.timestamp, points)
                if result is not None:
                    self.scans_received += 1
                    # A slow ROS consumer must not accumulate stale scans.
                    if len(self.queue) >= 2:
                        self.queue.pop(0)
                        self.scans_dropped += 1
                    self.queue.append(result)
        except Exception as exc:
            self.get_logger().warning(f"Velodyne callback skipped segment: {exc}")

    def publish_queue(self):
        # Don't substitute a /clock receive time for an actual scan capture
        # timestamp while initial epoch alignment is still in progress.
        if self.clock_offset is None:
            return
        with self.lock:
            if not self.queue:
                return
            scan_stamp, cloud_points = self.queue.pop(-1)
            self.scans_dropped += len(self.queue)
            self.queue.clear()
        stamp = ros_stamp(scan_stamp + self.clock_offset)
        self.publisher.publish(to_cloud(cloud_points, stamp, self.p["frame_id"]))
        self.scans_published += 1
        self.points_last = len(cloud_points)

    def report(self):
        now = time.monotonic()
        interval = max(now - self.last_report, 0.001)
        self.get_logger().info(
            f"Velodyne: segments={(self.segments_received-self.last_report_segments)/interval:.1f}Hz, "
            f"revolutions={(self.scans_received-self.last_report_scans)/interval:.1f}Hz, "
            f"ROS={(self.scans_published-self.last_report_published)/interval:.1f}Hz, "
            f"last_points={self.points_last}, dropped={self.scans_dropped}, "
            f"late_segments={self.accumulator.out_of_order}")
        self.last_report = now
        self.last_report_segments = self.segments_received
        self.last_report_scans = self.scans_received
        self.last_report_published = self.scans_published

    def cleanup(self):
        if self.destroyed:
            return
        self.destroyed = True
        for actor in self.sensors:
            try:
                actor.stop()
            except Exception as exc:
                self.get_logger().warning(f"Velodyne stop: {exc}")
        if self.sensor_ids:
            try:
                replies = self.client.apply_batch_sync(
                    [self.carla.command.DestroyActor(actor_id) for actor_id in self.sensor_ids], False)
                for actor_id, reply in zip(self.sensor_ids, replies):
                    if reply.error:
                        self.get_logger().warning(f"Velodyne actor {actor_id}: {reply.error}")
            except Exception as exc:
                self.get_logger().error(f"Velodyne banks require cleanup: {exc}")
        self.sensors.clear()
        self.sensor_ids.clear()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelodyneSimNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
