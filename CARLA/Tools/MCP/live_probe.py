"""Bounded read-only checks against the active CARLA and ROS session."""

from collections import Counter
import math
import os
import time


def probe_carla(session):
    try:
        import carla

        client = carla.Client(session["host"], session["port"])
        client.set_timeout(1.5)
        world = client.get_world()
        map_name = world.get_map().name
        previous = world.get_snapshot()
        try:
            frame = world.wait_for_tick(.8)
        except RuntimeError:
            frame = world.get_snapshot()
        actors = world.get_actors()
        types = Counter(a.type_id.split(".", 1)[0] for a in actors)
        cones = sum(a.type_id.startswith("static.prop.") and a.attributes.get("role_name", "").startswith("asurt_cone_")
                    for a in actors)
        hero = next((a for a in actors.filter("vehicle.*") if a.attributes.get("role_name") == "hero"), None)
        snapshot = frame.find(hero.id) if hero is not None else None
        pose = None
        if snapshot is not None:
            location = snapshot.get_transform().location
            velocity = snapshot.get_velocity()
            pose = {"actor_id": hero.id, "x": location.x, "y": location.y, "z": location.z,
                    "speed_mps": round(math.hypot(velocity.x, velocity.y), 3)}
        settings = world.get_settings()
        return {"state": "responding", "map": map_name, "frame": frame.frame,
                "frame_advanced": frame.frame > previous.frame,
                "synchronous": settings.synchronous_mode, "fixed_delta_seconds": settings.fixed_delta_seconds,
                "actor_counts": dict(types), "managed_cones": cones, "hero": pose,
                "actor_snapshot_available": frame.frame > 0}
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        return {"state": "unavailable", "error": str(error)[:300]}


def probe_ros(session, modules):
    domain = int(session.get("ros_domain_id", os.environ.get("ROS_DOMAIN_ID", "46")))
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from rosgraph_msgs.msg import Clock

        context = Context()
        rclpy.init(args=[], context=context, domain_id=domain, signal_handler_options=SignalHandlerOptions.NO)
        node = None
        executor = None
        try:
            node = rclpy.create_node("asurt_mcp_status_probe", context=context,
                                     enable_rosout=False, start_parameter_services=False)
            samples = Counter()

            def sample(topic):
                def receive(_message):
                    samples[topic] += 1
                return receive

            node.create_subscription(Clock, "/clock", sample("/clock"), qos_profile_sensor_data)
            node.create_subscription(Odometry, "/odom", sample("/odom"), qos_profile_sensor_data)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            topics = sorted({"/clock", "/odom", *(module["topic"] for module in modules)})
            start = time.monotonic()
            deadline = start + 3.0
            while (remaining := deadline - time.monotonic()) > 0:
                executor.spin_once(timeout_sec=min(.15, remaining))
                if samples["/clock"] and samples["/odom"] and all(node.count_publishers(topic) for topic in topics):
                    break
            return {"domain_id": domain, "sample_window_s": round(time.monotonic() - start, 3),
                    "received_samples": {topic: samples[topic] for topic in ("/clock", "/odom")},
                    "publisher_counts": {topic: node.count_publishers(topic) for topic in topics}}
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown(context=context)
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        return {"domain_id": domain, "error": str(error)[:300]}
