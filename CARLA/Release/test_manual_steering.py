"""Exercise keyboard parsing without a ROS installation or a driving window."""

import ast
from collections import defaultdict
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from build_release import DRIVING_SOURCE, REPOSITORY


class KeyboardSteeringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = REPOSITORY / DRIVING_SOURCE
        parsed = ast.parse(path.read_text(encoding="utf-8"))
        keyboard = next(node for node in parsed.body if isinstance(node, ast.ClassDef)
                        and node.name == "KeyboardControl")
        parser = next(node for node in keyboard.body if isinstance(node, ast.FunctionDef)
                      and node.name == "_parse_vehicle_keys")
        keys = "UP w DOWN s LEFT a RIGHT d SPACE".split()
        namespace = {f"K_{key}": key for key in keys}
        exec(compile(ast.Module(body=[parser], type_ignores=[]), str(path), "exec"), namespace)
        cls.parse = staticmethod(namespace[parser.name])
        events = next(node for node in keyboard.body if isinstance(node, ast.FunctionDef)
                      and node.name == "parse_events")
        namespace.update(
            carla=SimpleNamespace(VehicleControl=SimpleNamespace,
                                  VehicleLightState=SimpleNamespace(Brake=1, Reverse=2)),
            pygame=SimpleNamespace(event=SimpleNamespace(get=lambda: []),
                                   key=SimpleNamespace(get_pressed=lambda: [])),
            rclpy=SimpleNamespace(spin_once=lambda *args, **kwargs: None),
            math=math,
        )
        exec(compile(ast.Module(body=[events], type_ignores=[]), str(path), "exec"), namespace)
        cls.events = staticmethod(namespace[events.name])

    def test_both_input_modes_reach_the_signed_limit_over_1500_ms(self):
        for ackermann in (False, True):
            for key, sign in (("a", -1), ("d", 1)):
                with self.subTest(ackermann=ackermann, key=key):
                    state = SimpleNamespace(_steer_cache=0.0, _ackermann_enabled=ackermann,
                                            _control=SimpleNamespace(), _max_steer_rad=math.radians(30),
                                            _ackermann_control=SimpleNamespace())
                    keys = defaultdict(bool, {key: True})
                    command = state._ackermann_control if ackermann else state._control
                    limit = math.radians(30) if ackermann else 1.0
                    for _ in range(75):
                        self.parse(state, keys, 10)
                    self.assertAlmostEqual(command.steer, sign * limit / 2)
                    for _ in range(75):
                        self.parse(state, keys, 10)
                    self.assertAlmostEqual(command.steer, sign * limit)
                    self.parse(state, keys, 50)
                    self.assertEqual(command.steer, sign * limit)
                    self.parse(state, defaultdict(bool), 10)
                    self.assertEqual(command.steer, 0.0)

    def test_manual_accelerator_obeys_speed_ceiling(self):
        for ackermann in (False, True):
            for speed in (29.0, 30.0, 31.0):
                with self.subTest(ackermann=ackermann, speed=speed):
                    state = SimpleNamespace(
                        _control=SimpleNamespace(throttle=1.0, brake=0, gear=1),
                        _ackermann_control=SimpleNamespace(speed=45.0),
                        _force_manual_control=True, _ackermann_enabled=ackermann,
                        _autopilot_enabled=False, _max_velocity_ms=30.0, _lights=0,
                        _ros_node=None, _parse_vehicle_keys=Mock(),
                    )
                    player = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=speed, y=0, z=0),
                                             get_control=Mock(), apply_control=Mock(),
                                             apply_ackermann_control=Mock())
                    self.events(state, None, SimpleNamespace(player=player, hud=Mock()),
                                SimpleNamespace(get_time=lambda: 10), False)
                    if ackermann:
                        player.apply_ackermann_control.assert_called_once_with(state._ackermann_control)
                        self.assertEqual(state._ackermann_control.speed, 30.0)
                    else:
                        player.apply_control.assert_called_once_with(state._control)
                        self.assertEqual(state._control.throttle, 1.0 if speed < 30 else 0.0)
