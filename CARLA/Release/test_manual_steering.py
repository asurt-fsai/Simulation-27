"""Exercise keyboard parsing without a ROS installation or a driving window."""

import ast
from collections import defaultdict
import math
from types import SimpleNamespace
import unittest

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
