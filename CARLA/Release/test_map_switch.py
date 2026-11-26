"""Exercise startup recovery with the map names reported by CARLA."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Simulator/Launchers"))
from map_packs import select_world


class MapSwitchTest(unittest.TestCase):
    flat = "/Game/Carla/Maps/FlatAsphalt"
    silverstone = "/Game/ASURTMaps/silverstone_section/SilverStoneSection"

    def test_recovers_flat_asphalt_when_silverstone_was_loaded_at_boot(self):
        client = Mock()
        client.get_available_maps.return_value = [
            "/Game/Other/FlatAsphalt", self.flat, self.silverstone,
        ]
        client.load_world.return_value.get_map.return_value = SimpleNamespace(name=self.flat)
        world, active = select_world(client, Mock(), SimpleNamespace(name=self.silverstone), self.flat)
        self.assertIs(world, client.load_world.return_value)
        self.assertEqual(active.name, self.flat)
        client.load_world.assert_called_once_with(self.flat)
        self.assertEqual(client.set_timeout.call_args_list, [call(90), call(15)])

    def test_keeps_the_requested_world_when_already_ready(self):
        client, world = Mock(), Mock()
        active = SimpleNamespace(name=self.flat)
        self.assertEqual(select_world(client, world, active, self.flat.removeprefix("/Game/")), (world, active))
        self.assertEqual(client.mock_calls, [])

    def test_resolves_single_short_name_for_a_loose_cook(self):
        client = Mock()
        client.get_available_maps.return_value = [self.flat, "SilverStoneSection"]
        client.load_world.return_value.get_map.return_value = SimpleNamespace(name="SilverStoneSection")
        _, active = select_world(client, Mock(), SimpleNamespace(name=self.flat), self.silverstone)
        self.assertEqual(active.name, "SilverStoneSection")
        client.load_world.assert_called_once_with("SilverStoneSection")

    def test_rejects_ambiguous_cook_when_the_expected_path_is_missing(self):
        client = Mock()
        client.get_available_maps.return_value = ["/Game/A/FlatAsphalt", "/Game/B/FlatAsphalt"]
        with self.assertRaisesRegex(RuntimeError, "found 2 cooked maps named FlatAsphalt"):
            select_world(client, Mock(), SimpleNamespace(name=self.silverstone), self.flat)
        client.load_world.assert_not_called()

    def test_restores_timeout_when_map_loading_fails(self):
        client = Mock()
        client.get_available_maps.return_value = [self.flat]
        client.load_world.side_effect = RuntimeError("RPC timed out")
        with self.assertRaisesRegex(RuntimeError, "RPC timed out"):
            select_world(client, Mock(), SimpleNamespace(name=self.silverstone), self.flat)
        self.assertEqual(client.set_timeout.call_args_list, [call(90), call(15)])


if __name__ == "__main__":
    unittest.main()
