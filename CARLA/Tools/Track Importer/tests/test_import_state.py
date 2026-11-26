"""Ownership regression tests; no CARLA connection or actor mutation."""
import importlib.util
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock
from unittest.mock import Mock

FILE = Path(__file__).resolve().parents[1] / "import_cone_track.py"
SPEC = importlib.util.spec_from_file_location("asurt_cone_track_importer", FILE)
mod = importlib.util.module_from_spec(SPEC)
import sys
sys.modules[SPEC.name] = mod
SPEC.loader.exec_module(mod)


class Actor:
    def __init__(self, actor_id, role, kind):
        self.id = actor_id
        self.type_id = kind
        self.attributes = {"role_name": role}
        self.destroy = Mock(return_value=True)


class World:
    def __init__(self, actors):
        self.actors = actors
        self.worldmap = types.SimpleNamespace(name="Carla/Maps/silverstone/silverstone")
        self.frame = 10

    def get_map(self):
        return self.worldmap

    def get_actors(self):
        return self.actors

    def get_snapshot(self):
        return types.SimpleNamespace(frame=self.frame)

    def wait_for_tick(self, timeout=5.0):
        self.frame += 1
        return self.get_snapshot()


class FrameZeroWorld(World):
    """Model the real CARLA new-client actor cache: zero until first tick."""
    def __init__(self, actors, stalled=False):
        super().__init__(actors)
        self.frame = 0
        self.stalled = stalled
        self.wait_calls = 0

    def get_actors(self):
        return [] if self.frame == 0 else self.actors

    def wait_for_tick(self, timeout=5.0):
        self.wait_calls += 1
        if self.stalled:
            raise TimeoutError("external CARLA ticker unavailable")
        self.frame = 1234
        return self.get_snapshot()


class ImporterStateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / ".cone_track_actors.json"
        self.args = types.SimpleNamespace(host="127.0.0.1", port=2000)
        self.state = {
            "host": self.args.host, "port": self.args.port,
            "map": "Carla/Maps/silverstone/silverstone",
            "source": "sample.json",
            "actors": [
                {"id": 1731, "type_id": "static.prop.asurtbluecone",
                 "role_name": "asurt_cone_token_blue"},
                {"id": 1732, "type_id": "static.prop.asurtyellowcone",
                 "role_name": "asurt_cone_token_yellow"},
            ],
        }
        self.path.write_text(json.dumps(self.state))

    def test_recycled_sensor_id_archived_without_deletion(self):
        sensor = Actor(1731, "asurt_velodyne_vlp16", "sensor.lidar.ray_cast")
        world = World([sensor])
        self.assertTrue(mod.archive_stale_state(world, self.path, self.args))
        self.assertFalse(self.path.exists())
        archives = list(self.path.parent.glob(".cone_track_actors.json.stale-*.json"))
        self.assertEqual(len(archives), 1)
        self.assertEqual(json.loads(archives[0].read_text()), self.state)
        sensor.destroy.assert_not_called()

    def test_missing_all_actors_allows_clear_and_archives(self):
        mod.clear_managed(World([]), self.path, self.args)
        self.assertFalse(self.path.exists())
        self.assertEqual(len(list(self.path.parent.glob("*.stale-*.json"))), 1)
        # Repeated clear should be idempotent, not another error loop.
        mod.clear_managed(World([]), self.path, self.args)

    def test_live_cones_are_not_automatically_archived(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        self.assertFalse(mod.archive_stale_state(World([cone]), self.path, self.args))
        self.assertTrue(self.path.exists())
        cone.destroy.assert_not_called()

    def test_frame_zero_cache_waits_for_external_tick_before_archival(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        world = FrameZeroWorld([cone])
        self.assertFalse(mod.archive_stale_state(world, self.path, self.args))
        self.assertEqual(world.wait_calls, 1)
        self.assertTrue(self.path.exists())
        cone.destroy.assert_not_called()

    def test_frame_zero_timeout_blocks_state_mutation(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        world = FrameZeroWorld([cone], stalled=True)
        with self.assertRaisesRegex(RuntimeError, "uninitialized"):
            mod.clear_managed(world, self.path, self.args)
        self.assertTrue(self.path.exists())
        cone.destroy.assert_not_called()

    def test_recover_exact_archived_track_before_clear(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        yellow = Actor(1732, "asurt_cone_token_yellow", "static.prop.asurtyellowcone")
        other = Actor(999, "hero", "vehicle.vehicle.asurt_fsai")
        archive = self.path.with_name(self.path.name + ".stale-20261122T173326Z-audit.json")
        self.path.rename(archive)
        world = FrameZeroWorld([cone, yellow, other])
        mod.clear_managed(world, self.path, self.args)
        self.assertEqual(world.wait_calls, 1)
        cone.destroy.assert_called_once()
        yellow.destroy.assert_called_once()
        other.destroy.assert_not_called()
        self.assertFalse(self.path.exists())
        self.assertEqual(json.loads(archive.read_text()), self.state)

    def test_untracked_extra_cone_blocks_recovery_and_clear(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        yellow = Actor(1732, "asurt_cone_token_yellow", "static.prop.asurtyellowcone")
        unknown = Actor(9000, "asurt_cone_foreign_blue", "static.prop.asurtbluecone")
        archive = self.path.with_name(self.path.name + ".stale-example.json")
        self.path.rename(archive)
        with self.assertRaisesRegex(RuntimeError, "without an active"):
            mod.clear_managed(World([cone, yellow, unknown]), self.path, self.args)
        self.assertFalse(self.path.exists())
        self.assertTrue(archive.exists())
        for actor in (cone, yellow, unknown):
            actor.destroy.assert_not_called()

    def test_missing_state_with_live_tagged_cones_blocks_spawn(self):
        self.path.unlink()
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        args = types.SimpleNamespace(host=self.args.host, port=self.args.port,
                                     track="new.csv", allow_neutral_props=False)
        with self.assertRaisesRegex(RuntimeError, "live importer-tagged cones"):
            mod.spawn_cones({}, [], args, None, World([cone]), self.path)
        cone.destroy.assert_not_called()
        self.assertFalse(self.path.exists())

    def test_spawn_reconciles_stale_before_creating_new_record(self):
        sensor = Actor(1731, "asurt_velodyne_vlp16", "sensor.lidar.ray_cast")
        args = types.SimpleNamespace(host=self.args.host, port=self.args.port,
                                     track="fresh_track.json", allow_neutral_props=False)
        mod.spawn_cones({}, [], args, None, World([sensor]), self.path)
        self.assertTrue(self.path.exists())
        current = json.loads(self.path.read_text())
        self.assertEqual(current["actors"], [])
        self.assertTrue(current["source"].endswith("fresh_track.json"))
        self.assertEqual(len(list(self.path.parent.glob("*.stale-*.json"))), 1)
        sensor.destroy.assert_not_called()

    def test_spawn_refuses_to_overwrite_live_managed_cones(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        args = types.SimpleNamespace(host=self.args.host, port=self.args.port,
                                     track="new_track.json", allow_neutral_props=False)
        with self.assertRaisesRegex(RuntimeError, "live managed track state"):
            mod.spawn_cones({}, [], args, None, World([cone]), self.path)
        self.assertTrue(self.path.exists())
        cone.destroy.assert_not_called()

    def test_mixed_live_and_recycled_ids_refuses_any_deletion(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        sensor = Actor(1732, "asurt_zed2i_sim_left", "sensor.camera.rgb")
        with self.assertRaisesRegex(RuntimeError, "mixed/ambiguous"):
            mod.clear_managed(World([cone, sensor]), self.path, self.args)
        cone.destroy.assert_not_called()
        sensor.destroy.assert_not_called()
        self.assertTrue(self.path.exists())

    def test_saved_tag_at_unexpected_id_blocks_archival(self):
        cone = Actor(9000, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        with self.assertRaisesRegex(RuntimeError, "mixed/ambiguous"):
            mod.archive_stale_state(World([cone]), self.path, self.args)
        cone.destroy.assert_not_called()
        self.assertTrue(self.path.exists())

    def test_other_live_importer_track_blocks_stale_replacement(self):
        foreign = Actor(9001, "asurt_cone_different_blue", "static.prop.asurtbluecone")
        with self.assertRaisesRegex(RuntimeError, "mixed/ambiguous"):
            mod.archive_stale_state(World([foreign]), self.path, self.args)
        foreign.destroy.assert_not_called()
        self.assertTrue(self.path.exists())

    def test_other_cone_reuses_saved_id_and_blocks_archival(self):
        foreign = Actor(1731, "asurt_cone_different_blue", "static.prop.asurtbluecone")
        with self.assertRaisesRegex(RuntimeError, "mixed/ambiguous"):
            mod.archive_stale_state(World([foreign]), self.path, self.args)
        foreign.destroy.assert_not_called()
        self.assertTrue(self.path.exists())

    def test_active_cone_clear_only_destroys_owned_actor(self):
        cone = Actor(1731, "asurt_cone_token_blue", "static.prop.asurtbluecone")
        other = Actor(999, "hero", "vehicle.vehicle.asurt_fsai")
        mod.clear_managed(World([cone, other]), self.path, self.args)
        cone.destroy.assert_called_once_with()
        other.destroy.assert_not_called()
        self.assertFalse(self.path.exists())

    def test_wrong_map_or_server_preserves_state(self):
        w = World([])
        w.worldmap.name = "OtherMap"
        with self.assertRaisesRegex(RuntimeError, "map has changed"):
            mod.archive_stale_state(w, self.path, self.args)
        wrong = types.SimpleNamespace(host="127.0.0.2", port=2000)
        with self.assertRaisesRegex(RuntimeError, "different server"):
            mod.archive_stale_state(World([]), self.path, wrong)
        self.assertTrue(self.path.exists())

    def test_clearance_reuses_bbox_by_mesh_but_checks_every_cone(self):
        def vec(x=0.0, y=0.0):
            return types.SimpleNamespace(x=x, y=y)

        def rot(yaw=0.0):
            return types.SimpleNamespace(yaw=yaw, pitch=0.0, roll=0.0)

        def box(extent_x, extent_y):
            return types.SimpleNamespace(
                location=vec(), rotation=rot(), extent=vec(extent_x, extent_y))

        class BoxOwner:
            def __init__(self, typ, x, y, size, mesh=None):
                self.id = int(abs(x) * 100 + abs(y) * 10)
                self.type_id = typ
                self.attributes = {"mesh_path": mesh} if mesh else {}
                self.tf = types.SimpleNamespace(location=vec(x, y), rotation=rot())
                self.shape = box(size, size)
                self.box_queries = 0

            @property
            def bounding_box(self):
                self.box_queries += 1
                return self.shape

            def get_transform(self):
                return self.tf

        hero = BoxOwner("vehicle.vehicle.asurt_fsai", 0.0, 0.0, 1.0)
        # One generic prop type can spawn cones with different mesh bounds.
        meshes = ("ASURTBlueCone", "ASURTYellowCone", "ASURTBigOrangeCone")
        cones = [BoxOwner("static.prop.mesh", 25.0 + i, 1.0, .14, meshes[i % 3])
                 for i in range(470)]
        tf = types.SimpleNamespace(location=vec(), rotation=rot())
        args = types.SimpleNamespace(state=self.path, host=self.args.host,
                                     port=self.args.port)
        with mock.patch.object(mod, "owned_actors", return_value=cones):
            mod.validate_car_start(None, hero, tf, args)
        self.assertEqual(hero.box_queries, 1)
        self.assertEqual(sum(c.box_queries for c in cones), 3)

        # The optimization must still reject a real cone crossing the car.
        cones[-1].tf.location = vec(.3, .2)
        with mock.patch.object(mod, "owned_actors", return_value=cones):
            with self.assertRaisesRegex(RuntimeError, "overlaps cone"):
                mod.validate_car_start(None, hero, tf, args)


if __name__ == "__main__":
    unittest.main()
