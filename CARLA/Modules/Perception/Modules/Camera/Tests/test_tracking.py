import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / "Adapters/CARLA/Camera/asurt_zed2i_sim"))
from asurt_zed2i_sim.detection import Cone
from asurt_zed2i_sim.tracking import ConeTracker


def obj(color="blue", x=6.0, y=1.5, u=240):
    return Cone(color, 84.0, x, y, 0.0, (u, 180, 10, 20))


class ConeTrackingTests(unittest.TestCase):
    def test_single_frame_ghost_is_not_published(self):
        t = ConeTracker()
        self.assertEqual(t.update([obj()]), [])
        self.assertEqual(t.update([]), [])
        self.assertEqual(t.update([]), [])
        self.assertEqual(t.update([]), [])
        self.assertFalse(t.tracks)

    def test_confirmation_and_explicit_absence(self):
        t = ConeTracker()
        t.update([obj()])
        a = t.update([obj(x=6.1)])
        self.assertEqual(len(a), 1)
        self.assertEqual(a[0].color, "blue")
        self.assertEqual(t.update([]), [])  # Never publish stale detections
        b = t.update([obj(x=6.15)])
        self.assertEqual([p.id for p in a], [p.id for p in b])

    def test_image_order_reversal_keeps_stable_track_ids(self):
        t = ConeTracker()
        first = [obj("blue", 4.0, 1.4, 130), obj("yellow", 8.0, -1.5, 460)]
        t.update(first)
        a = t.update(first)
        b = t.update(list(reversed(first)))
        self.assertEqual({p.color: p.id for p in a},
                         {p.color: p.id for p in b})

    def test_odom_compensates_for_manual_car_movement(self):
        t = ConeTracker()
        # Stationary world cone at x=10, y=1 while the car moves forward.
        self.assertEqual(t.update([obj(x=10.0, y=1.0)], pose=(0, 0, 0)), [])
        a = t.update([obj(x=9.0, y=1.0)], pose=(1, 0, 0))
        self.assertEqual(len(a), 1)
        self.assertAlmostEqual(a[0].x, 9.0, places=4)
        b = t.update([obj(x=8.0, y=1.0)], pose=(2, 0, 0))
        self.assertEqual(b[0].id, a[0].id)
        self.assertAlmostEqual(b[0].x, 8.0, places=4)

    def test_large_disparity_jump_does_not_teleport_track(self):
        t = ConeTracker()
        t.update([obj(x=4.0)])
        a = t.update([obj(x=4.05)])
        self.assertEqual(len(a), 1)
        new = t.update([obj(x=20.0)])
        self.assertEqual(new, [])

    def test_duplicate_fragments_collapse(self):
        t = ConeTracker()
        same = [obj(x=4, u=130), obj(x=4.2, u=134)]
        t.update(same)
        published = t.update(same)
        self.assertEqual(len(published), 1)

    def test_rviz_holds_confirmed_cone_across_two_misses_only(self):
        t = ConeTracker()
        t.update([obj()], pose=(0, 0, 0), observed_at=10.0)
        fresh = t.update([obj(x=6.1)], pose=(0, 0, 0), observed_at=10.2)
        ident = fresh[0].id
        self.assertEqual(t.held_visualization({ident}, (0, 0, 0), now=10.2), [])
        self.assertEqual(t.update([], (0, 0, 0), observed_at=10.4), [])
        held = t.held_visualization({ident}, pose=(1, 0, 0), now=10.5)
        self.assertEqual([c.id for c in held], [ident])
        self.assertAlmostEqual(held[0].x, fresh[0].x - 1, places=4)
        self.assertEqual(t.update([], (1, 0, 0), observed_at=10.6), [])
        self.assertEqual([c.id for c in t.held_visualization({ident},
                         pose=(1, 0, 0), now=10.75)], [ident])
        self.assertEqual(t.held_visualization({ident}, pose=(1, 0, 0), now=11.1), [])
        self.assertEqual(t.update([], (1, 0, 0), observed_at=11.2), [])
        self.assertEqual(t.held_visualization({ident}, pose=(1, 0, 0), now=11.2), [])

    def test_visual_hold_requires_prior_display_and_confirmation(self):
        t = ConeTracker()
        t.update([obj()], observed_at=1.0)
        t.update([], observed_at=1.1)
        self.assertEqual(t.held_visualization({1}, now=1.2), [])
        t.update([obj()], observed_at=1.3)
        self.assertEqual(t.held_visualization({1}, now=1.4), [])
        t.update([], observed_at=1.45)
        self.assertEqual(t.held_visualization(set(), now=1.5), [])
        self.assertEqual(len(t.held_visualization({1}, now=1.5)), 1)
        self.assertEqual(t.held_visualization({1}, now=1.65), [])


if __name__ == "__main__":
    unittest.main()
