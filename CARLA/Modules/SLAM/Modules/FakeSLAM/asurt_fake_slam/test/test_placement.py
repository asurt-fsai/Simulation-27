from types import SimpleNamespace

from asurt_msgs.msg import Landmark

from asurt_fake_slam.node import noisy_landmarks, placed_landmarks, update_observations, visible_landmarks


def record(actor_id, color, position=None):
    data = {"id": actor_id, "type_id": "static.prop.mesh",
            "role_name": f"asurt_cone_course_{color}", "color": color}
    if position is not None:
        data["position"] = position
    return data


def live(data, location=None):
    return SimpleNamespace(type_id=data["type_id"], attributes={"role_name": data["role_name"]},
                           get_location=lambda: location)


def test_coordinates_colors_and_deleted_actor():
    blue = record(1, "blue", [10, -3, .5])
    large = record(2, "orange_big", [12, 2, .6])
    actors = {1: live(blue), 2: live(large)}
    poses = {}
    result = placed_landmarks([blue, large], actors, poses)
    assert result[1][:3] == (10., 3., .5)
    assert result[1][3][0] == Landmark.BLUE_CONE
    assert result[2][:3] == (12., -2., .6)
    assert result[2][3][0] == Landmark.LARGE_CONE
    del actors[1]
    assert list(placed_landmarks([blue, large], actors, poses)) == [2]


def test_recycled_id_and_wrong_role_are_excluded():
    yellow = record(17, "yellow", [1, 2, 3])
    actor = live(yellow)
    actor.attributes["role_name"] = "asurt_cone_other_yellow"
    assert not placed_landmarks([yellow], {17: actor}, {})
    actor.attributes["role_name"] = yellow["role_name"]
    actor.type_id = "static.prop.asurtyellowcone"
    assert not placed_landmarks([yellow], {17: actor}, {})


def test_legacy_map_uses_first_pose_until_the_owned_actor_disappears():
    yellow = record(24, "yellow")
    position = SimpleNamespace(x=4., y=5., z=.3)
    actor = live(yellow, position)
    poses = {}
    assert placed_landmarks([yellow], {24: actor}, poses)[24][:3] == (4., -5., .3)
    position.x = 9.0
    assert placed_landmarks([yellow], {24: actor}, poses)[24][:3] == (4., -5., .3)
    assert not placed_landmarks([yellow], {}, poses)
    assert not poses


def test_first_sighting_accumulates_and_revisit_preserves_original_pose():
    owned = {4: record(4, "yellow", [30, 0, 0]), 5: record(5, "blue", [7, 0, 0])}
    actual = {4: SimpleNamespace(x=3, y=-1, z=.2), 5: SimpleNamespace(x=7, y=0, z=.2)}
    snapshot = SimpleNamespace(find=lambda actor_id: SimpleNamespace(
        get_transform=lambda: SimpleNamespace(location=actual[actor_id])) if actor_id in actual else None)
    hero = SimpleNamespace(location=SimpleNamespace(x=0., y=0.), rotation=SimpleNamespace(yaw=0.))

    mapped = update_observations(owned, snapshot, hero, {})
    assert set(mapped) == {4}
    assert mapped[4][1][:3] == (3., 1., .2)
    assert all(type(coordinate) is float for coordinate in mapped[4][1][:3])

    hero.location.x = 4.
    after_passing = update_observations(owned, snapshot, hero, mapped)
    assert set(after_passing) == {4, 5}
    first_cone = after_passing[4]

    actual[4].x = 2.
    hero.rotation.yaw = 180.
    assert update_observations(owned, snapshot, hero, after_passing) == after_passing
    assert update_observations(owned, snapshot, None, after_passing) == after_passing

    owned[4] = dict(owned[4], role_name="asurt_cone_replacement_yellow")
    replacement = update_observations(owned, snapshot, hero, after_passing)
    assert replacement[4] != first_cone
    assert replacement[4][1][:3] == (2., 1., .2)

    del actual[4]
    assert set(update_observations(owned, snapshot, hero, replacement)) == {5}
    assert not update_observations({}, snapshot, hero, replacement)


def test_forward_footprint_recomputes_range_and_heading():
    hero = SimpleNamespace(location=SimpleNamespace(x=10., y=2.),
                           rotation=SimpleNamespace(yaw=90.))
    points = {
        1: (10., -6.5, 0., "blue"),    # On the range boundary.
        2: (10., -6.51, 0., "blue"),   # Just beyond range.
        3: (10., 0., 0., "yellow"),    # Behind the car.
        4: (12., -2., 0., "yellow"),   # Exactly alongside.
        5: (11., -3., 0., "yellow"),   # Ahead and slightly to the side.
    }
    assert set(visible_landmarks(points, hero)) == {1, 5}
    hero.rotation.yaw = -90.
    assert set(visible_landmarks(points, hero)) == {3}
    hero.location.y = -10.
    assert not visible_landmarks(points, hero)
    assert not visible_landmarks(points, None)


def test_noise_offsets_are_stable_scale_with_setting_and_preserve_identity():
    points = {7: (4., 0., .2, "blue"), 12: (4., 0., .2, "yellow")}
    assert noisy_landmarks(points, 0, 13) == points
    noisy = noisy_landmarks(points, .1, 13)
    assert noisy == noisy_landmarks(points, .1, 13)
    assert noisy != noisy_landmarks(points, .1, 14)
    doubled = noisy_landmarks(points, .2, 13)
    assert set(noisy) == set(points)
    for actor_id, value in noisy.items():
        assert value[:2] != points[actor_id][:2]
        assert value[2:] == points[actor_id][2:]
        assert abs(doubled[actor_id][1] - 2 * value[1]) < 1e-9
