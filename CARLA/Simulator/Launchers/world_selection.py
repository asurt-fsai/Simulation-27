"""Select the requested cooked world before starting the CARLA bridge."""


def select_world(client, world, world_map, requested):
    expected = requested.removeprefix("/Game/").strip("/")
    current = world_map.name.removeprefix("/Game/").strip("/")
    if current == expected:
        return world, world_map

    name = expected.rsplit("/", 1)[-1]
    matches = [item for item in client.get_available_maps() if item.strip("/").rsplit("/", 1)[-1] == name]
    exact = [item for item in matches if item.removeprefix("/Game/").strip("/") == expected]
    if len(exact) == 1:
        target = exact[0]
    elif len(matches) == 1:
        target = matches[0]
    else:
        raise RuntimeError(f"Unable to resolve {requested}: CARLA started on {world_map.name}; "
                           f"found {len(matches)} cooked maps named {name}.")

    accepted = {expected, target.removeprefix("/Game/").strip("/")}
    if len(matches) == 1:
        accepted.add(name)  # Some cooks expose only this map's short name.
    if current in accepted:
        return world, world_map

    client.set_timeout(90)
    try:
        world = client.load_world(target)
        world_map = world.get_map()
    finally:
        client.set_timeout(15)
    if world_map.name.removeprefix("/Game/").strip("/") not in accepted:
        raise RuntimeError(f"Expected {requested} after loading {target}, got {world_map.name}.")
    return world, world_map
