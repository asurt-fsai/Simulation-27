"""Identify physical course cones in both current and older CARLA sessions."""


def course_cone_color(actor):
    if actor.type_id == "static.prop.mesh":
        mesh = actor.attributes.get("mesh_path", "")
        if not mesh.startswith("/Game/ASURTCones/"):
            return None
        kind = mesh.rsplit(".", 1)[-1].lower()
    else:
        kind = actor.type_id.removeprefix("static.prop.")
    return {"asurtbluecone": "blue", "asurtyellowcone": "yellow"}.get(kind)
