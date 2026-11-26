"""Select a camera overlay frame without mixing raw and inferred images.

Camera captures can advance while a CPU detector is still working. The overlay
topic displays complete detection results; raw images have their own topics.
"""


def next_overlay_frame(displayed_frame, latest_raw, latest_detection, detection_enabled):
    """Return the next source snapshot, or None if none is ready.

    Both snapshots start with a CARLA frame number. A detection snapshot also
    carries its analyzed image, original ROS timestamp, arrival time and cones.
    A deliberately empty cone list is a completed result, not a missing one.
    """
    selected = latest_detection if detection_enabled else latest_raw
    if selected is None or selected[0] <= displayed_frame:
        return None
    return selected
