#!/usr/bin/python3
"""Run the July controller with paths already expressed in its target frame."""
from simple_pure_pursuit import simple_pp_node, simple_purepursuitcontroller


class LocalPathTF(simple_purepursuitcontroller.TFHelper):
    def transformMsg(self, message, target):
        if message.header.frame_id == target:
            # The original TF helper passes ROS's xyzw quaternion to a wxyz
            # API. Applying that conversion to identity reverses the path.
            return message
        return super().transformMsg(message, target)


if __name__ == "__main__":
    simple_purepursuitcontroller.TFHelper = LocalPathTF
    simple_pp_node.main()
