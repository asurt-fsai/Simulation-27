# PC SLAM launchers

The existing PC CARLA session supervisor owns these six processes. Use their
independent System Manager switches during a live session. The scripts load the
common, Perception and SLAM overlays. They use the PC session ROS domain unless
an isolated test supplies its own ROS_DOMAIN_ID.

Launchers execute the installed nodes directly so the supervisor can track and
stop each exact process. They consume the bridge and sensor topics already
present. LeGO-LOAM publishes historical TF on /slam/lidar/tf and
/slam/lidar/tf_static. Its unvalidated frames stay outside the main TF tree.

The camera and fusion pose outputs remain conditional. Only the adapter
publishes normalized LiDAR odometry after its alignment checks pass. Neither
the mapper's return-to-start observations nor the adapter's test results
establish fully validated localization or loop closure.
