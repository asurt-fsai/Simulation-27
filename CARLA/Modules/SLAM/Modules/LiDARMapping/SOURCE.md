# ASURT historical LeGO-LOAM provenance

The LeGO-LOAM and cloud_msgs source trees were extracted from the historical
ASURT repository (asurt-fsai/fs-system-26), branch Lego-Loam-cone-mapping,
commit 909b8f5626df2a8961d6c7a9a5dc6e3753dbf425. Its package metadata
declares the BSD license and retains the original authors.

The historical extensions include cone-map generation, cone-aware projection
and loop-closure signalling. The original launch file also starts additional
static TF publishers and RViz; use the isolated canonical launcher instead.

The default `config/asurt_vlp32c.yaml` assigns one range-image row to each
nominal VLP-32C beam. The `asurt_vlp16.yaml` profile remains available for the
16-channel scan. Both publish under `/slam/lidar`. Projected sensor clouds
retain the existing velodyne frame; internal LOAM pose-map frames use distinct
slam_lidar_* names. No additional static transform is created. Map-frame
alignment has to be evaluated before fused localization can be treated as
authoritative.

The original imageProjection.cpp was guarded against an empty sweep and
invalid ranges, which otherwise reach an unconditional front()/back()
access or asin division by zero. An isolated ROS test feeds empty and
synthetic VLP-16 and VLP-32C PointCloud2 messages and verifies the projected
rows in the velodyne frame. A separate domain-isolated test verifies
process reset on a simulated /odom clock rewind.

The local ROS Humble GTSAM package is unpacked in SLAM/ThirdParty without
modifying the host's system packages. Original historical build/install/log
trees were excluded from this extraction.
