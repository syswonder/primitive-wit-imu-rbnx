# Bundled WIT driver

wit_ros2_imu/{node.py,protocol.py,__init__.py} are copied unchanged from the existing
WIT project on ranger, WitImu_ws/src/wit_ros2_imu/wit_ros2_imu/ (2026-11-04).
The original package.xml identifies the driver license as MIT. No device
configuration or runtime data is copied. The original workspace remains intact.

The wrapper supplies private ROS parameter files and remappings; no external
workspace is required. test_protocol.py is adapted from that project's existing tests.
