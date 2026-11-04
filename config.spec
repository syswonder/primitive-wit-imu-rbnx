# Driver CMD_INIT configuration; independent of body/URDF and external ROS workspaces.
# manage_driver: bool, default true. false attaches to already running WIT topics.
# serial.port: absolute device path or auto (default). Prefer /dev/serial/by-id.
# serial.baud: positive integer, default 9600; must match the receiver.
# serial.usb_vid / usb_pid: optional integers [0,65535], supplied together.
# In auto mode explicit USB IDs select exactly one matching device; otherwise use
# /dev/wit_imu, /dev/imu_usb or by-id names containing WIT/WT901/IMU.
# Generic CP2102 VID/PID are not enabled by default: they do not identify a WIT sensor.
# frame_id: nonempty string, default wit_10axis_link; deployment supplies TF as needed.
# sentinel_timeout_s: number (0,120], seconds, default 10.0.
# topics: optional absolute ROS topic overrides, all must be distinct:
#   imu: /wit_10axis/data
#   magnetic_raw: /wit_10axis/magnetic_raw
#   pressure: /wit_10axis/pressure
#   barometric_altitude: /wit_10axis/barometric_altitude
#   status: /wit_10axis/status
# External mode rejects serial/frame overrides. Restart provider to change topic endpoints.
