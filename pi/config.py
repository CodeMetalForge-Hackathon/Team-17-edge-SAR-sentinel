"""Shared config. Every coordinate is in the MAP frame = the LiDAR's own frame.

MAP frame: origin at the LiDAR center, +x = the LiDAR's 0 degree direction (forward),
+y = 90 degrees counter-clockwise from that (left). Units are meters.
Measure everything below with a tape measure from the LiDAR center.
"""

MQTT_HOST = "localhost"   # broker runs on the Pi (sudo apt install mosquitto)
MQTT_PORT = 1883

# --- Sensor nodes (Arduinos) ---------------------------------------------
# node_id -> where it sits on the map and which serial port it is on.
# Use /dev/serial/by-id/... paths (ls /dev/serial/by-id/): /dev/ttyUSB0 / ttyACM0 numbering can change
# between reboots, and the RPLIDAR is also a /dev/ttyUSB* device.
NODES = {
    "n1": {"x": 1.5, "y": 0.8, "port": "/dev/ttyAMA0"},   # UNO Q wired to the Pi 5 GPIO UART (pins 8/10)
    # "n2": {"x": 2.2, "y": -1.0, "port": "/dev/ttyACM1"},
}
SERIAL_BAUD = 115200
FEATURE_WINDOW_S = 1.0     # node features are aggregated over this window

# --- LiDAR -----------------------------------------------------------------
# RPLIDAR angles increase CLOCKWISE when viewed from above. Map angles are CCW.
LIDAR_ANGLE_SIGN = -1      # flip to +1 if a point to your left shows up on the right
LIDAR_ANGLE_OFFSET_DEG = 0.0
LIDAR_MAX_RANGE_M = 6.0
LIDAR_MIN_RANGE_M = 0.15
CHANGE_THRESH_M = 0.25     # a bin is "changed" if something is this much closer than baseline
CHANGE_MIN_BINS = 3        # ignore blobs narrower than this many degrees
CHANGE_PERSIST_SCANS = 3   # must show up in this many consecutive scans
BASELINE_FILE = "baseline_scan.npy"

# --- Camera (co-located with LiDAR, pointing along LiDAR 0 degrees) -------
CAMERA_HFOV_DEG = 66.0     # Pi Camera Module 3 standard ~66, v2 ~62.2, check yours
CAMERA_ANGLE_OFFSET_DEG = 0.0  # if the camera is not aimed exactly at LiDAR 0 deg
PERSON_MIN_CONF = 0.5

# --- Drone -----------------------------------------------------------------
CRAZYFLIE_URI = "radio://0/80/2M/E7E7E7E7E7"
DRONE_PAD = {"x": 0.5, "y": -0.5, "yaw_deg": 0.0}  # where the takeoff pad is on the map,
                                                   # yaw = direction the drone's nose faces
DRONE_HEIGHT_M = 0.5
DRONE_MAX_LEG_M = 3.0      # safety clamp: refuse targets farther than this from the pad
DRONE_HOVER_S = 3.0
DRONE_SPEED_MPS = 0.3      # cruise speed for each leg
DRONE_MIN_VBAT = 3.8       # refuse takeoff below this resting battery voltage (full = 4.2 V)
DRONE_ESTIMATOR_TIMEOUT_S = 10.0  # max wait for the Kalman estimate to settle before takeoff
DRONE_DRY_RUN = True       # set False only when organizers are present and the area is clear

# --- Brain / policy -------------------------------------------------------
NEAR_RADIUS_M = 0.6        # how close a person/LiDAR change must be to a node to be fused
EVENT_COOLDOWN_S = 15.0    # don't re-dispatch to the same spot within this time
AVOID_RADIUS_M = 0.7
MODEL_FILE = "node_iforest.joblib"
NORMAL_LOG = "normal_features.csv"
