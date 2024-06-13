from pymavlink import mavutil

# Connect to the drone
connection_string = 'udp:localhost:14570'  # or '/dev/ttyAMA0', 57600 for serial connection
master = mavutil.mavlink_connection(connection_string, source_system=244) #

# Wait for the first heartbeat 
#
master.wait_heartbeat()
master.target_system = 1
master.target_component = 0
print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))

master.mav.heartbeat_send(
    type=mavutil.mavlink.MAV_TYPE_GCS,
    autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID, #MAV_AUTOPILOT_INVALID
    base_mode=0,
    custom_mode=0,
    system_status=mavutil.mavlink.MAV_STATE_UNINIT, #MAV_STATE_UNINIT
    # mavlink_version=3
)

# Define the tracking point parameters
point_x = 0.5  # Example value: center of the image (normalized 0..1, 0 is left, 1 is right)
point_y = 0.5  # Example value: center of the image (normalized 0..1, 0 is top, 1 is bottom)
radius = 0.1  # Example value: small radius (normalized 0..1, 0 is one pixel, 1 is full image width)

# Send the MAV_CMD_CAMERA_TRACK_POINT command

master.mav.command_long_send(
    master.target_system, # Target system
    255, # Target component (e.g., camera)
    mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT,  # Command
    0,                       # Confirmation
    point_x,                 # Param 1: Point x value (normalized 0..1)
    point_y,                 # Param 2: Point y value (normalized 0..1)
    radius,                  # Param 3: Point radius (normalized 0..1)
    0,                       # Param 4 (unused)
    0,                       # Param 5 (unused)
    0,                       # Param 6 (unused)
    0                        # Param 7 (unused)
)

print("MAV_CMD_CAMERA_TRACK_POINT command sent with parameters: x={}, y={}, radius={}".format(point_x, point_y, radius))

