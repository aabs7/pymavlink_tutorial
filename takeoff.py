# Import mavutil
from pymavlink import mavutil

# Create the connection
# From topside computer
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Wait a heartbeat before sending commands
master.wait_heartbeat()

mavutil.mavfile.set_mode(master,'GUIDED',0,0)

# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)
altitude = 10
#mavutil.mavfile.set_mode(master,'STABILIZE',0,0)
master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, altitude)

mavutil.mavfile.set_mode(master,'AUTO',0,0)
input()