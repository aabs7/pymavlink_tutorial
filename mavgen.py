#import ardupilotmega module for mavlink1
from pymavlink.dialects.v10 import ardupilotmega as mavlink1

#import common module for mavlink 2
from pymavlink.dialects.v20 import common as mavlink2

from pymavlink import mavutil
import time
import sys

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_system))

msg = the_connection.recv_match(blocking=True)

while True:
    msg = the_connection.recv_match(blocking=True)

    print(the_connection.messages['GPS_RAW_INT'].alt )
    time.sleep(1)



the_connection.mav.heartbeat_send(
        6, # type
        8, # autopilot
        192, # base_mode
        0, # custom_mode
        4, # system_status
        3  # mavlink_version
)

the_connection.mav.command_long_send(
        1, # autopilot system id
        1, # autopilot component id
        400, # command id, ARM/DISARM
        0, # confirmation
        1, # arm!
        0,0,0,0,0,0 # unused parameters for this command
)

mode = 'STABILIZE'

# Check if mode is available
# if mode not in the_connection.mode_mapping():
#     print('Unknown mode : {}'.format(mode))
#     print('Try:', list(the_connection.mode_mapping().keys()))
#     sys.exit(1)


# Get mode ID
mode_id = 4
#mode_id = the_connection.mode_mapping()[mode]


# Set new mode
# the_connection.mav.command_long_send(
#    the_connection.target_system, the_connection.target_component,
#    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
#    0, mode_id, 0, 0, 0, 0, 0) or:
# the_connection.set_mode(mode_id) or:

# the_connection.mav.set_mode_send(
#     the_connection.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     mode_id)

mavutil.mavfile.set_mode(the_connection,4,0,0)

# while True:
#     # Wait for ACK command
#     ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
#     ack_msg = ack_msg.to_dict()

#     # Check if command in the same in `set_mode`
#     if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
#         continue

#     # Print the ACK result !
#     print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
#     break


altitude = 10

the_connection.mav.command_long_send(the_connection.target_system,
                                    the_connection.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                    0, 0, 0, 0, 0, 0, 0, altitude)

print("Before disarm")
time.sleep(20)
print("After disarm")

the_connection.mav.command_long_send(
                1, # autopilot system id
                1, # autopilot component id
                400, # command id, ARM/DISARM
                0, # confirmation
                0, # disarm!
                0,0,0,0,0,0 # unused parameters for this command
                )



####################################################
# the_connection.mav.command_long_send(
#     the_connection.target_system,
#     the_connection.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     1, 0, 0, 0, 0, 0, 0)

# altitude = 30

# the_connection.mav.command_long_send(the_connection.target_system,
#                                     the_connection.target_component, 
#                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#                                     0, 0, 0, 0, 0, 0, 0, altitude)

input()