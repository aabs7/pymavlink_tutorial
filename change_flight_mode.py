# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

#mavutil.mavfile.set_mode(master,'GUIDED',0,0)

mode = 'LOITER'
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

while True:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    #print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break