# Import mavutil
from pymavlink import mavutil
from pymavlink import mavwp
import time

# Create the connection
# From topside computer
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

master.wait_heartbeat()

master.waypoint_request_list_send()
waypoint_count = 0

msg = master.recv_match(type=['MISSION_COUNT'],blocking=True)
waypoint_count = msg.count
print("msg count:",waypoint_count)

for i in range(waypoint_count):
    master.waypoint_request_send(i)
    msg = master.recv_match(type=['MISSION_ITEM'],blocking = True)
    print("Receiving waypoint {0}".format(msg.seq))
    print(msg)