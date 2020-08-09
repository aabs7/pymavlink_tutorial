"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
count = 0
# Get some information !
while True:
    msg = master.recv_match()
    if not msg:
        continue
    else:
        print(msg)
        if(msg.get_type() == 'ATTITUDE'):
            print("\r\n") 