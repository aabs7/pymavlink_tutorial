# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

mavutil.mavfile.set_mode(master,'GUIDED',0,0)