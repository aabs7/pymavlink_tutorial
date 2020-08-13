from drone import Drone
import time

drone = Drone('tcp:127.0.0.1:5762')
#drone.arm()
#drone.arm_and_takeoff(10,auto_mode=False)
waypoints = drone.mission_read()
print(waypoints)

#input()
# print("connecting !!!")
# time.sleep(1)
# print("Connected!!!")
# while True:
#     print("Battery:",drone.battery.voltage)
#     print("gs:",drone.groundspeed)
#     #print(drone.location)
#     print("Arm:",drone.is_armed)
#     print("is_armable:",drone.is_armable)
#     print("Ekf ok?",drone.ekf_ok)
#     print("status:",drone.system_status)
#     print("Mode:",drone.flight_mode)
#     print("\r\n")
#     time.sleep(1)

