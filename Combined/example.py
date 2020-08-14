from drone import Drone,Location
import time

drone = Drone('tcp:127.0.0.1:5762')

time.sleep(1)

drone.mission_upload()

flight_plan = drone.flight_plan
print(flight_plan)
input()
#drone.arm()
#time.sleep(1)
#targetAltitude = 10
#drone.arm_and_takeoff(targetAltitude,auto_mode=True)


# while True:
#     #print("Altitude:",drone.location.altR)
#     if drone.location.altR > 0.95 * targetAltitude:
#         print("break")
#         break

# point1 = Location(-35.3625939,149.1650759,10)
# drone.simple_goto(point1)

#time.sleep(10)
#print("RTL set")
#drone.set_flight_mode('RTL')

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

