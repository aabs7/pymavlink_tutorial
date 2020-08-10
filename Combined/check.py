from drone import Drone

drone = Drone('tcp:127.0.0.1:5762')
#drone.arm()
drone.arm_and_takeoff(10,auto_mode=False)
#drone.mission_upload()
input()

