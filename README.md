# pymavlink_tutorial
This repo contains code for connecting drone with pymavlink. All the drones that uses mavlink message can be controlled by codes in this repository.  
```drone.py``` under ```Combined``` folder is the implementation of ```prokura_drone``` api which is high level API to control drones. 

## Note:
THIS API IS TESTED IN ARDUPILOT AND WORKS WELL WITH ARDUPILOT FLIGHT STACK. HOWEVER, BECAUSE PX4 DOESN'T UTILIZE ALL THE MAVLINK MESSAGE USED IN ARDUPILOT, OR USES SOME MAVLINK MESSAGES DIFFERENT THAN ARDUPILOT, ERROR MAY OCCUR WHILE HANDLING MAVLINK MESSAGE IN PX4.
