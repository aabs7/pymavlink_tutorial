# Import mavutil
from pymavlink import mavutil
from pymavlink import mavwp

import time

#Import threading for reading mavlink messages
import threading



class Drone():
    def __init__(self,port):
        #start connection on the given port
        self.master = mavutil.mavlink_connection(port)
        self._home = None
        self.MAV_STATE = {
                0 : 'UNINIT',
                1 : 'BOOT',
                2 : 'CALIBRATING',
                3 : 'STANDBY',
                4 : 'ACTIVE',
                5 : 'CRITICAL',
                6 : 'EMERGENCY',
                7 : 'POWEROFF',
                8 : 'FLIGHT_TERMINATION'
            }
        
        ## The data coming from mavlink (not all the data are present)
        #'GLOBAL_POSITION_INT'
        self._lat = None
        self._lon = None
        self._alt = None
        self._relative_alt = None
        self._vx = None
        self._vy = None
        self._vz = None
        self._heading = None

        #'SYS_STATUS'
        self._voltage = None
        self._current = None
        self._level = None

        #'VFR_HUD'
        self._heading = None
        self._airspeed = None
        self._groundspeed = None
        self._throttle = None
        self._alt = None
        self._climb = None

        #'SERVO_OUTPUT_RAW'
        self._servo1_raw = None
        self._servo2_raw = None
        self._servo3_raw = None
        self._servo4_raw = None
        self._servo5_raw = None
        self._servo6_raw = None
        self._servo7_raw = None
        self._servo8_raw = None

        #'GPS_RAW_INIT'
        self._eph = None
        self._epv = None
        self._satellites_visible = None
        self._fix_type = None

        #'EKF_STATUS_REPORT'
        self._ekf_flag = None

        #'LOCAL_POSITION_NED'
        self._north = None
        self._east = None
        self._down = None
        

        #start new thread for getting data whenever object is called
        self.data_read = threading.Thread(target = self.update)
        self.data_read.start()
    
    def update(self):
        while True:
            msg = self.master.recv_match()
            if not msg:
                continue

        
    def set_flight_mode(self,mode):
        self.master.wait_heartbeat()
        mavutil.mavfile.set_mode(self.master,mode,0,0)
    
    def arm(self):
        self.master.mav.command_long_send(self.master.target_system,
                                    self.master.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                    0,
                                    1, 0, 0, 0, 0, 0, 0)
    
    def disarm(self):
        self.master.mav.command_long_send(self.master.target_system,
                                    self.master.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                    0,
                                    0, 0, 0, 0, 0, 0, 0)
    
    def arm_and_takeoff(self,altitude,auto_mode = True):
        self.set_flight_mode('GUIDED')
        self.arm()
        self.master.mav.command_long_send(0, 0, 
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                                            ,0, 0, 0, 0, 0, 0, 0, altitude)

        if(auto_mode):
            self.set_flight_mode('AUTO')

    def set_home(self,home_location=None):
    
        if home_location == None:
            home_location = self._home

        self.master.mav.command_long_send(
                        self.master.target_system, self.master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                        1, # set position
                        0, # param1
                        0, # param2
                        0, # param3
                        0, # param4
                        home_location[0], # lat
                        home_location[1], # lon
                        home_location[2])  # alt

    def mission_read(self, file_name = 'mission.txt'):
        self.master.wait_heartbeat()

        #ask for mission count
        self.master.waypoint_request_list_send()
        msg = self.master.recv_match(type = ['MISSION_COUNT'],blocking = True)
        waypoint_count = msg.count
        print("msg.count:",waypoint_count)

        output = 'QGC WPL 110\n'

        for i in range(waypoint_count):
            #ask for individual waypoint
            self.master.waypoint_request_send(i)
            #wait for receive mavlink msg type MISSION_ITEM 
            msg = self.master.recv_match(type=['MISSION_ITEM'],blocking = True)
            
            #commandline is used to store msg in a given format
            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (msg.seq,msg.current,msg.frame,msg.command,msg.param1,msg.param2,msg.param3,msg.param4,msg.x,msg.y,msg.z,msg.autocontinue)
            output += commandline

        #write to file
        with open(file_name,'w') as file_:
            print("Write mission to file")
            file_.write(output)

    

    def mission_upload(self, file_name = 'mission.txt'):
        self.master.wait_heartbeat()
        wp = mavwp.MAVWPLoader()
        with open(file_name) as f:
            for i, line in enumerate(f):
                if i == 0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:   
                    linearray=line.split('\t')
                    ln_seq = int(linearray[0])
                    ln_current = int(linearray[1])
                    ln_frame = int(linearray[2])
                    ln_command = int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_x=float(linearray[8])
                    ln_y=float(linearray[9])
                    ln_z=float(linearray[10])
                    ln_autocontinue = float(linearray[11].strip())
                    if(i == 1):
                        self._home = (ln_x,ln_y,ln_z)
                    p = mavutil.mavlink.MAVLink_mission_item_message(self.master.target_system, self.master.target_component, ln_seq, ln_frame,
                                                                    ln_command,
                                                                    ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                    wp.add(p)
        
        #while uploading mission, first home should be given
        self.set_home()
        msg = self.master.recv_match(type = ['COMMAND_ACK'],blocking = True)
        print(msg)
        print('Set home location: {0} {1}'.format(self._home[0],self._home[1]))
        time.sleep(1)

        #send waypoint to airframe
        self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wp.count())

        for i in range(wp.count()):
            msg = self.master.recv_match(type=['MISSION_REQUEST'],blocking=True)
            print(msg)
            self.master.mav.send(wp.wp(msg.seq))
            print('Sending waypoint {0}'.format(msg.seq))





    
