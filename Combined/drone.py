# Import mavutil
from pymavlink import mavutil
from pymavlink import mavwp
from pymavlink.dialects.v10 import ardupilotmega

import time

#Import threading for reading mavlink messages
import threading

class MavlinkMessage:
    def __init__(self,master):

        #master connection
        self.master = master
        
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
        self._ekf_poshorizabs = False
        self._ekf_constposmode = False
        self._ekf_predposhorizabs = False
        self._ekf_flag = None

        #'LOCAL_POSITION_NED'
        self._north = None
        self._east = None
        self._down = None

        #'HEARTBEAT'
        self._flightmode = None
        self._armed = False
        self._system_status = None
        self._autopilot_type = None  # PX4, ArduPilot, etc.
        self._vehicle_type = None  # quadcopter, plane, etc.

        #'ATTITUDE'
        self._roll = None
        self._pitch = None
        self._yaw = None
        self._rollspeed = None
        self._pitchspeed = None
        self._yawspeed = None

        #'MISSION_COUNT'
        self._msg_mission_count = None
        #'MISSION_ITEM'
        self._msg_mission_item = None
        #'COMMAND_ACK'
        self._msg_command_ack = None
        #'MISSION_REQUEST'
        self._msg_mission_request = None
     
        self.messages = {
            'GLOBAL_POSITION_INT'   :self.__read_global_pos_int,
            'SYS_STATUS'            :self.__read_system_status,
            'VFR_HUD'               :self.__read_vfr_hud,
            'SERVO_OUTPUT_RAW'      :self.__read_servo_output_raw,
            'GPS_RAW_INT'           :self.__read_gps_raw_int,
            'EKF_STATUS_REPORT'     :self.__read_ekf_status_report,
            'LOCAL_POSITION_NED'    :self.__read_local_position_ned,
            'HEARTBEAT'             :self.__read_heartbeat,
            'ATTITUDE'              :self.__read_attitude,
            #The variables for mavlink message listed below should be cleared once it is read.
            'MISSION_COUNT'         :self.__read_mission_count,
            'MISSION_ITEM'          :self.__read_mission_item,
            'MISSION_REQUEST'       :self.__read_mission_request,
            'COMMAND_ACK'           :self.__read_command_ack
        }

        #start new thread for getting data whenever object is called
        self.data_read = threading.Thread(target = self.__update)
        self.data_read.daemon = True    # In daemon mode so that ctrl + c will close the program
        self.data_read.start()
        
    def __update(self): 
        while True:
            #print("Here")
            msg = self.master.recv_match()

            if not msg:
                continue
            
            function = self.messages.get(msg.get_type(),lambda x:"Invalid")
            function(msg)
            

    def __read_global_pos_int(self,msg):
        self._lat = msg.lat * 1e-7
        self._lon = msg.lon * 1e-7
        self._alt = msg.alt * 1e-3
        self._relative_alt = msg.relative_alt * 1e-3
        self._vx = msg.vx
        self._vy = msg.vy
        self._vz = msg.vz
        self._heading = int(msg.hdg * 1e-2)

    def __read_system_status(self,msg):
        self._voltage = msg.voltage_battery
        self._current = msg.current_battery
        self._level = msg.battery_remaining

    def __read_vfr_hud(self,msg):
        self._airspeed = msg.airspeed
        self._groundspeed = msg.groundspeed
        self._throttle = msg.throttle
        self._alt = msg.alt
        self._climb = msg.climb
        
    def __read_servo_output_raw(self,msg):
        self._servo1_raw = msg.servo1_raw
        self._servo2_raw = msg.servo2_raw
        self._servo3_raw = msg.servo3_raw
        self._servo4_raw = msg.servo4_raw
        self._servo5_raw = msg.servo5_raw
        self._servo6_raw = msg.servo6_raw
        self._servo7_raw = msg.servo7_raw
        self._servo8_raw = msg.servo8_raw

    def __read_gps_raw_int(self,msg):
        self._eph = msg.eph
        self._epv = msg.epv
        self._satellites_visible = msg.satellites_visible
        self._fix_type = msg.fix_type

    def __read_ekf_status_report(self,msg):
        ekf_flags = msg.flags
        # boolean: EKF's horizontal position (absolute) estimate is good
        self._ekf_poshorizabs = (ekf_flags & ardupilotmega.EKF_POS_HORIZ_ABS) > 0
        # boolean: EKF is in constant position mode and does not know it's absolute or relative position
        self._ekf_constposmode = (ekf_flags & ardupilotmega.EKF_CONST_POS_MODE) > 0
        # boolean: EKF's predicted horizontal position (absolute) estimate is good
        self._ekf_predposhorizabs = (ekf_flags & ardupilotmega.EKF_PRED_POS_HORIZ_ABS) > 0
        


    def __read_local_position_ned(self,msg):
        self._north = msg.y
        self._east = msg.x
        self._down = msg.z

    def __read_heartbeat(self,msg):
        if self.master.probably_vehicle_heartbeat(msg):
            self._flightmode = mavutil.mode_mapping_acm[msg.custom_mode]
            self._armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self._system_status = msg.system_status
            self._autopilot_type = msg.autopilot
            self._vehicle_type = msg.type  # quadcopter, plane, etc.
            
    def __read_attitude(self,msg):
        self._roll = msg.roll
        self._pitch = msg.pitch
        self._yaw = msg.yaw
        self._rollspeed = msg.rollspeed
        self._pitchspeed = msg.pitchspeed
        self._yawspeed = msg.yawspeed

    def __read_mission_count(self,msg):
        self._msg_mission_count = msg

    def __read_mission_item(self,msg):
        self._msg_mission_item = msg

    def __read_mission_request(self,msg):
        self._msg_mission_request = msg
    
    def __read_command_ack(self,msg):
        self._msg_command_ack = msg
     


class Drone(MavlinkMessage):
    def __init__(self,port):
        #start connection on the given port
        self.master = mavutil.mavlink_connection(port)

        # store waypoints from the command
        self._waypoints = {}
        self._home = None

        #wait_heartbeat can be called before MavlinkMessage class initialization
        #after MavlinkMessage class initialization, due to thread for reading mavlink message, 
        #it is not advisable to even look for mavlink message inside methods of this class.
        #instead, check for the mavlink message inside MavlinkMessage class by adding the respective message.
        self.master.wait_heartbeat()

        #set home location when initializing
        msg = self.master.recv_match(type = 'GLOBAL_POSITION_INT',blocking = True)
        self._home = Location(msg.lat*1e-7,msg.lon*1e-7,msg.alt*1e-3,msg.relative_alt*1e-3)
        print("Home location set to lat = ", self._home.lat," lon = ",self._home.lon, "alt = ",self._home.alt)

        #read current mission
        self.mission_read()

        MavlinkMessage.__init__(self,self.master)
        
        
    def set_flight_mode(self,mode):
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
        armable = False
        while not armable:
            armable = self.is_armable
        self.set_flight_mode('GUIDED')
        self.arm()
        self.master.mav.command_long_send(0, 0, 
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                                            ,0, 0, 0, 0, 0, 0, 0, altitude)

        if(auto_mode):
            self.set_flight_mode('AUTO')
        
    def simple_goto(self,location):
        self.master.mav.mission_item_send(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                           mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0,
                                           0, 0, 0, location.lat, location.lon,
                                           location.alt)

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
                        home_location.lat, # lat
                        home_location.lon, # lon
                        home_location.alt)  # alt

    def mission_read(self, file_name = 'mission.txt',store_mission = True):
        #ask for mission count
        self.master.waypoint_request_list_send()

        #wait for receive mavlink msg type MISSION_COUNT
        msg = self.master.recv_match(type = ['MISSION_COUNT'],blocking = True)

        waypoint_count = msg.count
        print("msg.count:",waypoint_count)

        output = 'QGC WPL 110\n'
        mission_count = 0
        for i in range(waypoint_count):
            #ask for individual waypoint
            self.master.waypoint_request_send(i)
            #wait for receive mavlink msg type MISSION_ITEM 

            msg = self.master.recv_match(type = ['MISSION_ITEM'],blocking = True)
            
            #commandline is used to store msg in a given format
            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (msg.seq,msg.current,msg.frame,msg.command,msg.param1,msg.param2,msg.param3,msg.param4,msg.x,msg.y,msg.z,msg.autocontinue)
            output += commandline

            #store the waypoints in waypoint dictionary
            if (msg.command != 22):
                if(msg.seq != 0):   #i.e not home location
                    self._waypoints[mission_count] = {
                        'lat':msg.x,
                        'lng':msg.y,
                        'alt':msg.z,
                        'command':msg.command
                    }
                else:               #i.e home location
                    self._home.lat = msg.x
                    self._home.lon = msg.y
                    self._waypoints[mission_count] = {
                        'lat':self._home.lat,
                        'lng':self._home.lon
                    }
                mission_count += 1
            

        #write to file
        with open(file_name,'w') as file_:
            print("Write mission to file")
            file_.write(output)
        
        return self._waypoints
    

    def mission_upload(self, file_name = 'mission.txt'):
        wp = mavwp.MAVWPLoader()
        #clear waypoints before uploading, so that new waypoints can be added
        self._waypoints.clear()
        mission_count = 0
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

                    #store in waypoints
                    if(ln_command != 22):
                        if(ln_seq != 0):   #i.e not home location
                            self._waypoints[mission_count] = {
                                'lat':ln_x,
                                'lng':ln_y,
                                'alt':ln_z,
                                'command':ln_command
                            }
                        else:
                            self._waypoints[mission_count] = {
                                                    'lat':ln_x,
                                                    'lng':ln_y
                                                    }
                        mission_count += 1

                    p = mavutil.mavlink.MAVLink_mission_item_message(self.master.target_system, self.master.target_component, ln_seq, ln_frame,
                                                                    ln_command,
                                                                    ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                    wp.add(p)
        
        #while uploading mission, first home should be given
        self.set_home()
        #msg = self.master.recv_match(type = ['COMMAND_ACK'],blocking = True)
        #print(msg)
        print('Set home location: {0} {1}'.format(self._home.lat,self._home.lon))
        time.sleep(1)

        #send waypoint to airframe
        self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wp.count())

        for i in range(wp.count()):
            #msg = self.master.recv_match(type=['MISSION_REQUEST'],blocking=True)
            msg = None
            while msg == None:
                msg = self._msg_mission_request
            self._msg_mission_request = None #clear after read
            print(msg)
            self.master.mav.send(wp.wp(msg.seq))
            print('Sending waypoint {0}'.format(msg.seq))

    @property
    def flight_plan(self):
        return self._waypoints

    @property
    def is_armable(self):
        # check that we have a GPS fix
        # check that EKF pre-arm is complete
        return (self._fix_type > 1) and self._ekf_predposhorizabs

    @property
    def ekf_ok(self):
        # use same check that ArduCopter::system.pde::position_ok() is using
        if self._armed:
            return self._ekf_poshorizabs and not self._ekf_constposmode
        else:
            return self._ekf_poshorizabs or self._ekf_predposhorizabs

    @property
    def system_status(self):
        return {
            0: 'UNINIT',
            1: 'BOOT',
            2: 'CALIBRATING',
            3: 'STANDBY',
            4: 'ACTIVE',
            5: 'CRITICAL',
            6: 'EMERGENCY',
            7: 'POWEROFF',
            8 : 'FLIGHT_TERMINATION'
        }.get(self._system_status, None)

    @property
    def is_armed(self):
        return self._armed
    
    @property
    def flight_mode(self):
        return self._flightmode

    @property
    def heading(self):
        return self._heading
    
    @property
    def groundspeed(self):
        return self._groundspeed

    @property
    def airspeed(self):
        return self._airspeed

    @property
    def velocity(self):
        return [self._vx, self._vy, self._vz]
    
    @property
    def battery(self):
        return Battery(self._voltage,self._current,self._level)

    @property
    def attitude(self):
        return Attitude(self._roll,self._pitch,self._yaw,self._rollspeed,self._pitchspeed,self._yawspeed)

    @property
    def location(self):
        return Location(self._lat,self._lon,self._alt,self._relative_alt,self._north,self._east,self._down)

    @property
    def gps_0(self):
        return(GPSInfo(self._eph,self._epv,self._fix_type,self._satellites_visible))
   

class Battery():

    def __init__(self, voltage, current, level):
        self.voltage = voltage / 1000.0
        if current == -1:
            self.current = None
        else:
            self.current = current #/ 100.0
        if level == -1:
            self.level = None
        else:
            self.level = level
    def __str__(self):
        return "Battery:voltage={},current={},level={}".format(self.voltage, self.current,
                                                               self.level)


class Location():

    def __init__(self,lat=None,lon=None,alt=None,altR=None,north=None,east=None,down=None):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.altR = altR
        self.north = north
        self.east = east
        self.down = down

    def __str__(self):
        return "LocationGlobal:lat=%s,lon=%s,altR=%s,alt=%s  ||  LocationLocal:north=%s,east=%s,down=%s" % (self.lat, self.lon, self.altR,self.alt,self.north, self.east, self.down)


class GPSInfo():
    def __init__(self, eph, epv, fix_type, satellites_visible):
        self.eph = eph
        self.epv = epv
        self.fix_type = fix_type
        self.satellites_visible = satellites_visible

    def __str__(self):
        return "GPSInfo:fix=%s,num_sat=%s" % (self.fix_type, self.satellites_visible)

class Attitude():
    def __init__(self, roll, pitch, yaw,rollspeed,pitchspeed,yawspeed):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll
        self.rollspeed = rollspeed
        self.pitchspeed = pitchspeed
        self.yawspeed = yawspeed

    def __str__(self):
        return "Attitude:roll=%s,pitch=%s,yaw=%s" % (self.roll, self.pitch,self.yaw)

class Velocity():
    def __init__(self,vx,vy,vz):
        self.vx = vx
        self.vy = vy
        self.vz = vz

    def __str__(self):
        return "Velocity:vx=%s,vy=%s,vz=%s" % (self.vx, self.vy,self.vz)