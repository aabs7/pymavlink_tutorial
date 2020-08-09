# Import mavutil
from pymavlink import mavutil

# Create the connection
# From topside computer
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')

MAV_STATE = {
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

while True:
    msg = master.recv_match()
    
    if not msg:
        continue
    else:
        dict_msg = msg.to_dict()
        if msg.get_type() == 'HEARTBEAT':
            if (dict_msg['type'] != 6):
                print("Flt Mode:",mavutil.mode_mapping_acm[dict_msg['custom_mode']])
                print("Status:", MAV_STATE[dict_msg['system_status']])
        
        if msg.get_type() == 'GLOBAL_POSITION_INT':
            print("Lat:",dict_msg['lat']," Lon:",dict_msg['lon']," Alt:",dict_msg['alt']," Alt R:",dict_msg['relative_alt']," Heading:",dict_msg['hdg'])
        
        if msg.get_type() == 'GPS_RAW_INT':
            print("Sat:", dict_msg['satellites_visible']," Hdop:",dict_msg['eph']," Fix:",dict_msg['fix_type'])

        if msg.get_type() == 'VFR_HUD':
            print("GS:",dict_msg['groundspeed']," AS:",dict_msg['airspeed']," Heading:",dict_msg['heading'])
            print("Armed:",master.motors_armed())
        
        if msg.get_type() == 'BATTERY_STATUS':
            print("volt:",dict_msg['voltages'][0] * 0.001)
            
        
        
        
        
        