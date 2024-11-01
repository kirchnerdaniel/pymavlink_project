from pymavlink import mavutil
import time

class drone:

    #Set the drone parameters: port, flight mode. 
    #The 'mode' parameter can be empty (''), then the flight mode will not change.
    def __init__(self, port, mode):
        self.port = port
        self.vehicle = None
        self.armed_time = 0
        self.mode = mode

        self.connect()

        if mode != '':
            self.set_mode(mode)
        else:
            mode_id = self.vehicle.recv_match(type='HEARTBEAT', blocking=True).custom_mode
            mode_mapping = self.vehicle.mode_mapping()

            for name, id in mode_mapping.items():
                if id == mode_id:
                    mode_name = name
            
            self.mode = mode_name

    #Start a connection listening on a UDP port (14551)
    def connect(self):
        self.vehicle = mavutil.mavlink_connection(f'udpin:localhost:{self.port}')
        # This sets the system and component ID of remote system for the link
        self.vehicle.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.vehicle.target_system, self.vehicle.target_component))


    #Set the drone flight mode. The 'mode' parameter is the flight mode name as a string. 
    def set_mode(self, mode):
        self.mode = mode
        mode_id = self.vehicle.mode_mapping()[mode]
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
            0,
            1, mode_id, 0, 0, 0, 0, 0
        )


    #Arms or disarms the drone. The 'arm_parameter' is 0 to disarm and 1 to arm.
    def arm_disarm(self, arm_parameter):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm_parameter, 0, 0, 0, 0, 0, 0
        )
        if arm_parameter == 1:
            self.vehicle.motors_armed_wait()
            print('Armed!')
            self.armed_time = time.time()
        elif arm_parameter == 0:
            self.vehicle.motors_disarmed_wait()
            print('Disarmed!')
        else:
            print('Arm parameter incorrect!')


    #Take off the drone in local position. The 'altitude' parameter is the desired altitude as a number. 
    #The function waits in a loop until the drone takes off.
    def takeoff(self, altitude):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0, altitude
        )
        current_altitude = 0
        print('Wait to taking off!')
        while current_altitude != altitude:
            current_altitude = round(self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True).z * -1)


    #Set the message interval. It works only when 'streamrate' is -1. 
    #The 'message_id' parameter is the message ID number, or use mavutil.mavlink.MAVLINK_MSG_ID_... The 'interval' parameter set the interval in miscroseconds.
    def set_message_interval(self, message_id, interval):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval,
            0,
            0,
            0,
            0,
            0
        )
  



        
    

