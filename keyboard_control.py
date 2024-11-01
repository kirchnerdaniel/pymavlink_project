from drone import drone
from pymavlink import mavutil
import getch

#Send the new positions to the drone.
def control_drone(x,y,z):
   global current_position
   current_position.x = current_position.x + x
   current_position.y = current_position.y + y
   current_position.z = current_position.z + z
   new_drone.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, 
        new_drone.vehicle.target_system,
        new_drone.vehicle.target_component, 
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        int(0b010111111000), 
        current_position.x, 
        current_position.y, 
        current_position.z, 
        0, 0, 0, 0, 0, 0, 0, 0))

#Handle the keyboard inputs.
#[w - forward, s - backward, a - left, d - right, q - up, e - down, t - take off, z - land] 
def handle_input(keyboard_input):
    if keyboard_input == ('w' or 'W'):
        control_drone(1,0,0)
    elif keyboard_input == ('s' or 'S'):
        control_drone(-1,0,0)
    elif keyboard_input == ('a' or 'A'):
        control_drone(0,-1,0)
    elif keyboard_input == ('d' or 'D'):
        control_drone(0,1,0)
    elif keyboard_input == ('q' or 'Q'):
        control_drone(0,0,-1)
    elif keyboard_input == ('e' or 'E'):
        control_drone(0,0,1)
    elif keyboard_input == ('t' or 'T'):
        start()
    elif keyboard_input == ('z' or 'z'):
        land()
    else:
        print('Invalid key!')

#Set guided mode, arm the drone and takeoff. Then set the current position.
def start():
    global current_position
    new_drone.set_mode('GUIDED')
    new_drone.arm_disarm(1)
    new_drone.takeoff(5)
    current_position = new_drone.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print('Ready to go!')

#Set land mode, disarm the drone and update the current position.
def land():
    global current_position
    new_drone.set_mode('LAND')
    print('Land')
    new_drone.vehicle.motors_disarmed_wait()
    print('Disarmed!')
    current_position = new_drone.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)


current_position = None
new_drone = drone('14551', '')
start()

while 1:
    handle_input(getch.getch())
    #Press Ctrl + C to exit the program.
