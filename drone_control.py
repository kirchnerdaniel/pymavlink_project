from drone import drone
from pymavlink import mavutil
import time

#Navigate to the first checkpoint
def navigate(drone, target):
        drone.vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10, 
            drone.vehicle.target_system,
            drone.vehicle.target_component, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            int(0b110111111000), 
            int(target["lat"] * 10 ** 7), 
            int(target["lon"] * 10 ** 7), 
            int(target["alt"]), 
            0, 0, 0, 0, 0, 0, 0, 0)
        )

#Navigate to the next checkpoint and land after all checkpoints are reached.      
def navigate_next(drone, gps, targets):
    global checkpoint

    if checkpoint >= len(targets):
        print("All checkpoints reached.")
        if drone.mode == 'LAND':
            return
        else: 
            drone.set_mode('LAND')
        return
    
    #Set the tolerance for the checkpoint in latitude, longitude. 
    lat_tolerance = 10  
    lon_tolerance = 10  
    
    if (int(targets[checkpoint]["lat"] * 10 ** 7) - lat_tolerance < gps.lat < int(targets[checkpoint]["lat"] * 10 ** 7) + lat_tolerance) and (int(targets[checkpoint]["lon"] * 10 ** 7) - lon_tolerance < gps.lon < int(targets[checkpoint]["lon"] * 10 ** 7) + lon_tolerance):

        print(f"Checkpoint {checkpoint+1} reached!")
        checkpoint += 1
        
        if checkpoint < len(targets):
            time.sleep(3)
            navigate(drone, targets[checkpoint])
    else:
        return


checkpoint = 0
#Set the targets waypoint.
targets = [
    {
        "lat" : -35.3633620,
        "lon" : 149.1652373,
        "alt" : 5
    },
    {
        "lat" : -35.3633620,
        "lon" : 149.1653373,
        "alt" : 10
    },
    {
        "lat" : -35.3632620,
        "lon" : 149.1653373,
        "alt" : 10
    },
    {
        "lat" : -35.3632620,
        "lon" : 149.1652373,
        "alt" : 5
    },
]

new_drone = drone('14551', 'GUIDED')

new_drone.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000)
new_drone.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000)
new_drone.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 1000000)

new_drone.arm_disarm(1)

new_drone.takeoff(1)

navigate(new_drone, targets[0])

#Print the battery voltage, speed (m/s), drone direction and current position (lat-lon).
while 1:
    battery = new_drone.vehicle.recv_match(type='SYS_STATUS', blocking=True).voltage_battery
    print(f"Battery Voltage: {battery}")
    
    speed_direction = new_drone.vehicle.recv_match(type='VFR_HUD', blocking=True)
    print(f"Ground Speed: {speed_direction.groundspeed:.2f} m/s, Heading: {speed_direction.heading}")

    gps = new_drone.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    print(f"Latitude: {gps.lat / 10 ** 7}, Longitude: {gps.lon / 10 ** 7}")
    navigate_next(new_drone, gps, targets)
