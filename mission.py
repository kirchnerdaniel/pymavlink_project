from drone import drone
from pymavlink import mavutil
import time
import math

#Create the Mission Item.
#The 'i' parameter is the sequence number for item within mission.
#'Current' is the current mission item (False:0, True:1).
# 'X' - latitude, 'Y' - longitude, 'Z' - altitude.
class mission_item:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.autocontinue = 1
        self.param1 = 3.0
        self.param2 = 2.00
        self.param3 = 20.00
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

#Set the flight time. Return the elapsed time if the drone is armed. Otherwise return the total flight time.
def get_flight_time(drone):
    heartbeat = drone.vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        return time.time() - drone.armed_time
    else:
        return flight_time

#Upload the mission to the drone
def upload_mission(drone, mission_items):
    n = len(mission_items)
    print("Send message out")

    drone.vehicle.mav.mission_count_send(
        drone.vehicle.target_system,
        drone.vehicle.target_component, 
        n, 0)

    for waypoint in mission_items:
        print("Creating a waypoint")

        drone.vehicle.mav.mission_item_send(
            drone.vehicle.target_system,
            drone.vehicle.target_component,
            waypoint.seq,
            waypoint.frame, 
            waypoint.command,
            waypoint.current,
            waypoint.autocontinue,
            waypoint.param1,
            waypoint.param2,
            waypoint.param3,
            waypoint.param4,
            waypoint.param5,
            waypoint.param6,
            waypoint.param7,
            waypoint.mission_type
        )

        if waypoint == mission_items[n-1]:
            mission = drone.vehicle.recv_match(type='MISSION_ACK', blocking=True)
            if mission.type == 0:
                print("Mission accepted") 
            else:
                print('Mission error')

#Start the mission. Arm the drone and take off.
def start_mission(drone):
    print("Mission start")
    drone.arm_disarm(1)
    drone.takeoff(1)
    drone.vehicle.mav.command_long_send(
        drone.vehicle.target_system,
        drone.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )

mission_waypoints = []
flight_time = 0

new_drone = drone('14551', 'GUIDED')

new_drone.set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000)

#Set the mission waypoints
mission_waypoints.append(mission_item(0, 0, -35.3632620, 149.1652347, 10))
mission_waypoints.append(mission_item(1, 0, -35.3633620, 149.1652373, 10))
mission_waypoints.append(mission_item(2, 0, -35.3633620, 149.1653373, 10))
mission_waypoints.append(mission_item(3, 0, -35.3632620, 149.1653373, 10))
mission_waypoints.append(mission_item(4, 0, -35.3632620, 149.1652347, 10))

upload_mission(new_drone, mission_waypoints)

start_mission(new_drone)

#Print the current lat-lon coordinates, flight time and current mission item.
while 1:
    gps = new_drone.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True) 
    print(f"Latitude: {gps.lat / 10 ** 7}, Longitude: {gps.lon / 10 ** 7}")

    flight_time = get_flight_time(new_drone)
    print(f"Flight time: {int(flight_time)}")

    current_waypoint = new_drone.vehicle.recv_match(type='MISSION_CURRENT', blocking=True) 
    print(f"Current mission item: {current_waypoint.seq}")
    
    

    