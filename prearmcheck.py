from drone import drone
from pymavlink import mavutil

#Print response messages separately.
def print_data(data_name, data_type):
    print(f'\n{data_name}')
    for key in data_type._fieldnames:
        value = getattr(data_type, key)
        print(f"{key}: {value}")


new_drone = drone('14551', '')

#Run pre arm check on the drone. 
new_drone.vehicle.mav.command_long_send(
    new_drone.vehicle.target_system,
    new_drone.vehicle.target_component,
    mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
    0,
    0, 0, 0, 0, 0, 0, 0
    )

#Print the response message ID and result
prearm_check = new_drone.vehicle.recv_match(type='COMMAND_ACK', blocking=True)
print(f'\nCommand: {prearm_check.command} \nResult: {prearm_check.result}')

#Get the general system state, and GPS data
sys_status = new_drone.vehicle.recv_match(type='SYS_STATUS', blocking=True)
gps = new_drone.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

print_data('GPS DATA:', gps)
print_data('SYS_STATUS:', sys_status)


