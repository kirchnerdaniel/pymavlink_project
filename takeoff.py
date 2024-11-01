from drone import drone

#Create a new drone with the port and flight mode
new_drone = drone('14551', 'GUIDED')

new_drone.arm_disarm(1)

new_drone.takeoff(10)

new_drone.set_mode('LAND')

new_drone.vehicle.motors_disarmed_wait()
print('Disarmed!')

