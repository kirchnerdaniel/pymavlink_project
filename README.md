This project uses Arducopter version 4.3.2.


**drone.py**

A class to represent a drone. This class provides methods to connect to the drone, adjust the flight mode, set the message send interval, arm the drone, and take off.


**takeoff.py**

The drone selects 'GUIDED' mode and takes off to an altitude of 10 meters. When it reaches this altitude, it switches to 'LAND' mode.


**prearmcheck.py**

This script runs pre-arm check command. After that, it logs the system state and GPS data.


**dronecontrol.py**

This script changes the send message interval to 1Hz. After that, the drone navigates to the given waypoints. (In targets array are the waypoint coordinates and altitudes.) When the drone reaches a waypoint, it moves on to the next one. After completing all waypoints, the drone lands. The script logs the battery voltage, speed, drone direction and current position.


**mission.py**

This script creates mission items, uploads them to the drone, and starts the mission.


**keyboard_control.py**

This script allows you to control the drone using the keyboard. The 'getch' library is required!

W - forward
S - backward
A - left
D - right
Q - up
E - down
T - takeoff
Z - land

After the script starts, it sets 'GUIDED' flight mode, arms the drone and waits for takeoff. When it displays "Ready to go!" you can navigate using the keys.
