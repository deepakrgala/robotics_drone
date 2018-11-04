This project will focus on the component of implementation on physical robots. 
You will fly an actual quadcopter in the real world to a subset of checkpoints of varying importance. 
The goal is to visit as many important checkpoints as possible and return to the starting location within a set amount of time.

This program proj3flying.py prints the latitude and longitude values of the AR Drone. 
It subscribe to the navdata_gps topic in ardrone_autonomy and uses a function gps_data() to print the values on screen.
It also simply launch (takes off) and lands the drone.
The launch time can be decided by adjusting the time value in time.sleep() between launch and land functions in main. 
