# ROS_MODULE

This is a ros module for sgr a

#Prerequisites
To detect and track peoples this module use [OpenPtrack](http://openptrack.org/) ros module look at [github repo](https://github.com/OpenPTrack/open_ptrack)
for further information.
<br />
To run stop_distance.py need matlab's python wrapper (see matlab website for further information).


# Step 1: Start Tracking
To start human detection and tracking you need to run:
~~~
roslaunch tracking detection_and_tracking.launch
~~~

# Step 2: Start Smartband
To start the smartband listening module you need to run:
~~~
roslaunch sgr_project smartband.launch
~~~

# Step 3: Start Aproaching
To allow the robot approach a human you need to run:
~~~
roslaunch sgr_project approach.launch
~~~