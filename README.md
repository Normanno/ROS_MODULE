# ROS_MODULE

This is a ros module for sgr a

#Prerequisites
To detect and track peoples this module use [OpenPtrack](http://openptrack.org/) ros module look at [github repo](https://github.com/OpenPTrack/open_ptrack)
for further information.
<br />
To compute stop distance need matlab's python wrapper (see matlab website for further information).
<br />
To move toward human need RosAria


# Step 1: Start Tracking
To start human detection and tracking you need to run:
~~~
roslaunch tracking detection_and_tracking.launch
~~~

# Step 2: Start RosAria and Smartband receiver
To start RosAria, the smartband receiver and the main control node.
~~~
roslaunch sgr_project init.launch
~~~

# Step 3: Start Aproaching
To allow the robot approach a human you need to run:
~~~
roslaunch sgr_project approach.launch
~~~


#Step 4: Send data from the smartband
Download and in install the application at the [git repo](https://github.com/Normanno/WearSensorsApp.git) branch "ROS"
<br /> 
Then: <br />
- (1) Install the application on both smartphone and smartwatch
- (2) Insert host, port, and path of the smartband_receiver node.
- (3) Toggle "Start ROS" (this will run a check on the paramaters) 
- (4) Toggle "Activate" to start sending data to ROS


#Step 5 (Optional): Control client
If you want to change parameters like linear/angular velocity or personality parameters just run the client with:
~~~
roslaunch sgr_project client.launch
~~~