Mobile Robot - Ros
================================

Research track 1 - Assignment 2 
-------------------------------

In this repo is implemented the bug0 algorithm for a mobile robot in Rviz, a tool for ROS Visualization, and Gazebo, the 3D simulator for ROS.
There are also other nodes to retrieve some important information from the simulation.

Installing and running
----------------------

Once the environment is installed and the repository has been cloned, run:
```bash
$ catkin_make
```
in your root folder then:
```bash
$ roslaunch assignment_2_2023 assignment1.launch
```
If you want to call the implemented service just run:
```bash
$ rosservice call /dist_speed
```
and
```bash
$ rosservice call /last_target
```
While the data published on the topic are visible running:
```bash
$ rostopic echo /Pos_vel
```

###Node A


The node `aclient.cpp` implements an action client that:
*allows the user to set a target:
```plaintext
-request x coordinate
-request y coordinate
-fill the fields of the goal section of the Plannin.action file  with x and y
-send the goal to the action server
```
*allows the user to cancel a target:
```plaintext
-ask whether to cancel the target
-if the input is different from the expected re-ask the input
-if the user want to cancel the target and it is not yet been reached, cancel the target    
```
*Use the status of the action server to know when the target has been reached:
```plaintext
-wait for the result
-if status == "SUCCEEDED", then the target has been reached  
```
The above sections of code are inside an infinite while loop, so that when a target is reached then you may be asked to enter a new one.
Furthermore, this functions are implemented with threads in a such way that, the publishers and the subsribers, that are in the main function, cannot be bothered by waiting for the task to complete. Also the request to cancel the target is in a separate thread to avoid blocking the program waiting for the input, infact in this case if the robot reached the target, the program would not be aware of it and the user could delete a target already reached.
*The node also publishes the robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic /odom:
```plaintext
 -initialization of the subscriber for the topic odom
 -initialization of the publisher 

 callback of the subscriber:
 -fill the custom message fields with the of the message of type Odometry
```
The custom message Pos_vel.msg has been constructed to accept four float values corresponding to (x,y, vel_x, vel_z)

###Node B

The node `last_target.cpp` when called, returns the coordinates of the last target sent by the user, subscribing the goal topic of the actionLib and pushing its fields in the service /last_target.

###Node C

The node dist_speed.cpp subscribes the robot's position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot's average speed.

###RQT_Graph

The ros graph of the application showing nodes and topics:

![Testo alternativo](/src/rosgraph.png)

