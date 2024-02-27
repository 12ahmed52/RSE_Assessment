# Micropolis Robotics - Robotics Software Engineer Technical Assesment

## Milestones

### 1 - Preform a SLAM on the provided world
The used Algorithm for the 3D Mapping is hdl_graph_slam: </br>
An Open Source ROS package for real-time 6DOF SLAM using a 3D LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection.</br>
I have added some buildings and raised the lidar to use the 360 points for more accurate SLAM.</br>


### 2 - Offline Localization
Next, to move the robot autonomously around the map you need to localize the robot in real-time without using SLAM (offline localization).</br>
Implement/Use a localization algorithm to localize the robot in the map, and test that your localization is working by movibg the robot manyually arround the map and validate the localization output.

### 3 - Autonomous Navigation with Obstacle avoidance
Once you have a represntation of the environment and you can localize your robot within it, you can then start the autonomous navigation of the robot.</br>
Implement/Use an autonomous navigation algorithm to drive the robot by itself to a defined goal that can be set in the RViz GUI, while avoiding any obstacle.

### 4 - External GUI Teleoperation
To make sure a smother operation of the robot when deploying it in the field, it's better to have a user friendly remote operation capabilties.</br>
Develop a GUI that allows use to remotly control the robot from outside a ROS environment.
Feel free to pick whatever framework or stack you like (C++ Qt, Python, Web-based GUI, etc...).
**NOTE:** Implement only the basic functionality (Drive, Steer).

### 5 - User Defined Navigation (Open)
Develop the previous milestone and adopt it so the user of your GUI can also perform the Navigation functionality (Sendg Waypoints/Goal, Mapping, etc...).

### (Optional) - Develop an Odometry Source for the robot
The very first required components to start working on any autonomous functionality are the position, orientation, velocity feedback of the robot.</br>
If we ignore the Odometry feedback provided by Gazebo, based on the robot description provided and the sensor set, develop a node that produce an Odometry feedback as accurate as possible.



```bash
GOOD LUCK!
```
￼
Collapse

has context menu
