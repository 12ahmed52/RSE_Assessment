# Micropolis Robotics - Robotics Software Engineer Technical Assesment

## Milestones

### 1 - Preform a SLAM on the provided world
The used Algorithm for the 3D Mapping is hdl_graph_slam: </br>
An Open Source ROS package for real-time 6DOF SLAM using a 3D LIDAR. It is based on 3D Graph SLAM with NDT scan matching-based odometry estimation and loop detection.</br>
I have added some buildings and raised the lidar to use the 360 points for more accurate SLAM.</br>
### The Output 3D Map:
![Screenshot from 2024-02-27 12-56-58](https://github.com/12ahmed52/RSE_Assessment/assets/52854480/5534e068-ef6b-4754-ae3e-743361833c49)



### 2 - Offline Localization
To Localize the Robot in the Map the used algorithm was hdl_localization: </br>
A ROS package for real-time 3D localization using a 3D LIDAR. This package performs Unscented Kalman Filter-based pose estimation. It first estimates the sensor pose from IMU data implemented on the LIDAR, and then performs multi-threaded NDT scan matching between a globalmap point cloud and input point clouds to correct the estimated pose but I have disabled the IMU. </br>
### The Localization output in the 3D Map:

![Screenshot from 2024-02-27 13-08-45](https://github.com/12ahmed52/RSE_Assessment/assets/52854480/8dffd831-54ef-406f-af08-5420694e7a99)

### 3 - Autonomous Navigation with Obstacle avoidance
To Navigate Safetly from one Point to another I have used the ros nav_stack. </br>
### Planners Used:
### teb_local_planner:
The underlying method called Timed Elastic Band locally optimizes the robot's trajectory with respect to trajectory execution time, separation from obstacles and compliance with kinodynamic constraints at runtime.</br>
### global_planner:
A fast, interpolated global planner for navigation it uses A* or dijkstra based on the configuration.</br>
### Navigation Output:
![Screenshot from 2024-02-25 15-29-02](https://github.com/12ahmed52/RSE_Assessment/assets/52854480/aa8a9d67-bd36-4897-bf3a-c731af51b0d7)


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
