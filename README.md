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


### 4 - External GUI Teleoperation and Navigation
The Implemented GUI is based on QT it has two parts:</br>
Front end: Part that is responsible for the user interface.</br>
back end: A ROS Node that recieves data from the Front end using TCP/IP communication and JSON msgs. I have used TCP/IP communication so that we can connect to the robot from other devices.</br>
The GUI is responsible for the Teleoperation and sending goals to the nav stack.</br>
![Screenshot from 2024-02-27 14-51-55](https://github.com/12ahmed52/RSE_Assessment/assets/52854480/6cef3a99-ebed-4010-a261-2e35215cc9c5)

### For more advanced and user friendly GUI:
we can use VIZANTI a web based gui that have advanced functionalities as viewing the point cloud and camera data, Sending nav goals and a list of goals, view TF and Robot location etc.</br>
![Screenshot from 2024-02-22 15-54-26](https://github.com/12ahmed52/RSE_Assessment/assets/52854480/8044c9a6-64c3-47ee-9552-d5a758925b14)


### Finally how to run the whole system
The following command will launch the navigation stack, hdl_localization, Twis_Mux, Pointcloud_to_LaserScan and the ros_gui
```bash
roslaunch system_launch system_launch.launch
```
