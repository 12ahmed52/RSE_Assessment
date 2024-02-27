# Micropolis Robotics - Robotics Software Engineer Technical Assesment

## Milestones
To achieve the desired goal, we break it down to smaller     to be achieved in order based on its debendency for the the next step.


### 1 - Preform a SLAM on the provided world
First, you need to map the robot world so it can understand it for later operations. </br>
Utilize your knowledge of SLAM algorithms to produce a digital map of the world.

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
