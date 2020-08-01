[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# RoboND_P5_Home-Service-Robot
Final Robotic Software Engineer Nanodegree Project "Home Service Robot".  
This project simulates a robot that picks up a virtual object at a certain position, transports it to the target position and places the object there again.

### Output Video
[![Home Service Robot](http://img.youtube.com/vi/TAsDPuI8lic/0.jpg)](http://www.youtube.com/watch?v=TAsDPuI8lic "Home Service Robot")

### Packages

##### pick_objects
Moves robot from to pickup zone and publish the `DO_PICKUP` action on `/my_robot/action_publisher` topic.  
Moves robot to dropoff zone and public the `DO_DROPOFF` action on `/my_robot/action_publisher` topic.

##### add_markers
Subscribes the `/my_robot/action_publisher` topic.
Views a marker on the pickup zone, hides the marker on `DO_PICKUP` and view the marker in the dropoff zone after receiving `DO_DROPOFF`.

#### Third Party 
[gmapping](http://wiki.ros.org/gmapping)  
With the gmapping_demo.launch file, you can easily perform SLAM and build a map of the environment with a robot equipped with laser range finder sensors or RGB-D cameras.

[turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)  
With the keyboard_teleop.launch file, you can manually control a robot using keyboard commands.

[turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)  
With the view_navigation.launch file, you can load a preconfigured rviz workspace. You’ll save a lot of time by launching this file, because it will automatically load the robot model, trajectories, and map for you.

[turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)  
With the turtlebot_world.launch you can deploy a turtlebot in a gazebo environment by linking the world file to it.

### Steps to launch the simulation

#### Step 1 Update and upgrade the Workspace image
```sh
$ sudo apt-get update
$ sudo apt-get upgrade -y
```

#### Step 2 Clone the lab folder in /home/workspace/
```sh
$ cd /home/workspace/
$ git clone https://github.com/tobiassteidle/RoboND_P5_Home-Service-Robot

```

#### Step 3 Compile the code
```sh
$ cd /home/workspace/RoboND_P5_Home-Service-Robot/catkin_ws
$ catkin_make
```

#### Step 4 Source ROS in this workspace
```sh
$ source devel/setup.bash
```

#### Step 5 Run the Simulation
There are several ways to start the simulation.

##### Testing SLAM
```sh
$ cd /home/workspace/RoboND_P5_Home-Service-Robot/catkin_ws
$ ./src/scripts/test_slam.sh
```

##### Testing Navigation and Localisation
```sh
$ cd /home/workspace/RoboND_P5_Home-Service-Robot/catkin_ws
$ ./src/scripts/test_navigation.sh
```

#### Testing Service Robot
Pickup and Dropoff Zone location is configured in `src/params/parameters.yaml`.  
The Map is in the `src/map` directory.  
RVIZ configuration is in `src/rvizConfig/rvizConfig.rviz`.  

```sh
$ cd /home/workspace/RoboND_P5_Home-Service-Robot/catkin_ws
$ ./src/scripts/home_service.sh
```
