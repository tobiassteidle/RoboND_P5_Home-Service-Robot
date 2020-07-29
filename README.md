# ACHTUNG PROJEKT NOCH PRIVATE

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

# RoboND_P5_Home-Service-Robot
Final Robotic Software Engineer Nanodegree Project "Home Service Robot"

### Used Packages

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

#### Step 2 Install dependencies
```sh
$ TODO

```

#### Step 3 Clone the lab folder in /home/workspace/
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
```sh
$ TODO
```




#### ToDos
- [x] Simulation Setup  
*Evtl müssen "pick_objects" und "add_markers" als Package angelegt werden.*

- [x] SLAM Testing
- [ ] Wall Follower Node
- [ ] Navigation Testing
- [ ] Waypoint Node
- [ ] Virtual Objects
- [ ] Put it all Together
