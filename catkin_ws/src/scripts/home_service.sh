#!/bin/sh
export ROBOT_INITIAL_POSE='-x -0.01 -y 0.63 -z 0 -R 0 -P 0 -Y 0';
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/src/map/Flat.world " &
sleep 10
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/map/Flat.yaml" &
sleep 5
xterm  -e  " roslaunch add_markers rviz.launch rviz_config_file:=$(pwd)/src/rvizConfig/rvizConfig.rviz" &
sleep 5
xterm  -e  " rosparam load src/params/parameters.yaml" &
xterm  -e  " rosrun add_markers add_markers" &
sleep 3
xterm  -e  " rosrun pick_objects pick_objects" &
