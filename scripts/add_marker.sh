#!/bin/sh

export TURTLEBOT_GAZEBO_WORLD_FILE=$(pwd)/../map/project.world
export TURTLEBOT_GAZEBO_MAP_FILE=$(pwd)/../map/map.yaml

xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch"  &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch"  &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch"  &
sleep 5
xterm  -e  " rosrun add_markers add_markers_virtual_objects"