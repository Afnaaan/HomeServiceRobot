# HomeServiceRobot

### Overview
This is a Home Service Robot simulation project for path planning and navigation in ROS. Using Turtlebot robot with coustom gazebo world building a map of the world with the robot 
using The gmapping package for OpenSlam's Gmapping which provides laser-based SLAM (Simultaneous Localization and Mapping), localizing with AMCL(Adaptive Monte Carlo 
Localization) from turtlebot_gazebo package, using the ROS Navigation stack to plan our robot trajectory from start to goal position while avoiding obstacles on its path,
contoling the robot with turtlebot_teleop package, and viewing navigation with Rviz from turtlebot_rviz_launchers in turtlebot_interactions package.
**Keywords:** Udacity, ROS, Robot, Turtlebot, Gazebo, Rviz, Mapping, Gmapping, SLAM, Localization, Monte Carlo, Navigation stack, Path planning.

### Packages Used:
#### Ros Packages
* [slam_gmapping](https://github.com/ros-perception/slam_gmapping)
  * This package creates a map of the environment using laser-based SLAM.
* [turtlebot](https://github.com/turtlebot/turtlebot)
  * Used turtlebot_teleop  to manually drive the robot from the keyboard in some the scripts file.
* [turtlebot_interactions](https://github.com/turtlebot/turtlebot_interactions)
   * This package is used to launch RViz which allows the user to visualize SLAM, particle filters and send nav goals. Here RViz config files can be saved which will launch the map, robot model, markers etc.
* [turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator)
  * This package contains the turtlebot_gazebo file which allows you to launch the turtlebot world  with the turtlebot(turtlebot_world.launch) and launches the amcl algorithm (amcl_demo.launch). The robot is able to localize itself using the amcl package. 

#### Packages created by me
* [pick_objects](https://github.com/Afnaaan/HomeServiceRobot/tree/main/pick_objects)
  * This package sends pick up and drop off goals to the robot which then uses the ROS navigation stack to reach the goal. Pick_objects package drives the navigation of the robot.
* [add_markers](https://github.com/Afnaaan/HomeServiceRobot/tree/main/add_markers)
  * This package publishes a virtual object in Rviz that appears in the pick up goal location. Once the robot reaches the pick up location the object disappears and the robot is then directed to the drop off goal location by the pick_objects package. Once the robot reaches the drop off the goal location then the object appears. 
  
### Scripts
* ./test_slam.sh
   * allows user to teleop the robot and have it interface with SLAM to visualize the map in RViz
* ./test_navigation.sh
  * users can give a direction to the robot with 2D nav goal in rviz and using the ROS navigation stack it will localize itself and reach the goal.
* ./pick_objects.sh
  * robot drives to pick up location and drop off location.
* ./add_markers/sh
  * marker appears at pick up location waits 5 secs then appears at drop off location
* ./home_service/sh
  * robot autonomously drivers to pick up location picks up object and the drives to drop off location and drops off object. 

### Usage:

First create a catkin workspace
[To know how see catkin tutorial](http://wiki.ros.org/ROS/Tutorials/catkin/CreateWorkspace)

#### To map your environment

1. add your world file in 'map' folder
2. edit value of 'TURTLEBOT_GAZEBO_WORLD_FILE' in test_slam.h script
3. type in these commands in the catkin workspace
```
> catkin_make
> source devel/setup.bash
> cd src/scripts
> ./home_service.sh
```
4. move your robot with keyboard_teleop termenal in your world to explor and map the area you want to map
5. in the path you want to save your map in run this cmd "rosrun map_server map_saver"

*to use this map in my services edit value of 'TURTLEBOT_GAZEBO_MAP_FILE' in its script*

#### To launch the home service 
type in these commands in the catkin workspace. 
```
> catkin_make
> source devel/setup.bash
> cd src/scripts
> ./home_service.sh
```
