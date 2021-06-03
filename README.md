# Home Service Robot

## References
This project utilizes the following ROS packages.

[gmapping](http://wiki.ros.org/gmapping): The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.

[amcl](http://wiki.ros.org/amcl): amcl is a probabilistic localization system for a robot moving in 2D. It implements the adaptive Monte Carlo localization approach, which uses a particle filter to track the pose of a robot against a known map.

[navigation stack](http://wiki.ros.org/navigation): A 2D navigation stack that takes in information from odometry, sensor streams, and a goal pose and outputs safe velocity commands that are sent to a mobile base. It is based on the Dijkstra's, a variant of the Uniform Cost Search algorithm, to plan a robot trajectory from start to goal position. The ROS navigation stack permits a robot to avoid any obstacle on its path by re-planning a new trajectory once the robot encounters them while localizing itself with AMCL.

[turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop): Provides teleoperation using joysticks or keyboard.

[turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers): Launchers for visualizing TurtleBot.

[turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo): Gazebo launchers and worlds for TurtleBot simulation.

In addition, the project includes the following packages:

[pick_objects](https://github.com/GinaSierra/HomeServiceRobot/tree/main/pick_objects): Includes a node that will communicate with the ROS navigation stack and autonomously send successive goals for a robot to reach.

[add_markers](https://github.com/GinaSierra/HomeServiceRobot/tree/main/add_markers): Includes a node to model a virtual object with markers in rviz. The virtual object is the one being picked and delivered by the robot, thus it first appears in its pickup zone, and then in its drop off zone once the robot reaches it.

## Package Tree
```
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├──                                # Your packages and direcotries
    |
    ├──pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├──add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```
## Install ROS Packages
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
$ sudo apt-get update
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-perception/slam_gmapping
$ git clone https://github.com/turtlebot/turtlebot
$ git clone https://github.com/turtlebot/turtlebot_interactions
$ git clone https://github.com/turtlebot/turtlebot_simulator
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosdep -i install gmapping
$ rosdep -i install turtlebot_teleop
$ rosdep -i install turtlebot_rviz_launchers
$ rosdep -i install turtlebot_gazebo
$ catkin_make
$ source devel/setup.bash
```
## Test SLAM
Needs to have run permissions.
```
$ chmod +x ~/catkin_ws/src/test_slam.sh
```
Run test_slam.sh to test slam is working, it requires manual movement of the robot.
```
$ ~/catkin_ws/src/test_slam.sh
```
## Test Navigation
Needs to have run permissions.
```
$ chmod +x ~/catkin_ws/src/test_navigation.sh
```

Run test_navigation.sh to test navigation is working.
```
$ ~/catkin_ws/src/test_navigation.sh
```

## Pick Objects
Needs to have run permissions.
```
$ chmod +x src/pick_objects.sh
```
Run pick_objects.sh to run two step navigation.

```
$ ~/catkin_ws/src/pick_objects.sh
```

## Add Markers
Needs to have run permissions.
```
$ chmod +x src/add_markers.sh
```
Run add_markers.sh to add and remove a marker on the pick up and drop off locations.

```
$ ~/catkin_ws/src/add_markers.sh
```

## Home Service
Needs to have run permissions.
```
$ chmod +x src/home_service.sh
```

Run home_service.sh to pick and drop marker to specified locations.
```
$ ~/catkin_ws/src/home_service.sh
```

