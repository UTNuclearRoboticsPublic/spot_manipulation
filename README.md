# NRG Spot Manipulation                                                                                       
This repository contains all of the packages required to operate the Spot Arm through ROS, including jogging and MoveIt congfiguration. The `spot_manipulation_driver` package provides a python wrapper and ROS node to send commands to the Spot arm and finger, as well as twist command subscribers for the end-effector.

The `spot_moveit_config` package provides the configuration settings for using the Spot Arm with MoveIt. This requires that the manipulation driver also be running.

## Spot Manipulation Driver

To run the robot arm without command of the robot body, execute the command
```bash
ros2 run spot_manipulation_driver manipulation_driver_node --ros-args -p hostname:=192.168.50.3 -p publish_joint_states:=True 
```

Running both the body driver and the manipulation driver together can be achieved by running the `combined_driver_node` executable, however the easier way would be to follow the bringup instructions in the `spot_ros` package [here](https://github.com/UTNuclearRobotics/spot_ros).

## Spot MoveIt Config
The `spot_moveit_config` package can be used to bring up MoveIt servers with the correct settings to operate with the Spot Robot. The main launch files of interest are:

```bash
ros2 launch spot_moveit_config move_group.launch.py     # bring up move group server
ros2 launch spot_moveit_config spot_execution.launch.py # same as above, but launches RViz as well
```

This package is configured to consider environmental obstacles localized to the map frame, so the `spot_navigation` localization capabilities need to be running for this to work. If you wish to operate without obstacle avoidance (which you shouldn't), you can comment out the `spot_mobility_joint` entry in the robot SRDF and comment out all of the 3D sensors in the `sensors_3d.yaml` file.
