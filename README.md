# NRG Spot Manipulation
This repository contains the `spot_manipulation_driver` package required to operate the Spot Arm through ROS with the notable features listed below. Additionally, the `spot_moveit_config` package provides the configuration settings for using the Spot Arm with MoveIt. This requires that the manipulation driver also be running.

## Notable Features
- **ROS Services:**
  - Take arm to predefined named poses
  - Open and close gripper

- **ROS Action Servers:**
  - Execute arm joint trajectories
  - Execute gripper trajectories
  - Execute arm and gripper trajectories as a single synced trajectory
  - Execute body, arm and gripper trajectories as a single synced trajectory
  - Execute cartesian position/force trajectory for the EE
  - Given pixel coordinates, camera info, and a transform snapshot have the robot go and grasp the pixel-specified object in an image.

- **ROS Subscriptions:**
  - Jog arm with twist messages

## Spot Manipulation Driver
The recommended way to run this driver is to run it combined with the body driver. For that, detailed information is available [here](https://github.com/UTNuclearRoboticsPublic/spot_ros.git)

If there is a need to run just the manipulation driver, execute the following command from the command line:
```bash
ros2 run spot_manipulation_driver manipulation_driver_node --ros-args -p hostname:=192.168.50.3 -p publish_joint_states:=True
```

## Spot MoveIt Config
The `spot_moveit_config` package can be used to bring up MoveIt servers with the correct settings to operate with the Spot Robot. The main launch files of interest are:

```bash
ros2 launch spot_moveit_config move_group.launch.py     # bring up move group server
ros2 launch spot_moveit_config spot_execution.launch.py # same as above, but launches RViz as well
```

This package is configured to consider environmental obstacles localized to the map frame, so the `spot_navigation` localization capabilities need to be running for this to work. If you wish to operate without obstacle avoidance (which you shouldn't), you can comment out the `spot_mobility_joint` entry in the robot SRDF and comment out all of the 3D sensors in the `sensors_3d.yaml` file.

## Spot_bringup Launch Args
- `manipulation_action_namespace:="/spot_moveit"` for moveit-related planning
- `kinematic_model:="mobile_manipulation"` to publish mobile-manipulation-related joint states

## Authors
Janak Panthi (aka Crasun Jans) and Alex Navarro
