# Description
This repository contains a ROS package with a python wrapper and ROS node to produce follow_joint_trajectory action servers for the Spot arm and finger.
Additionally, twist command subscribers for the end-effector are also brought up.</br>
# Instructions</br>
Execute the following commands from two separate terminals:</br>
```rosrun spot_manipulation_driver follow_joint_trajectory_action_server 192.168.50.3```</br>
```roslaunch spot_manipulation_driver cameras.launch```</br>
