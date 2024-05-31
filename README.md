# Swipe Demo
1) Bringup from `spot_ros`. Expected and relevant service: `spot_manipulation_driver/force_trajectory`. Ensure `nrg_spot_manipulation` package is under `mary` branch and compile before bringing up the driver.
```
ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3

```
2) Position the arm close to the contact surface using the MoveIt Motion Planning GUI
3) Call the service from command line. Example distance: `distance__y: -0.4`, `force_z: -20`. Force units in Newtons. `speed: 0.05`. Equivalent to 2in/s

##Additional Pertinent Information about the Code
To switch plane force is applied on, in the function `arm_force_trajectory_executor` in `spot_manipulaton_driver.py`, the axis mode of each axis must be adjusted accordingly under `arm_cartesian_command`, and the quaternion, `q`, needs to be adjusted to point the end-effector in the appropriate direction.


# Non Contact Demo
1) Bringup from `spot_ros`. Expected and relevant service: `spot_manipulation_driver/ncontact_trajectory`. Ensure `nrg_spot_manipulation` package is under `mary` branch and compile before bringing up the driver.
```
ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3

```
2) Position the arm close to the desired offset from the surface using the MoveIt Motion Planning GUI
3) Call the service from command line. Example distance: `distance__y: -0.4`, `offset_z: 0.3`. Distance units in meters. `speed: 0.05`. Equivalent to 2in/s

## Additional Pertinent Information about the Code
To switch plane offset is applied from (only if distance is no longer being applied strictly in y), in the function `arm_ncontact_trajectory_executor` in `spot_manipulaton_driver.py`:
1) the predicted positions of each axis must be adjusted such that the axis the movement is directed in does not contain the offset, 
2) `linear_velocity` tuning must be adjusted to exclude the direction(s) in which movement distance is specified.
