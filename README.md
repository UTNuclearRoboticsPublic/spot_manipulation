# Skeleton and Notes for Building this ReadMe
## Swipe Demo
1) Bringup from `spot_ros`. Expected and relevant service: `spot_manipulation_driver/force_trajectory`. At the time of this writing, ensure `nrg_spot_manipulation` package is under `mary` branch and compile before bringing up the driver.
```
ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3

```
2) Position the arm close to the contact surface using the MoveIt Motion Planning GUI
3) Call the service from command line. Example distance: `distance__y: -0.4`, `force_z: -20`. Force units in Newtons. `speed: 0.05`. Equivalent to 2in/s

## Additional Pertinent Information about the Code
