# ChonkUR

Descriptions, deployments, and MoveIt configuration for the ChonkUR robot,
part of the [iMETRO Facility](https://ntrs.nasa.gov/citations/20240013956) at NASA's Johnson Space Center.
This project is intended for use in one of ER4's managed workspaces (such as the `clr_ws`).

The robot includes:

* UR10e serial manipulator
* Robotiq Hand-E Gripper with custom printed fingers
* Wrist mounted Realsense RGB-D camera

![alt text](./chonkur.png "ChonkUR MockUp")

## Usage

The project includes a kinematic simulation for the robot.
Launching the controllers and hardware interface is done using the provided launch files:

```bash
# To launch the kinematic simulation
ros2 launch chonkur_deploy chonkur_sim.launch.py

# To launch the hardware robot
ros2 launch chonkur_deploy chonkur_hw.launch.py
```

A MoveIt RViz widget can then be launched with:

```bash
ros2 launch chonkur_moveit_config chonkur_moveit.launch.py
```
