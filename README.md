# ChonkUR L Rail-E

Descriptions, deployments, and MoveIt configuration for the ChonkUR L Rail-E robot system,
part of the [iMETRO Facility](https://ntrs.nasa.gov/citations/20240013956) at NASA's Johnson Space Center.
This project is intended for use in one of ER4's managed workspaces (such as the `clr_ws`).

The hardware includes the ChonkUR robot mounted on an Ewellix Lift and Vention Linear Rail system.

![alt text](./clr.png "CLR MockUp")

## Usage

The project includes a kinematic simulation for the robot.
To launch the simulation with mock hardware:

```bash
ros2 launch clr_deploy control.launch.py use_fake_hardware:=true
```

A MoveIt RViz widget can then be launched with:

```bash
ros2 launch clr_moveit_config clr_moveit.launch.py
```
