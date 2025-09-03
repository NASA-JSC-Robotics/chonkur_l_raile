# CLR Mujoco Config

Mujoco configuration for launching CLR in a simulated Mujoco environment.

## Launching

The program must be launched from inside a supported environment.
Details are not included here, but can be found in the `ros2_mujoco_simulation` repo.

To start the sim,

```bash
ros2 launch clr_mujoco_config clr_mujoco.launch.py
```

## Conversion

We provide a [launch file](./launch/generate_clr_mjcf.launch.py) to run the MJCF conversion tool against the description file in [clr_xacro.urdf](./urdf/clr_xacro.urdf).
By default, the file will use the included [mujoco_inputs.xml](./description/mujoco_inputs.xml).

To run the converter:

```bash
ros2 launch clr_mujoco_config generate_clr_mjcf.launch.py
```

The resulting output will be written to a folder called `mjcf_data` in the current directory.
From there, the contents can be simulated with,

```bash
${MUJOCO_DIR}/bin/simulate mjcf_data/scene.xml
```

Any contents can be copied and updated as needed.
