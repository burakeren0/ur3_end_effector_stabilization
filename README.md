# UR3 End-Effector Stabilization (ROS 2)

This package provides a ROS 2-based control architecture for stabilizing the UR3 robot arm end-effector mounted on a Husky mobile base. It includes computed-torque and classic feedback controllers (PD, PI-PD, PID), inverse kinematics utilities, launch files for simulation, and helper scripts for data logging and world generation.

**Repository Layout and Detailed Description**

- **CMakeLists.txt**: Top-level CMake build file for the ROS 2 package.
- **package.xml**: ROS 2 package manifest with dependencies and metadata.
- **README.md**: This file.
- **q6_fixed_test.csv, target_actual_angles.csv, target_angles_log.csv**: Example/recorded data CSVs used for offline analysis and plotting.

- **analysis/**: MATLAB scripts and analysis helpers used to validate kinematics and compare logged data.
	- [analysis/matlab_files/compare_rotations.m](analysis/matlab_files/compare_rotations.m): Compare rotation matrices/angles from different sources.
	- [analysis/matlab_files/forward_kinematics.m](analysis/matlab_files/forward_kinematics.m): FK helper used for validating end-effector poses.
	- [analysis/matlab_files/plot_target_angles.m](analysis/matlab_files/plot_target_angles.m): Plotting utilities for target vs actual joint angles.

- **config/**: YAML configuration files for controllers and tuning parameters.
	- [config/husky_ur3_controllers.yaml](config/husky_ur3_controllers.yaml): Controller parameters (PID/PD gains, computed-torque settings, topics, and other tuning constants).

- **include/ur3_end_effector_stabilization/**: C++ header files (public headers used by the nodes in `src/`).

- **launch/**: ROS 2 Python launch files that start simulation, controllers, and the stabilization nodes. Typical launch files:
	- [launch/parkour_pd.launch.py](launch/parkour_pd.launch.py)
	- [launch/parkour_pi_pd.launch.py](launch/parkour_pi_pd.launch.py)
	- [launch/parkour_pid.launch.py](launch/parkour_pid.launch.py)
	- [launch/parkour_ct_pd.launch.py](launch/parkour_ct_pd.launch.py)
	- [launch/parkour_ct_pi_pd.launch.py](launch/parkour_ct_pi_pd.launch.py)
	- [launch/parkour_ct_pid.launch.py](launch/parkour_ct_pid.launch.py)

	Each launch file configures which controller stack to use (PD / PI-PD / PID / computed-torque variations) and accepts launch arguments such as `world` to select a simulation world.

- **rviz/**: RViz configuration files and displays to visualize the robot and trajectories.

- **scripts/**: Python helper scripts used for logging, reading data, and generating worlds.
	- [scripts/matlab_tf_logger.py](scripts/matlab_tf_logger.py): Logs TF and joint states in a format compatible with MATLAB analysis.
	- [scripts/read_target_and_actual_angles.py](scripts/read_target_and_actual_angles.py): Reads and processes CSV logs for plotting or debugging.
	- [scripts/read_target_angles.py](scripts/read_target_angles.py): Utility to read target angle sequences.
	- **world_generators/**: Python generators that produce custom worlds for testing.
		- [scripts/world_generators/generate_bumpy_world.py](scripts/world_generators/generate_bumpy_world.py)
		- [scripts/world_generators/generate_custom_parkour.py](scripts/world_generators/generate_custom_parkour.py)
		- [scripts/world_generators/generate_extreme_parkour.py](scripts/world_generators/generate_extreme_parkour.py)
		- [scripts/world_generators/generate_high_freq_world.py](scripts/world_generators/generate_high_freq_world.py)
		- [scripts/world_generators/generate_hilly_world.py](scripts/world_generators/generate_hilly_world.py)
		- [scripts/world_generators/generate_street_world.py](scripts/world_generators/generate_street_world.py)

- **src/**: Core C++ nodes and controller implementations.
	- [src/computed_torque_pd.cpp](src/computed_torque_pd.cpp): Computed-torque + PD stabilizer implementation.
	- [src/computed_torque_pi_pd.cpp](src/computed_torque_pi_pd.cpp): Computed-torque + PI-PD hybrid controller.
	- [src/computed_torque_pid.cpp](src/computed_torque_pid.cpp): Computed-torque + PID controller variant.
	- [src/inverse_kinematics.cpp](src/inverse_kinematics.cpp): Inverse kinematics utilities used to compute joint targets for a desired end-effector pose.
	- [src/pd_controller.cpp](src/pd_controller.cpp): PD controller node implementation.
	- [src/pi_pd_controller.cpp](src/pi_pd_controller.cpp): PI-PD style controller node.
	- [src/pid_controller.cpp](src/pid_controller.cpp): PID controller node.
	- [src/target_pose_full_rpy.cpp](src/target_pose_full_rpy.cpp): Computes full target pose with roll/pitch/yaw for end-effector tracking.

- **urdf/**: Robot description files (URDF/Xacro) for Husky + UR3 setups.
	- [urdf/mobile_manipulator_moveit.urdf](urdf/mobile_manipulator_moveit.urdf)
	- [urdf/mobile_manipulator.urdf.xacro](urdf/mobile_manipulator.urdf.xacro)

- **worlds/**: Gazebo/SDF world files used for simulation testing and benchmarking.
	- [worlds/bumpy_world.sdf](worlds/bumpy_world.sdf)
	- [worlds/extreme_disturbance.world](worlds/extreme_disturbance.world)
	- [worlds/high_freq_bumpy.world](worlds/high_freq_bumpy.world)
	- [worlds/hilly_world.sdf](worlds/hilly_world.sdf)
	- [worlds/street_world.sdf](worlds/street_world.sdf)


## Requirements

- ROS 2 (compatible distribution)
- ament_cmake build system
- C++ toolchain (gcc/clang) and Eigen3
- ROS 2 dependencies: `rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2`, `tf2_ros`, `robot_state_publisher`, `xacro`
- Optional: `teleop_twist_keyboard` for teleoperation and Husky / UR packages installed in the workspace or system


## Installation and Build

1. From your workspace `src` folder, place or clone this package.

2. Build the workspace (from workspace root):

```bash
cd ~/ur3_ws
colcon build --symlink-install
```

3. Source the workspace overlay in each new shell:

```bash
source ~/ur3_ws/install/setup.bash
```


## Quick Start — Launching Simulation

Pick one of the provided launch files. For example, to start the PD controller scenario:

```bash
ros2 launch ur3_end_effector_stabilization parkour_pd.launch.py
```

To change the simulation world at launch, pass the `world` argument:

```bash
ros2 launch ur3_end_effector_stabilization parkour_pd.launch.py world:=bumpy_world
```

Most launch files will spawn Gazebo (or your chosen simulator), load the robot description, start `controller_manager` and load the appropriate controllers (for example `ur_effort_controller` and `diff_drive_base_controller`), then start the stabilization nodes.


## Monitoring and Utilities

- To log TFs for MATLAB analyses, run: `ros2 run ur3_end_effector_stabilization matlab_tf_logger.py` (use `--ros-args -p use_sim_time:=true` when recording simulation data).
- To inspect or replay saved target/actual angle CSVs, use the Python utilities in `scripts/` such as [scripts/read_target_and_actual_angles.py](scripts/read_target_and_actual_angles.py).


## Development Notes

- The computed-torque nodes compute required joint torques using the robot model and then apply feedback via PD / PI-PD / PID laws. Tuning the gains inside [config/husky_ur3_controllers.yaml](config/husky_ur3_controllers.yaml) is typically required for different payloads and worlds.
- The inverse kinematics implementation in [src/inverse_kinematics.cpp](src/inverse_kinematics.cpp) is used to convert desired end-effector poses (position + orientation) to joint-space targets; this is important when the mobile base moves under the arm.
- RViz configurations under `rviz/` can be used to visualize reference trajectories and the real-time pose of the end-effector.


## Example Commands

Start PD scenario with a bumpy world:

```bash
ros2 launch ur3_end_effector_stabilization parkour_pd.launch.py world:=bumpy_world
```

Drive Husky forward via topic publish:

```bash
ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.0}}}" -r 10
```


## Files Worth Inspecting

- [src/computed_torque_pd.cpp](src/computed_torque_pd.cpp) — starting point for computed-torque behavior.
- [src/target_pose_full_rpy.cpp](src/target_pose_full_rpy.cpp) — how target pose and RPY are computed and published.
- [launch/parkour_ct_pd.launch.py](launch/parkour_ct_pd.launch.py) — example of a computed-torque + PD launch pipeline.
- [config/husky_ur3_controllers.yaml](config/husky_ur3_controllers.yaml) — tuning and mapping of controllers to joints/topics.


## License

This package is licensed under the Apache-2.0 License.

