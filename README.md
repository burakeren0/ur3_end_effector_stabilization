## Package Architecture (Folder Structure)

* `urdf/`: Basic robot definition (Xacro) files combining Husky and UR3.
* `launch/`: Launch files that launch the simulation, Rviz and maps.
* `config/`: ROS 2 controller parameters (JTC, DiffDrive, Velocity/Effort Controllers).
* `worlds/`: Shake test tracks generated with Python (in SDF format).
* `scripts/`: Map generator scripts, data collectors, neural-network files and autonomous test (dance) nodes.
* `analysis/`: MATLAB kinematic verification codes and collected simulation data (CSV).


## Creating a Workspace

Note: First, you need to install the Universal Robots and Clearpath packages.

```bash
cd ~
mkdir -p ur3_ws/src
cd ~/ur3_ws/src
git clone https://github.com/burakeren0/ur3_end_effector_stabilization.git
```



## How to operate it?

**1. terminal (Starting the simulation):**
```bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_end_effector_stabilization <senin_launch_dosyan>.launch.py
```

**2. terminal (running the controller node) :**
```bash
source ~/ur3_ws/install/setup.bash
ros2 control set_controller_state scaled_joint_trajectory_controller inactive
ros2 control set_controller_state ur_effort_controller active
ros2 run ur3_end_effector_stabilization computed_torque_v_node
```
**3. terminal (obtaining end-point angle outputs):**
```bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_end_effector_stabilization angle_calculator.py --ros-args -p use_sim_time:=true
```

**4. terminal (Drive the Husky robot forward):**
```bash
source ~/ur3_ws/install/setup.bash
ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.0}}}" -r 10
```
