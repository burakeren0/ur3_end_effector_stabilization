## 📂 Paket Mimarisi (Klasör Yapısı)

* `urdf/`: Husky ve UR3'ü birleştiren temel robot tanımlama (Xacro) dosyaları.
* `launch/`: Simülasyonu, Rviz'i ve haritaları başlatan fırlatma dosyaları.
* `config/`: ROS 2 kontrolcü parametreleri (JTC, DiffDrive, Velocity/Effort Controllers).
* `worlds/`: Python ile üretilen sarsıntı test parkurları (SDF formatında).
* `scripts/`: Harita üretici betikler, veri toplayıcılar ve otonom test (dans) düğümleri.
* `analysis/`: MATLAB kinematik doğrulama kodları ve toplanan simülasyon verileri (CSV).

## 🛠️ Nasıl Çalıştırılır?

**1. terminal (simülasyonu başlatma):**
```bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_end_effector_stabilization <senin_launch_dosyan>.launch.py
```

**2. terminal (kontrolcü düğümünü çalıştırma) :**
```bash
source ~/ur3_ws/install/setup.bash
ros2 control set_controller_state scaled_joint_trajectory_controller inactive
ros2 control set_controller_state ur_effort_controller active
ros2 run ur3_end_effector_stabilization computed_torque_v_node
```
**3. terminal (uç-nokta açı çıktılarını elde etme .csv dosyası):**
```bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_end_effector_stabilization angle_calculator.py --ros-args -p use_sim_time:=true
```

**4. terminal (platformu ile sürme):**
```bash
source ~/ur3_ws/install/setup.bash
ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.0}}}" -r 10
```
