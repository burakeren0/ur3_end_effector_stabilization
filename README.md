# UR3 End Effector Stabilization

Bu paket, Husky mobil taban üstünde taşınan UR3 robot kolunun son efektör stabilizasyonunu sağlayan ROS 2 tabanlı bir kontrol mimarisidir. Hesaplanmış tork kontrolü ile PD / PI-PD / PID tabanlı dengeleme ve ters kinematik hesaplamaları içerir.

## Paket İçeriği

- `urdf/` : Husky ve UR3 robot tanımlarını içeren Xacro/URDF dosyaları.
- `launch/` : Simülasyon, RViz ve farklı kontrol senaryoları için ROS 2 launch dosyaları.
- `config/` : Kontrolör parametreleri, PID/PD/CT ayarları ve konfigürasyonlar.
- `worlds/` : Bumpy, hilly, extreme ve street parkur ortamları.
- `scripts/` : Veri toplama, dünya üretimi ve yardımcı Python düğümleri.
- `analysis/` : MATLAB kinematik doğrulama kodları ve kaydedilmiş veri dosyaları.
- `src/` : Hesaplanmış tork kontrolörleri, ters kinematik ve hedef poz hesaplayıcıları.
- `include/ur3_end_effector_stabilization/` : Paket içinde kullanılan başlık dosyaları.

## Gereksinimler

- ROS 2 (Uyumlu distribüsyon)
- `ament_cmake`
- `rclcpp`, `std_msgs`, `geometry_msgs`, `sensor_msgs`
- `urdf`, `xacro`, `robot_state_publisher`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `Eigen3`
- `teleop_twist_keyboard`
- Husky ve Universal Robots paketleri (çalışma ortamında kurulu olmalıdır)

## Kurulum

1. Workspace dizinine gidin:

```bash
cd ~/ur3_ws/src
```

2. Bu paketi workspace içine yerleştirin veya klonlayın.

3. Workspace kökündeki kaynakları derleyin:

```bash
cd ~/ur3_ws
colcon build --symlink-install
```

4. Her yeni terminalde çalıştırmadan önce workspace ortamını kaynaklayın:

```bash
source ~/ur3_ws/install/setup.bash
```

## Kullanım

### 1. Simülasyonu başlatma

Aşağıdaki launch dosyalarından birini seçerek simülasyonu başlatın:

```bash
ros2 launch ur3_end_effector_stabilization parkour_pd.launch.py
ros2 launch ur3_end_effector_stabilization parkour_pi_pd.launch.py
ros2 launch ur3_end_effector_stabilization parkour_pid.launch.py
ros2 launch ur3_end_effector_stabilization parkour_ct_pd.launch.py
ros2 launch ur3_end_effector_stabilization parkour_ct_pi_pd.launch.py
ros2 launch ur3_end_effector_stabilization parkour_ct_pid.launch.py
```

### 2. Kontrol düğümlerinin başlatılması

Seçtiğiniz launch dosyası, simülasyon ile birlikte ilgili kontrolörleri ve hesaplanmış tork / PD / PI-PD / PID düğümünü otomatik olarak başlatır. Bu nedenle normal kullanımda ayrı bir `ros2 run` komutu girmenize gerek yoktur.

> Paketinizdeki launch dosyaları `controller_manager` aracılığıyla `ur_effort_controller` ve `diff_drive_base_controller` denetleyicilerini yükler ve aynı zamanda hedef poz, ters kinematik ve dengeleme düğümlerini başlatır.

### 3. Son efektör açılarını izleme

Son efektör açılarını almak için:

```bash
ros2 run ur3_end_effector_stabilization angle_calculator.py --ros-args -p use_sim_time:=true
```

### 4. Husky mobil tabanını sürme

Husky’yi ileri doğru sürmek için aşağıdaki komutu kullanın:

```bash
ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.0}}}" -r 10
```

## Öne Çıkan Dosyalar

- `src/computed_torque_pd.cpp` : Computed torque + PD kontrolü.
- `src/computed_torque_pi_pd.cpp` : Computed torque + PI-PD kontrolü.
- `src/computed_torque_pid.cpp` : Computed torque + PID kontrolü.
- `src/inverse_kinematics.cpp` : Ters kinematik hesaplamaları.
- `src/target_pose_full_rpy.cpp` : Hedef poz ve RPY hesaplamaları.
- `scripts/world_generators/` : Simülasyon parkurlarını üreten Python betikleri.

## Öneriler

- Farklı kontrol stratejilerini test etmek için farklı launch dosyalarını kullanın.
- Simülasyon ortamını değiştirmek için `worlds/` klasöründeki world dosyalarını veya `scripts/world_generators/` betiklerini kullanın.
- Yeni bir terminal açtığınızda `source ~/ur3_ws/install/setup.bash` komutunu unutmayın.

## Lisans

Bu paket `Apache-2.0` lisansı ile lisanslanmıştır.
