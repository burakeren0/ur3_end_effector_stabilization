# Mobile Manipulator: UR3 End-Effector Stabilization

Bu proje, engebeli arazilerde hareket eden bir mobil manipülatörün (Clearpath Husky + Universal Robots UR3), gövdede oluşan sarsıntılara rağmen uç işlevcisini (end-effector) uzayda sabit bir konumda ve yönelimde tutabilmesi (Active Stabilization / Active Suspension) amacıyla geliştirilmektedir.

Proje **ROS 2 Jazzy** ve **Gazebo Harmonic** altyapısı kullanılarak simüle edilmektedir.

## 🚀 Projede Şu Ana Kadar Neler Yapıldı?

1. **Donanım ve Simülasyon Entegrasyonu:**
   * Husky (diferansiyel sürüş) ve UR3 (6 serbestlik dereceli kol) tek bir Xacro/URDF dosyasında birleştirildi.
   * `gz_ros2_control` üzerindeki donanım arayüzü çakışmaları (Hardware Interface Conflicts) çözülerek iki sistemin tek bir Controller Manager altında uyumlu çalışması sağlandı.

2. **Prosedürel Test Ortamları (Worlds):**
   * Süspansiyon ve stabilizasyon testleri için Python betikleri kullanılarak prosedürel Gazebo haritaları üretildi:
     * `bumpy_world.sdf`: Rastgele dağıtılmış taşlar ve engebelerden oluşan kaotik parkur.
     * `street_world.sdf`: Tam tümsekler, asimetrik (tek tekerlek) tümsekler ve çukurlardan oluşan standart cadde parkuru.

3. **Matematiksel Modelleme (Kinematics & Jacobian):**
   * UR3 için İleri Kinematik (Forward Kinematics) modeli sembolik olarak çıkarıldı.
   * Çapraz çarpım yöntemiyle $6 \times 6$'lık **Geometrik Jakobi (Jacobian)** matrisi türetilerek uç noktanın Diferansiyel Hız modeli kuruldu (MATLAB).

4. **Veri Toplama ve Doğrulama (Kinematic Validation):**
   * ROS 2 üzerinden pürüzsüz sinüzoidal yörüngeler üreten bir düğüm (`robot_dancer.py`) ile robota hareket verildi.
   * Eşzamanlı olarak eklem durumları (`/joint_states`) ve TF ağacı üzerinden uç nokta hızları ölçülerek kaydedildi (`velocity_kinematics_analysis.py`).
   * MATLAB'de teorik Jacobian matrisi ile simülasyondan alınan gerçek veriler karşılaştırıldı. **Genel Ortalama Hata (RMSE): ~0.022** çıkarak model %100'e yakın bir oranla doğrulandı.

## 📂 Paket Mimarisi (Klasör Yapısı)

* `urdf/`: Husky ve UR3'ü birleştiren temel robot tanımlama (Xacro) dosyaları.
* `launch/`: Simülasyonu, Rviz'i ve haritaları başlatan fırlatma dosyaları.
* `config/`: ROS 2 kontrolcü parametreleri (JTC, DiffDrive, Velocity/Effort Controllers).
* `worlds/`: Python ile üretilen sarsıntı test parkurları (SDF formatında).
* `scripts/`: Harita üretici betikler, veri toplayıcılar ve otonom test (dans) düğümleri.
* `analysis/`: MATLAB kinematik doğrulama kodları ve toplanan simülasyon verileri (CSV).

## 🛠️ Nasıl Çalıştırılır?

**1. Çalışma Alanını Derleme:**
```bash
cd ~/ur3_ws
colcon build --symlink-install --packages-select mobile_manipulator
source install/setup.bash
```

**2. Cadde/Sokak Sarsıntı Testini Başlatma:**
```bash
ros2 launch mobile_manipulator street_terrain_test.launch.py
```

**3. Robotu Engebelerde Sürme:**
```bash
ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}, angular: {z: 0.0}}}" -r 10
```

