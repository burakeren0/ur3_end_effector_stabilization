#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from dynamic_matrix import robot_dynamics  # Dosya adı .py olmadan yazılır

class ComputedTorqueController(Node):
    def __init__(self):
        super().__init__('computed_torque_node')
        
        # --- 1. KONTROLCÜ AYARLARI (Tuning) ---
        # Kp ve Kv değerlerini UR3'ün ataletine göre optimize ettik
        Kp_degerleri = [15.0, 12.5, 12.5, 12.0, 5.0, 2.0]
        # Kritik sönüm (Critical Damping) formülü: Kv = 2 * sqrt(Kp)
        Kv_degerleri = [2.0 * np.sqrt(k) for k in Kp_degerleri]
        
        self.Kp = np.diag(Kp_degerleri)
        self.Kv = np.diag(Kv_degerleri)
        
        # Hedef Açı (Radyan) - Robotu "Home" pozisyonuna benzer bir yere gönderiyoruz
        self.q_d = np.array([np.pi/4, -np.pi/2, 0.0, -np.pi/4, np.pi/2, 0.0])
        
        # Durum değişkenleri
        self.q = np.zeros(6)
        self.dq = np.zeros(6)
        self.data_received = False 
        
        # UR3 Eklem İsimleri (Gazebo/Hardware tarafındaki isimlerle birebir aynı olmalı)
        self.joint_order = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]

        # --- 2. NUMBA WARM-UP (FONKSİYON ISITMA) ---
        # Numba ilk çalışmada derleme yaptığı için 1-2 saniye gecikebilir.
        # Kontrol döngüsü başlamadan önce sahte bir veriyle bir kez çalıştırıyoruz.
        self.get_logger().info('Numba JIT derlemesi başlatılıyor (Warm-up)...')
        dummy_q = np.zeros(6)
        dummy_dq = np.zeros(6)
        _ = robot_dynamics(dummy_q, dummy_dq) # İlk derleme burada biter
        self.get_logger().info('Numba derlemesi tamamlandı. Sistem hazır.')

        # --- 3. ROS ABONELİK VE YAYINLAR ---
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/ur_effort_controller/commands',
            10
        )
        
        # Kontrol Döngüsü: 100 Hz
        self.timer = self.create_timer(0.01, self.control_loop)

    def joint_state_callback(self, msg):
            """ROS'tan gelen dağınık sensör verisini doğru sıraya dizer."""
            try:
                for i, name in enumerate(msg.name):
                    if name in self.joint_order:
                        idx = self.joint_order.index(name)
                        self.q[idx] = msg.position[i]
                        # Bazı simülatörler hızı boş gönderebilir, kontrol edelim
                        if len(msg.velocity) > i:
                            self.dq[idx] = msg.velocity[i]
                
                self.data_received = True
            except Exception as e:
                self.get_logger().error(f'Veri işleme hatası: {e}')

    def get_robot_matrices(self, q, dq):
            """
            dynamic_matrix.py içindeki Numba ile hızlandırılmış 
            fonksiyonu kullanarak matrisleri döndürür.
            """
            # q ve dq zaten numpy array olarak geliyor
            D, C, g = robot_dynamics(q, dq)
            return D, C, g
            

    def control_loop(self):
        """Hesaplanmış Tork Kontrol (CTC) Matematiği"""
        if not self.data_received:
            return # Sensör verisi gelmediyse bekle
            
        # 1. Hataları Hesapla
        e = self.q_d - self.q
        de = 0.0 - self.dq # Hedef sabit olduğu için hız hatası doğrudan -dq'dur
        
        # 2. Dış Döngü (Sanal Komut 'u')
        # numpy'da matris çarpımı için '@' veya 'np.dot' kullanılır
        u = self.Kp @ e + self.Kv @ de
        
        # 3. İç Döngü (Dinamik Matrisleri Getir)
        D, C, g = self.get_robot_matrices(self.q, self.dq)
        
        # 4. Tork Kuralı: tau = D*u + C*dq + g
        tau = D @ u + C @ self.dq + g
        
        # 5. Robota Gönder
        msg = Float64MultiArray()
        msg.data = tau.tolist() # Numpy array'i normal Python listesine çeviriyoruz
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ComputedTorqueController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Kapatılıyor... Torklar sıfırlanıyor.')
    finally:
        if rclpy.ok() and node.executor and not node.executor.is_shutdown:
            msg = Float64MultiArray()
            msg.data = [0.0] * 6
            node.publisher_.publish(msg)
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()