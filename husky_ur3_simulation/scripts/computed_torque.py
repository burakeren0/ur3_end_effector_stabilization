#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

class ComputedTorqueController(Node):
    def __init__(self):
        super().__init__('computed_torque_node')
        
        # --- 1. KONTROLCÜ AYARLARI (Tuning) ---
        # MATLAB'de belirlediğimiz Kp ve Kv değerleri
        Kp_degerleri = [15, 12.5, 12.5, 12.0, 5, 2]
        Kv_degerleri = np.sqrt(Kp_degerleri) * 2.0 # Kritik sönüm için Kv = 2*sqrt(Kp) önerilir
        
        self.Kp = np.diag(Kp_degerleri)
        self.Kv = np.diag(Kv_degerleri)
        
        # Hedef Açı (Radyan cinsinden! Derece yazma patlarız)
        self.q_d = np.array([np.pi/4, -np.pi/2, 0.0, -np.pi/4, np.pi/2, 0.0])
        
        # Durum değişkenleri (Başlangıçta boş, sensörden gelince dolacak)
        self.q = np.zeros(6)
        self.dq = np.zeros(6)
        self.data_received = False # Sensör verisi gelmeden tork üretme!
        
        # Doğru matris sıramız (Tabandan bileğe)
        self.joint_order = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]

        # --- 2. ROS ABONELİK VE YAYINLAR ---
        # Sensör verisini oku (Açı ve Hız)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Tork komutunu gönder
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/ur_effort_controller/commands',
            10
        )
        
        # Kontrol Döngüsü: Saniyede 100 kere (100 Hz = 0.01 saniye) çalışır.
        # CTC gibi dinamik kontrolcüler çok hızlı güncellenmelidir!
        self.timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info('Computed Torque Kontrolcüsü Başlatıldı. Hedef bekleniyor...')

    def joint_state_callback(self, msg):
        """ROS'tan gelen dağınık sensör verisini doğru sıraya dizer."""
        for i, name in enumerate(msg.name):
            if name in self.joint_order:
                idx = self.joint_order.index(name)
                self.q[idx] = msg.position[i]
                self.dq[idx] = msg.velocity[i]
        
        self.data_received = True

    def get_robot_matrices(self, q, dq):
            """
            UR3'ün dinamik matrislerini hesaplar: D(q), C(q, dq), g(q)
            """

# ----------------------------------------------------------------------
            # Değişken Tanımlandırılması
            # ----------------------------------------------------------------------
            th1, th2, th3, th4, th5, th6 = q[0], q[1], q[2], q[3], q[4], q[5]
            th1_dot, th2_dot, th3_dot, th4_dot, th5_dot, th6_dot = dq[0], dq[1], dq[2], dq[3], dq[4], dq[5]

            # g(q) için sık kullanılanlar
            s_234 = np.sin(th2 + th3 + th4)
            c_234 = np.cos(th2 + th3 + th4)
            c_23  = np.cos(th2 + th3)
            c_2   = np.cos(th2)
            c_5   = np.cos(th5)
            s_5   = np.sin(th5)

            # D(q) için sık kullanılan optimizasyon değişkenleri (D[0,0] için şart)
            t34 = th3 + th4
            t234 = th2 + th3 + th4
            t_2th2 = 2.0 * th2
            t_2th23 = 2.0 * (th2 + th3)
            t23 = th2 + th3
            t_2th5 = 2.0 * th5
            
            # ---------------------------------------------------------------------
            # 1. G(q) YERÇEKİMİ VEKTÖRÜNÜN HESAPLANMASI
            # ---------------------------------------------------------------------
            g = np.zeros(6)
            g[0] = 0.0
            g[1] =  0.978 * s_234 - 6.354 * c_23 - 11.905 * c_2 - 0.237 * c_234 * s_5
            g[2] =  0.978 * s_234 - 6.354 * c_23 - 0.237 * c_234 * s_5
            g[3] =  0.978 * s_234 - 0.237 * c_234 * s_5
            g[4] = -0.237 * s_234 * c_5
            g[5] =  0.0

            # ---------------------------------------------------------------------
            # 2. D(q) ATALET MATRİSİNİN HESAPLANMASI
            # ---------------------------------------------------------------------
            D = np.zeros((6, 6)) 

            D[0, 0] = (
                  0.0029 * (np.sin(t34 + th5) + np.sin(t_2th2 + t34 + th5) - np.sin(t34 - th5) - np.sin(t_2th2 + t34 - th5))
                - 0.0026 * (np.sin(th4 - th5) + np.sin(t_2th23 + th4 - th5) - np.sin(th4 + th5) - np.sin(t_2th23 + th4 + th5))
                - 0.0243 * (np.sin(t34) + np.sin(t_2th2 + t34))
                - 0.0213 * (np.sin(t_2th23 + th4) + np.sin(th4))
                + 0.1264 * c_23**2
                + 0.2448 * c_2**2
                + 0.3156 * (np.cos(th2 + 0.5*th3)**2 + np.cos(0.5*th3)**2)
                + 0.0109 * np.cos(0.5*th5)**2
                + 0.0005 * c_5**2
                - 0.0073 * np.cos(t234)**2
                - 0.0021 * (np.cos(t234 - 0.5*th5)**2 - np.cos(t234 + 0.5*th5)**2)
                - 0.0003 * (np.cos(t234 - th5)**2 + np.cos(t234 + th5)**2)
                - 0.2413
            )
            D[0, 1] = (
                  0.0112 * np.cos(t234)
                + 0.0024 * np.cos(t234 - th5)
                + 0.0003 * (np.cos(t234 - t_2th5) - np.cos(t234 + t_2th5) - np.cos(t234 + th5))
                + 0.0026 * (np.sin(t23 + th5) + np.sin(t23 - th5))
                + 0.0029 * (np.sin(th2 - th5) + np.sin(th2 + th5))
                + 0.0520 * np.sin(t23)
                + 0.1066 * np.sin(th2)
                - 0.0005 * c_5
                - 0.0130
              ) 
            
            
            
            # ---------------------------------------------------------------------
            # 3. C(q, dq) CORIOLIS MATRİSİNİN HESAPLANMASI
            # ---------------------------------------------------------------------
            # Aynı şekilde C matrisi de devasa büyüklükte (t252'ye kadar çıkıyor).
            # Şimdilik stabilite testi için C matrisini ihmal edip yerçekimi(g) telafisine odaklanalım.
            C = np.zeros((6, 6))

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