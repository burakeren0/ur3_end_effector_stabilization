#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
import time

# ==========================================
# AYARLAR
# ==========================================
JOINT_NAMES = [
    'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
    'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
]
TRAJECTORY_TOPIC = '/scaled_joint_trajectory_controller/joint_trajectory'
TARGET_FRAME = 'ur_base_link' 
SOURCE_FRAME = 'ur_tool0'       

# HEDEF: Robotu -0.2 rad/s hıza ulaştırmak
TARGET_SPEED = -0.2 
TOTAL_DURATION = 4.0 # Hareket süresi uzatıldı (Daha yumuşak geçiş için)
MEASURE_TIME = TOTAL_DURATION / 2.0 # Tam ortada ölçüm yapacağız
DELTA_T = 0.2
# ==========================================

class FixedVelocityTest(Node):
    def __init__(self):
        super().__init__('fixed_velocity_test')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        
        self.traj_pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_joints = {}
        self.current_velocities = {}
        self.test_started = False
        
        self.get_logger().info('Node Hazır. Veri bekleniyor...')

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES:
                self.current_joints[name] = msg.position[i]
                if len(msg.velocity) > 0:
                    self.current_velocities[name] = msg.velocity[i]
                else:
                    self.current_velocities[name] = 0.0

        if not self.test_started and all(j in self.current_joints for j in JOINT_NAMES):
            self.test_started = True
            time.sleep(1.0)
            self.execute_smart_trajectory()

    def execute_smart_trajectory(self):
        self.get_logger().info('Akıllı Yörünge Hesaplanıyor...')
        
        start_pos = [self.current_joints[name] for name in JOINT_NAMES]
        
        # --- 3 NOKTALI YÖRÜNGE OLUŞTURUYORUZ ---
        
        # 1. ORTA NOKTA HESABI (Hızın maksimum olduğu yer)
        # Konum = Başlangıç + (Hız * (Süre/2))
        mid_pos = list(start_pos)
        mid_pos[1] = start_pos[1] + (TARGET_SPEED * (TOTAL_DURATION / 2.0))
        
        # 2. BİTİŞ NOKTASI HESABI (Robotun duracağı yer)
        # Konum = Başlangıç + (Hız * Süre)
        end_pos = list(start_pos)
        end_pos[1] = start_pos[1] + (TARGET_SPEED * TOTAL_DURATION)
        
        traj_msg = JointTrajectory()
        traj_msg.header.frame_id = TARGET_FRAME
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = JOINT_NAMES
        
        # NOKTA 1: BAŞLANGIÇ (t=0, v=0) -> DURUYORUZ
        p1 = JointTrajectoryPoint()
        p1.positions = start_pos
        p1.velocities = [0.0] * 6
        p1.time_from_start = Duration(sec=0, nanosec=0)
        
        # NOKTA 2: ORTA (t=2.0, v=-0.2) -> HIZLANIYORUZ
        # Burası bizim ölçüm yapacağımız yer!
        p2 = JointTrajectoryPoint()
        p2.positions = mid_pos
        velocities_mid = [0.0] * 6
        velocities_mid[1] = TARGET_SPEED # İstediğimiz hızı buraya yazıyoruz
        p2.velocities = velocities_mid
        p2.time_from_start = Duration(sec=int(TOTAL_DURATION/2), nanosec=0)

        # NOKTA 3: BİTİŞ (t=4.0, v=0) -> DURUYORUZ (Hatayı önleyen kısım burası!)
        p3 = JointTrajectoryPoint()
        p3.positions = end_pos
        p3.velocities = [0.0] * 6 # Son hız SIFIR olmak zorunda
        p3.time_from_start = Duration(sec=int(TOTAL_DURATION), nanosec=0)
        
        traj_msg.points = [p1, p2, p3]
        
        self.traj_pub.publish(traj_msg)
        self.get_logger().info(f'Yörünge Gönderildi! {MEASURE_TIME} saniye sonra (tam ortada) ölçülecek...')
        
        # Tam orta noktada ölçüm yap
        time.sleep(MEASURE_TIME)
        self.measure_and_report()

    def measure_and_report(self):
        try:
            # T1
            tf1 = self.tf_buffer.lookup_transform(TARGET_FRAME, SOURCE_FRAME, rclpy.time.Time())
            
            # Gerçek sensör verilerini yakala
            q_snap = [self.current_joints[name] for name in JOINT_NAMES]
            q_dot_snap = [self.current_velocities.get(name, 0.0) for name in JOINT_NAMES]
            
            time.sleep(DELTA_T)
            
            # T2
            tf2 = self.tf_buffer.lookup_transform(TARGET_FRAME, SOURCE_FRAME, rclpy.time.Time())
            
            # Hız Hesabı
            p1 = tf1.transform.translation
            p2 = tf2.transform.translation
            
            vx = (p2.x - p1.x) / DELTA_T
            vy = (p2.y - p1.y) / DELTA_T
            vz = (p2.z - p1.z) / DELTA_T
            
            print("\n" + "="*70)
            print("MATLAB DOĞRULAMASI İÇİN SONUÇLAR (BAŞARILI)")
            print("="*70)
            print(f"1. MATLAB 'q_dot' (Gerçek Sensör Verisi) [rad/s]:")
            # Virgülden sonra 4 basamak gösterelim ki okunur olsun
            print([float(f"{x:.5f}") for x in q_dot_snap])
            print("-" * 70)
            print(f"2. MATLAB 'q' (Gerçek Açı) [rad]:")
            print([float(f"{x:.5f}") for x in q_snap])
            print("-" * 70)
            print(f"3. ÖLÇÜLEN HIZ (V_measured) [m/s]:")
            print(f"Vx: {vx:.5f}")
            print(f"Vy: {vy:.5f}")
            print(f"Vz: {vz:.5f}")
            print("="*70 + "\n")
            
            raise SystemExit

        except Exception as e:
            self.get_logger().error(f"Hata: {str(e)}")
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = FixedVelocityTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()