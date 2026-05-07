#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

M_PI = math.pi
# ==========================================
# KULLANICI AYARLARI (Açiları Buraya Gir)
# ==========================================
HEDEF_ACILAR = [0, -M_PI/2.0, M_PI/2.0, -M_PI, 0.0, 0.0]
HAREKET_SURESI = 7.0 
# ==========================================

class MoveAndMeasure(Node):
    def __init__(self):
        super().__init__('move_and_measure_node')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_names = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]
        
        self.get_logger().info('Node Başlatıldı. 1 saniye içinde hareket emri gönderilecek...')
        self.timer_start = self.create_timer(1.0, self.send_movement_command)
        self.timer_measure = None 

    def send_movement_command(self):
        self.timer_start.cancel() 
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        
        point.positions = HEDEF_ACILAR
        point.time_from_start = Duration(sec=int(HAREKET_SURESI))
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Hedef gönderildi: {HEDEF_ACILAR}')
        self.get_logger().info(f'Robot harekete başladı... {HAREKET_SURESI} saniye bekleniyor...')
        
        self.timer_measure = self.create_timer(HAREKET_SURESI + 1.0, self.measure_pose)

    def tool0_to_eelink_quaternion(self, qx, qy, qz, qw):
        """
        ur_tool0 oryantasyonunu matematiksel olarak ur_ee_link oryantasyonuna çevirir.
        ee_link, tool0'ın yerel Z ekseninde +90 derece dönmüş halidir.
        """
        half_pi = math.pi / 4.0 # Z ekseni etrafında 90 derece rotasyon için (pi/2) / 2
        rz = math.sin(half_pi)
        rw = math.cos(half_pi)
        
        # Yerel Hamilton Çarpımı (q_new = q_old * q_rot)
        qx_new = qx*rw + qy*rz
        qy_new = qy*rw - qx*rz
        qz_new = qz*rw + qw*rz
        qw_new = qw*rw - qz*rz
        
        return qx_new, qy_new, qz_new, qw_new

    def measure_pose(self):
        self.timer_measure.cancel()
        
        try:
            # TF Ağacında ee_link olmadığı için mecburen tool0 okuyoruz.
            target_frame = 'ur_base_link_inertia' 
            source_frame = 'ur_tool0'      
            
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            
            # --- HAYAT KURTARAN MATEMATİKSEL DÜZELTME ---
            # tool0 quaternionunu, C++ IK çözücüsünün beklediği ee_link quaternionuna dönüştürüyoruz
            qx, qy, qz, qw = self.tool0_to_eelink_quaternion(qx, qy, qz, qw)
            # --------------------------------------------
            
            print("\n" + "="*50)
            print(f"HEDEF AÇILAR (Rad): {HEDEF_ACILAR}")
            print("-" * 50)
            print(f"VARIŞ NOKTASI (Sanal '{target_frame} -> ur_ee_link' Çevirimi):")
            print(f"Position (m) : X={x:.5f}, Y={y:.5f}, Z={z:.5f}")
            print(f"Orientation  : Qx={qx:.4f}, Qy={qy:.4f}, Qz={qz:.4f}, Qw={qw:.4f}")
            print("="*50 + "\n")
            
            self.get_logger().info("İşlem tamamlandı. Node kapatılıyor...")
            raise SystemExit 
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'TF verisi okunamadı: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MoveAndMeasure()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()