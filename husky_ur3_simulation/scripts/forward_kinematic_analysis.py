#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

# ==========================================
# KULLANICI AYARLARI (Açiları Buraya Gir)
# ==========================================
# [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3] (Radyan cinsinden)
HEDEF_ACILAR = [0.0, -1.57/2, -1.57/6, -1.57/2.1, 1.57, 0.0]

# Hareketin tamamlanması için beklenecek süre (Saniye)
HAREKET_SURESI = 7.0 
# ==========================================

class MoveAndMeasure(Node):
    def __init__(self):
        super().__init__('move_and_measure_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # TF Listener (Konum okumak için)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_names = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]
        
        self.get_logger().info('Node Başlatıldı. 1 saniye içinde hareket emri gönderilecek...')
        
        # Sistem otursun diye 1 saniye sonra hareketi başlat
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
        
        # Hareket süresi bittikten biraz sonra ölçüm yap
        self.timer_measure = self.create_timer(HAREKET_SURESI + 1.0, self.measure_pose)

    def measure_pose(self):
        self.timer_measure.cancel()
        
        try:
            # --- GÜNCELLENEN KISIM ---
            # TF Ağacına göre doğru frame isimleri:
            target_frame = 'ur_base_link_inertia'  # Referans (Sabit taban)
            source_frame = 'ur_tool0'      # Hedef (Uç nokta)
            
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            # -------------------------
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            
            print("\n" + "="*50)
            print(f"HEDEF AÇILAR (Rad): {HEDEF_ACILAR}")
            print("-" * 50)
            print(f"VARIŞ NOKTASI (Frame: {target_frame} -> {source_frame}):")
            print(f"Position (m) : X={x:.5f}, Y={y:.5f}, Z={z:.5f}")
            print(f"Orientation  : Qx={qx:.4f}, Qy={qy:.4f}, Qz={qz:.4f}, Qw={qw:.4f}")
            print("="*50 + "\n")
            
            self.get_logger().info("İşlem tamamlandı. Node kapatılıyor...")
            raise SystemExit 
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'TF verisi okunamadı: {str(e)}')
            self.get_logger().info('İpucu: Frame isimleri "ur_base_link_inertia" ve "ur_tool0" olarak denendi.')

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