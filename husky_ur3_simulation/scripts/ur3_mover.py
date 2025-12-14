#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class UR3Mover(Node):
    def __init__(self):
        super().__init__('ur3_mover_node')
        
        # Publisher: Joint Trajectory Controller'a emir gönderir
        # Topic ismi controller yaml dosyasındaki isme göre belirlenir
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # UR3 eklem isimleri (tf_prefix="ur_" olduğu için başlarına ekledik)
        self.joint_names = [
            'ur_shoulder_pan_joint',
            'ur_shoulder_lift_joint',
            'ur_elbow_joint',
            'ur_wrist_1_joint',
            'ur_wrist_2_joint',
            'ur_wrist_3_joint'
        ]
        
        self.get_logger().info('UR3 Hareket Node Başlatıldı...')
        
        # 2 saniye bekleyip hareketi başlat (Bağlantıların oturması için)
        self.timer = self.create_timer(2.0, self.move_robot)
        self.movement_stage = 0

    def move_robot(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        
        if self.movement_stage == 0:
            # 1. HAREKET: Home Pozisyonu (Dik duruş)
            self.get_logger().info('Hareket 1: Home Pozisyonuna Gidiliyor...')
            # Hedef açılar (Radyan cinsinden)
            point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            point.time_from_start = Duration(sec=4) # 4 saniyede git
            self.movement_stage += 1
            
        elif self.movement_stage == 1:
            # 2. HAREKET: Öne Eğilme (Pick Pozisyonu gibi)
            self.get_logger().info('Hareket 2: Öne Eğiliyor...')
            point.positions = [1.57, -1.0, 1.0, -1.57, -1.57, 0.0] # Sağa dön ve eğil
            point.time_from_start = Duration(sec=4)
            self.movement_stage += 1
            
        else:
            # Döngü başa döner
            self.movement_stage = 0
            return # Fonksiyondan çık, bir sonraki timer tetiklemesinde başa dönecek

        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR3Mover()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()