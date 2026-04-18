#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
import csv
import os

class RawDataLogger(Node):
    def __init__(self):
        super().__init__('raw_data_logger')
        
        # Tertemiz yeni veri dosyamız
        self.csv_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/raw_training_data.csv')
        self.file = open(self.csv_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        
        # Tam 13 Sütun: 6 Açı + 4 Quaternion + 3 Açısal Hız
        self.writer.writerow([
            'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
            'qw', 'qx', 'qy', 'qz',
            'wx', 'wy', 'wz'
        ])
        
        self.joint_names = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]
        self.current_q = None
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        # YENİ DONANIMIMIZI DİNLİYORUZ (UR3 Tabanındaki IMU)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        
        self.get_logger().info('FAZ 1: Ham Veri Hasadı BAŞLATILDI. /base_imu_data dinleniyor...')

    def joint_callback(self, msg):
        if all(name in msg.name for name in self.joint_names):
            self.current_q = [msg.position[msg.name.index(name)] for name in self.joint_names]

    def imu_callback(self, msg):
        if self.current_q is None:
            return
            
        # Quaternion (Husky/UR3 Tabanı Yönelimi)
        qw = msg.orientation.w
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        
        # Açısal Hız (Sarsıntı/Dönüş)
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        
        row = self.current_q + [qw, qx, qy, qz, wx, wy, wz]
        self.writer.writerow(row)

def main(args=None):
    rclpy.init(args=args)
    node = RawDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Kayıt durduruldu. Veri güvenle kaydedildi.")
    finally:
        node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
