#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from tf2_ros import Buffer, TransformListener
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class TeacherDataLogger(Node):
    def __init__(self):
        super().__init__('teacher_data_logger')
        
        import os
        self.filename = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/supervised_training_data_raw.csv')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Abonelikler
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        
        self.csv_file = open(self.filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # 1. Zaman | 2-13. Ağın Girişi (q, dq) | 14-19. Ağın Girişi (IMU) | 20-25. Öğretmenin İhtiyacı (Gerçek Taban Hızı)
        header = ['time', 
                  'q1','q2','q3','q4','q5','q6', 
                  'dq1','dq2','dq3','dq4','dq5','dq6', 
                  'ax','ay','az', 'imu_wx','imu_wy','imu_wz',
                  'v_bx','v_by','v_bz', 'w_bx','w_by','w_bz']
        self.csv_writer.writerow(header)
        
        # Geçici hafıza değişkenleri
        self.last_imu = None
        self.last_time = None
        self.last_pos = None
        self.last_quat = None

        self.get_logger().info('Öğretmen Veri Kaydedici BAŞLATILDI.')
        self.get_logger().info(f'Veriler {self.filename} konumuna kaydediliyor...')

    def imu_callback(self, msg):
        # IMU'dan gelen çizgisel ivme ve açısal hızları kaydet
        self.last_imu = [
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]

    def joint_callback(self, msg):
        if self.last_imu is None:
            return # IMU verisi gelmeden kaydetme
            
        try:
            names = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
                     'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
            
            if not all(n in msg.name for n in names):
                return

            qs = [msg.position[msg.name.index(n)] for n in names]
            dqs = [msg.velocity[msg.name.index(n)] for n in names]
            
            # Tabanın (ur_base_link_inertia) odom'a (Dünya) göre konumunu al (Gerçek sarsıntı hızını bulmak için)
            t = self.tf_buffer.lookup_transform('odom', 'ur_base_link_inertia', rclpy.time.Time())
            
            curr_pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            curr_quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            curr_time = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            
            if self.last_time is not None:
                dt = curr_time - self.last_time
                if dt > 0.01: # 10ms (100 Hz) eşiği
                    # Çizgisel Hız (Türev)
                    vel = (curr_pos - self.last_pos) / dt
                    
                    # Açısal Hız (Türev)
                    r_curr = R.from_quat(curr_quat)
                    r_last = R.from_quat(self.last_quat)
                    r_delta = r_curr * r_last.inv()
                    omega = r_delta.as_rotvec() / dt

                    # Eğer Husky hareket ediyorsa (sarsılıyorsa) kaydet
                    if np.linalg.norm(vel) > 0.01 or np.linalg.norm(omega) > 0.01:
                        row = [curr_time] + qs + dqs + self.last_imu + vel.tolist() + omega.tolist()
                        self.csv_writer.writerow(row)
            
            self.last_pos = curr_pos
            self.last_quat = curr_quat
            self.last_time = curr_time
            
        except Exception:
            pass

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info('Kayıt Tamamlandı ve Dosya Kapatıldı.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeacherDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
