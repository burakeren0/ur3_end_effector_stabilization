#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import csv
import numpy as np
from scipy.spatial.transform import Rotation as R

class VelocityDataLogger(Node):
    def __init__(self):
        super().__init__('velocity_data_logger')
        
        self.filename = 'robot_kinematics_data.csv'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # Dosyayı aç ve başlıkları yaz
        self.csv_file = open(self.filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        header = ['time', 'q1','q2','q3','q4','q5','q6', 'dq1','dq2','dq3','dq4','dq5','dq6', 
                  'vx','vy','vz', 'wx','wy','wz']
        self.csv_writer.writerow(header)
        
        self.last_time = None
        self.last_pos = None
        self.last_quat = None

        self.get_logger().info(f'Veri kaydedici başlatıldı. Dosya: {self.filename}')
        self.get_logger().info('Robotu hareket ettirdiğinizde veriler kaydedilecek (Ctrl+C ile durdurun)...')

    def joint_callback(self, msg):
        try:
            # 1. Eklem Verilerini Al (UR3 Sıralaması)
            names = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
                     'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
            
            # Gelen mesajdaki eklemleri kontrol et
            if not all(n in msg.name for n in names):
                return

            qs = [msg.position[msg.name.index(n)] for n in names]
            dqs = [msg.velocity[msg.name.index(n)] for n in names]
            
            # 2. TF'den Pozisyon ve Oryantasyon Çek
            # Zaman aşımı ekleyerek TF'nin hazır olmasını bekleyelim
            t = self.tf_buffer.lookup_transform('ur_base_link_inertia', 'ur_tool0', rclpy.time.Time())
            
            curr_pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            curr_quat = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            curr_time = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
            
            vx, vy, vz, wx, wy, wz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            
            if self.last_time is not None:
                dt = curr_time - self.last_time
                if dt > 0.01: # 10ms eşiği (Gürültü ve hız patlamalarını engellemek için)
                    # Çizgisel Hız
                    vel = (curr_pos - self.last_pos) / dt
                    vx, vy, vz = vel
                    
                    # Açısal Hız (Omega)
                    r_curr = R.from_quat(curr_quat)
                    r_last = R.from_quat(self.last_quat)
                    r_delta = r_curr * r_last.inv()
                    angle_axis = r_delta.as_rotvec() 
                    omega = angle_axis / dt
                    wx, wy, wz = omega

                    # Sadece robot gerçekten hareket ediyorsa kaydet (Hızlardan biri 1e-4'ten büyükse)
                    if any(abs(v) > 0.0001 for v in dqs):
                        row = [curr_time] + qs + dqs + [vx, vy, vz, wx, wy, wz]
                        self.csv_writer.writerow(row)
            
            self.last_pos = curr_pos
            self.last_quat = curr_quat
            self.last_time = curr_time
            
        except Exception:
            # TF henüz dolmamış olabilir, sessizce bekle
            pass

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info('Dosya kapatıldı ve kaydedildi.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Eğer rclpy henüz kapanmadıysa kapat
        if rclpy.ok():
            rclpy.shutdown()

# EKSİK OLAN VE ÇALIŞMAYI SAĞLAYAN KISIM:
if __name__ == '__main__':
    main()
