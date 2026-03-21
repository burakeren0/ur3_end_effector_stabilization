#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import joblib
import os

class ActiveStabilizer(Node):
    def __init__(self):
        super().__init__('active_stabilizer')
        
        # 1. Eğitilmiş YZ Modelini ve Ölçeklendiricileri Yükle
        model_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/ur3_stabilization_model.pkl')
        scaler_x_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/ur3_scaler_X.pkl')
        scaler_y_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/ur3_scaler_Y.pkl')
        
        self.model = joblib.load(model_path)
        self.scaler_X = joblib.load(scaler_x_path)
        self.scaler_Y = joblib.load(scaler_y_path)
        
        # 2. ROS İletişim Ağı (Abonelikler ve Yayımcılar)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.cmd_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        
        self.last_imu = None
        self.joint_names = ['ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
                            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint']
        
        self.get_logger().info('Aktif Stabilizasyon Kontrolcüsü (YZ) DEVREDE! Titreşimler bekleniyor...')

    def imu_callback(self, msg):
        self.last_imu = [
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]

    def joint_callback(self, msg):
        if self.last_imu is None:
            return
            
        try:
            # 1. Anlık Durumu (State) Oku
            if not all(n in msg.name for n in self.joint_names):
                return

            qs = [msg.position[msg.name.index(n)] for n in self.joint_names]
            dqs = [msg.velocity[msg.name.index(n)] for n in self.joint_names]
            
            # Ağın Girişi: 18 Parametre (Açılar + Hızlar + IMU)
            X_raw = np.array(qs + dqs + self.last_imu).reshape(1, -1)
            
            # 2. YZ Tahmini (Kara Kutu Çalışıyor)
            X_scaled = self.scaler_X.transform(X_raw)                 # Girişi ölçeklendir
            Y_pred_scaled = self.model.predict(X_scaled)              # YZ'ye sor
            Y_pred = self.scaler_Y.inverse_transform(Y_pred_scaled)   # Çıkan sonucu gerçek rad/s hızına çevir
            
            ideal_velocities = Y_pred[0].tolist()

            # 3. Motorlara Komut Gönder (Sarsıntıyı Bastır!)
            # Rapor güvenlik kriteri: Hızlara doyum (limit) ekle
            max_vel = 1.5 # rad/s güvenlik limiti
            safe_velocities = [max(-max_vel, min(max_vel, v)) for v in ideal_velocities]

            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            # Stabilizasyon için anlık hız komutlarını veriyoruz.
            # Konum olarak mevcut konuma çok küçük bir adım (dt=0.05) ekleyerek robotu yönlendiriyoruz.
            dt = 0.05 
            point.positions = [q + (v * dt) for q, v in zip(qs, safe_velocities)]
            point.velocities = safe_velocities
            
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(dt * 1e9)
            
            traj_msg.points.append(point)
            self.cmd_pub.publish(traj_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Kontrol hatası: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ActiveStabilizer()
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
