#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
import joblib
import numpy as np
import os

class ActiveStabilizerV3(Node):
    def __init__(self):
        super().__init__('active_stabilizer_v3')
        
        # 1. Eğitilmiş Beyni ve Ölçeklendiriciyi Yükle
        model_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/stabilizer_model_v3.pkl')
        scaler_path = os.path.expanduser('~/ur3_ws/src/mobile_manipulator/analysis/feature_scaler_v3.pkl')
        
        self.model = joblib.load(model_path)
        self.scaler = joblib.load(scaler_path)
        
        self.joint_names = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]
        
        self.current_q = None
        self.latest_imu = None
        
        # L-Pozisyonu ve Yumuşak Yay (Kp)
        self.q_home = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
        self.Kp = 0.5 
        
        # Gerçek Zamanlı EMA Filtresi
        self.alpha = 0.15 
        self.filtered_w = np.array([0.0, 0.0, 0.0])
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/ur_velocity_controller/commands', 10)
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        
        self.get_logger().info("V3 STABİLİZATÖRÜ (99.8% BAŞARI) DEVREDE! Bekleniyor...")

    def joint_callback(self, msg):
        if all(name in msg.name for name in self.joint_names):
            self.current_q = [msg.position[msg.name.index(name)] for name in self.joint_names]

    def imu_callback(self, msg):
        self.latest_imu = msg

    def control_loop(self):
        if self.current_q is None or self.latest_imu is None:
            return
            
        raw_w = np.array([
            self.latest_imu.angular_velocity.x, 
            self.latest_imu.angular_velocity.y, 
            self.latest_imu.angular_velocity.z
        ])
        
        self.filtered_w = (self.alpha * raw_w) + ((1.0 - self.alpha) * self.filtered_w)
        wx, wy, wz = self.filtered_w
        
        current_q_np = np.array(self.current_q)
        error = self.q_home - current_q_np
        p_velocity = self.Kp * error

        # EĞER DÜZ YOLDAYSA:
        if abs(wx) < 0.01 and abs(wy) < 0.01 and abs(wz) < 0.01:
            safe_velocity = np.clip(p_velocity, -0.5, 0.5)
            self.publish_velocity_command(safe_velocity)
            # Yapay zekanın uyuduğunu saniyede bir kez bize bildir:
            if np.random.rand() < 0.05:
                self.get_logger().info("Robot düz yolda. Sarsıntı yok. L-Pozisyonu kilitli.")
            return

        # SARSINTI VARSA AI DEVREYE GİRER:
        raw_features = np.array([self.q_home.tolist() + [wx, wy, wz]])
        scaled_features = self.scaler.transform(raw_features)
        ai_velocity = self.model.predict(scaled_features)[0]
        
        final_velocity = ai_velocity + p_velocity
        safe_final_velocity = np.clip(final_velocity, -1.0, 1.0) 
        
        # AI'ın motorlara ne hız verdiğini anlık olarak ekrana yazdır!
        self.get_logger().info(f"Sarsıntı! Wz: {wz:.2f} -> AI Omuz Dönüş Hızı (Pan): {ai_velocity[0]:.2f}")
        
        self.publish_velocity_command(safe_final_velocity)

    def publish_velocity_command(self, dq):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in dq]
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActiveStabilizerV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Aktif stabilizasyon sonlandırıldı.")
    finally:
        node.publish_velocity_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
