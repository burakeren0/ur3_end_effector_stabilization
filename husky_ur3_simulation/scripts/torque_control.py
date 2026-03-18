#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import random

class UR3TorqueController(Node):
    def __init__(self):
        super().__init__('ur3_torque_controller_node')
        
        # Konu adı yeni kontrolcü adıyla eşleşiyor: ur_effort_controller
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/ur_effort_controller/commands',
            10
        )
        
        self.joint_names = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]
        
        self.get_logger().info('UR3 Rastgele Tork Kontrol Node\'u Başlatıldı...')
        self.get_logger().info('Her 3 saniyede bir rastgele tork değerleri gönderilecek.')
        
        # Her 3 saniyede bir rastgele tork uygulamak için zamanlayıcı
        self.timer = self.create_timer(3.0, self.apply_random_torque)
        self.test_count = 0

    def apply_random_torque(self):
        self.test_count += 1
        # Her eklem için güvenli tork limitleri (Nm)
        # shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
        torque_limits = [5.0, 5.0, 3.0, 1.0, 1.0, 1.0] 
        
        # Limitler dahilinde rastgele tork değerleri oluştur [-limit, limit]
        torques = [random.uniform(-limit, limit) for limit in torque_limits]
        
        msg = Float64MultiArray()
        msg.data = torques
        self.publisher_.publish(msg)
        
        # Gönderilen torkları logla
        log_msg = f'Test {self.test_count}: Torklar gönderildi: '
        log_msg += ', '.join([f'{val:.2f}' for val in torques])
        self.get_logger().info(log_msg)

        if self.test_count >= 10:
            self.get_logger().info('10 test tamamlandı. Tork sıfırlanıyor ve node kapatılıyor.')
            # Torkları sıfırla
            msg.data = [0.0] * 6
            self.publisher_.publish(msg)
            self.timer.cancel()
            
            # Node'u güvenli bir şekilde kapatmak için kısa bir bekleme
            self.create_timer(0.1, self.shutdown_node)

    def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = UR3TorqueController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Klavye ile çıkış yapıldı. Torklar sıfırlanıyor.')
    finally:
        # Spin'den çıkıldığında veya test bittiğinde node zaten kapatılmış olabilir.
        if rclpy.ok() and node.executor and not node.executor.is_shutdown:
            # Çıkışta torkların sıfırlandığından emin ol
            msg = Float64MultiArray()
            msg.data = [0.0] * 6
            node.publisher_.publish(msg)
            node.get_logger().info('Node kapatılıyor, torklar sıfırlandı.')
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()