#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class HuskyCircle(Node):
    def __init__(self):
        super().__init__('husky_circle_node')
        
        # Husky'nin tekerlek kontrolcüsüne hız mesajı gönderiyoruz.
        # Controller ayarlarında use_stamped_vel: false olduğu için 'cmd_vel_unstamped' kullanıyoruz.
        self.publisher_ = self.create_publisher(
            Twist, 
            '/diff_drive_base_controller/cmd_vel_unstamped', 
            10
        )
        
        # 0.1 saniyede bir (10 Hz) komut gönder
        self.timer = self.create_timer(0.1, self.move_in_circle)
        self.get_logger().info('Husky daire çizmeye başlıyor! 🔄')

    def move_in_circle(self):
        msg = Twist()
        
        # Daire Çizme Mantığı:
        # Hem ileri git (Linear X) hem de dön (Angular Z)
        
        msg.linear.x = 0.5   # İleri Hız (m/s)
        msg.angular.z = 0.5  # Dönme Hızı (rad/s)
        
        # Not: Yarıçap = linear.x / angular.z formülüyle hesaplanır.
        # Burada 0.5 / 0.5 = 1 metre yarıçapında dönecektir.
        
        self.publisher_.publish(msg)

    def stop_robot(self):
        """Node kapanırken robotu durdurmak için"""
        self.get_logger().info('Durduruluyor...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HuskyCircle()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C yapıldığında robotu durdur ve kapat
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()