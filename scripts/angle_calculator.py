#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener
import math
import csv
import os

class AngleCalculator(Node):
    def __init__(self):
        super().__init__('angle_calculator')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # CSV dosyasını çalışma alanının kök dizinine oluştur
        self.csv_file_path = os.path.expanduser('~/ur3_ws/angle_data.csv')
        
        # Dosyayı ilk açtığında başlıkları yaz (Eski verileri ezer)
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Zaman_s', 'Roll_Derece', 'Pitch_Derece', 'Yaw_Derece'])
            
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Saniyede 20 defa on_timer fonksiyonunu tetikler (20 Hz)
        self.timer = self.create_timer(0.05, self.on_timer)
        
        self.get_logger().info(f'Veriler matris formatinda kaydediliyor: {self.csv_file_path}')

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def on_timer(self):
        try:
            # Zemin ('base_link') ve Uç Nokta ('ur_tool0')
            t = self.tf_buffer.lookup_transform('base_link', 'ur_tool0', rclpy.time.Time())
            quat = t.transform.rotation
            roll, pitch, yaw = self.euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
            
						# Zamanı ve açıları dereceye çevirip kaydet
            current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg = math.degrees(yaw)
            
            # Üç açıyı da CSV'ye ekle
            with open(self.csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([current_time, roll_deg, pitch_deg, yaw_deg])
                
        except TransformException:
            # Hata verirse terminali spamlememesi için 2 saniyede bir yazdırır
            self.get_logger().info('TF Bekleniyor...', throttle_duration_sec=2.0)

def main():
    rclpy.init()
    node = AngleCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
