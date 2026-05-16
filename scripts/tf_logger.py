import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import csv
import time
import os  # Klasör ve dosya yolları yönetimi için eklendi

def euler_from_quaternion(x, y, z, w):
    """Quaternion'u Euler (Roll, Pitch, Yaw) açılarına dönüştürür."""
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

class TransformLogger(Node):
    def __init__(self):
        super().__init__('tf_logger')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- HEDEF KLASÖR VE DOSYA YOLU AYARLARI ---
        output_dir = "/home/taylan/ur3_ws/src/ur3_end_effector_stabilization/analysis"
        os.makedirs(output_dir, exist_ok=True)  # 'analysis' klasörü yoksa otomatik oluşturur
        file_path = os.path.join(output_dir, 'simulasyon_verisi.csv')
        
        # CSV dosyasını belirtilen tam yolda oluştur ve başlıkları yaz
        self.csv_file = open(file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Time', 
            'Base_X_World', 'Base_Y_World', 'Base_Z_World', 'Base_Roll_W', 'Base_Pitch_W', 'Base_Yaw_W',
            'EE_X_Base', 'EE_Y_Base', 'EE_Z_Base',
            'EE_Roll_W', 'EE_Pitch_W', 'EE_Yaw_W'
        ])
        
        self.start_time = time.time()
        self.timer = self.create_timer(0.05, self.on_timer) # 20 Hz örnekleme hızı
        self.get_logger().info(f"TF Veri Kaydedici Başladı.\nVeriler şuraya yazılıyor: {file_path}")

    def on_timer(self):
        try:
            # 1. Base Frame'in World (odom) Frame'e göre konumu ve açısı
            t_base_world = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            bx = t_base_world.transform.translation.x
            by = t_base_world.transform.translation.y
            bz = t_base_world.transform.translation.z
            bq = t_base_world.transform.rotation
            br, bp, byaw = euler_from_quaternion(bq.x, bq.y, bq.z, bq.w)

            # 2. Uç noktanın (ur_tool0) Base Frame'e (base_link) göre konumu
            t_ee_base = self.tf_buffer.lookup_transform('base_link', 'ur_tool0', rclpy.time.Time())
            eex_b = t_ee_base.transform.translation.x
            eey_b = t_ee_base.transform.translation.y
            eez_b = t_ee_base.transform.translation.z

            # 3. Uç noktanın (ur_tool0) World (odom) Frame'e göre açısı
            t_ee_world = self.tf_buffer.lookup_transform('odom', 'ur_tool0', rclpy.time.Time())
            eeq = t_ee_world.transform.rotation
            eer, eep, eeyaw = euler_from_quaternion(eeq.x, eeq.y, eeq.z, eeq.w)

            # Veriyi CSV'ye kaydet
            current_time = time.time() - self.start_time
            self.csv_writer.writerow([
                current_time, 
                bx, by, bz, br, bp, byaw,
                eex_b, eey_b, eez_b,
                eer, eep, eeyaw
            ])
            
        except TransformException as ex:
            pass 

def main(args=None):
    rclpy.init(args=args)
    node = TransformLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.csv_file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()