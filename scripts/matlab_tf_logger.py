#!/usr/bin/env python3
import argparse
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def degrees(rad):
    return math.degrees(rad)


class MatlabTFLogger(Node):
    def __init__(self, args):
        super().__init__('matlab_tf_logger')

        self.world_frame = args.world_frame
        self.husky_frame = args.husky_frame
        self.ur_base_frame = args.ur_base_frame
        self.ee_frame = args.ee_frame
        self.imu_topic = args.imu_topic
        self.sample_period = 1.0 / args.sample_rate

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.imu_orientation = None
        self.imu_received = False
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)

        self.tf_ready = False
        self.start_time = time.time()
        self.wait_start = time.time()

        output_dir = os.path.join(os.path.dirname(__file__), '..', 'analysis')
        os.makedirs(output_dir, exist_ok=True)
        file_path = os.path.join(output_dir, args.output_file)

        self.csv_file = open(file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'husky_x_world', 'husky_y_world', 'husky_z_world',
            'husky_roll_world_deg', 'husky_pitch_world_deg', 'husky_yaw_world_deg',
            'ee_x_ur_base', 'ee_y_ur_base', 'ee_z_ur_base',
            'ee_roll_world_deg', 'ee_pitch_world_deg', 'ee_yaw_world_deg'
        ])

        self.timer = self.create_timer(self.sample_period, self.on_timer)
        self.get_logger().info(f'CSV verisi yazılıyor: {file_path}')
        self.get_logger().info(
            f'World frame="{self.world_frame}", Husky frame="{self.husky_frame}", '
            f'UR base frame="{self.ur_base_frame}", EE frame="{self.ee_frame}", IMU topic="{self.imu_topic}"'
        )

    def imu_callback(self, msg: Imu):
        self.imu_orientation = msg.orientation
        self.imu_received = True

    def on_timer(self):
        if not self.tf_ready:
            if self.tf_buffer.can_transform(self.world_frame, self.husky_frame, rclpy.time.Time()) and \
               self.tf_buffer.can_transform(self.ur_base_frame, self.ee_frame, rclpy.time.Time()) and \
               self.tf_buffer.can_transform(self.world_frame, self.ee_frame, rclpy.time.Time()):
                self.tf_ready = True
                self.get_logger().info('TF hazır: veri kaydına başlanıyor.')
            else:
                if time.time() - self.wait_start > 5.0:
                    self.get_logger().warning('TF henüz hazır değil. Bekleniyor...')
                return

        try:
            t_base_world = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.husky_frame,
                rclpy.time.Time()
            )
            bx = t_base_world.transform.translation.x
            by = t_base_world.transform.translation.y
            bz = t_base_world.transform.translation.z
            if self.imu_received:
                bq = self.imu_orientation
                brow, bpitch, byaw = euler_from_quaternion(bq.x, bq.y, bq.z, bq.w)
            else:
                bq = t_base_world.transform.rotation
                brow, bpitch, byaw = euler_from_quaternion(bq.x, bq.y, bq.z, bq.w)

            t_ee_base = self.tf_buffer.lookup_transform(
                self.ur_base_frame,
                self.ee_frame,
                rclpy.time.Time()
            )
            eex = t_ee_base.transform.translation.x
            eey = t_ee_base.transform.translation.y
            eez = t_ee_base.transform.translation.z

            t_ee_world = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.ee_frame,
                rclpy.time.Time()
            )
            eeq = t_ee_world.transform.rotation
            eeroll, eepitch, eeyaw = euler_from_quaternion(eeq.x, eeq.y, eeq.z, eeq.w)

            self.csv_writer.writerow([
                time.time() - self.start_time,
                bx, by, bz,
                degrees(brow), degrees(bpitch), degrees(byaw),
                eex, eey, eez,
                degrees(eeroll), degrees(eepitch), degrees(eeyaw)
            ])

        except TransformException as exc:
            self.get_logger().warning(f'TF hatası: {exc}')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description='Matlab için Husky+UR3 TF verisi kaydedici'
    )
    parser.add_argument('--world-frame', default='odom', help='World/odom frame adı')
    parser.add_argument('--husky-frame', default='base_link', help='Husky taban frame adı')
    parser.add_argument('--ur-base-frame', default='ur_base_link_inertia', help='UR3 taban frame adı')
    parser.add_argument('--ee-frame', default='ur_tool0', help='UR3 uç nokta frame adı')
    parser.add_argument('--imu-topic', default='/imu_data', help='Husky/UR3 base IMU topic adı')
    parser.add_argument('--output-file', default='matlab_tf_data.csv', help='Çıktı CSV dosyası adı')
    parser.add_argument('--sample-rate', type=float, default=20.0, help='Örnekleme frekansı (Hz)')
    args = parser.parse_args()

    rclpy.init()
    node = MatlabTFLogger(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
