#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class MasterMover(Node):
    def __init__(self):
        super().__init__('master_mover_node')
        
        # Husky Publisher (TwistStamped)
        self.husky_pub = self.create_publisher(
            TwistStamped, 
            '/diff_drive_base_controller/cmd_vel', 
            10
        )
        
        # UR3 Publisher
        self.ur3_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(1.0, self.execute_mission)
        self.get_logger().info("Mobil Manipülatör Görevi Başladı!")

    def execute_mission(self):
        # 1. Husky'yi hareket ettir
        husky_msg = TwistStamped()
        husky_msg.header.stamp = self.get_clock().now().to_msg()
        husky_msg.header.frame_id = "base_link"
        husky_msg.twist.linear.x = 0.2  # 0.2 m/s hız
        self.husky_pub.publish(husky_msg)

        # 2. UR3 Kolunu hareket ettir (Home Pozisyonu)
        ur3_msg = JointTrajectory()
        ur3_msg.joint_names = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0] # Dik duruş [cite: 1069]
        point.time_from_start = Duration(sec=2)
        ur3_msg.points.append(point)
        self.ur3_pub.publish(ur3_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MasterMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()