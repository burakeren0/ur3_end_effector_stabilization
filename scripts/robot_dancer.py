import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class RobotDancer(Node):
    def __init__(self):
        super().__init__('robot_dancer')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.joint_names = [
            'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
            'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint'
        ]
        self.get_logger().info("Bağlantı kuruluyor... 2 saniye sonra 15 saniyelik dans rotası gönderilecek.")
        self.timer = self.create_timer(2.0, self.send_trajectory)
        self.sent = False

    def send_trajectory(self):
        if self.sent:
            return
        self.sent = True
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        # 15 saniyelik rota oluştur, her 0.1 saniyede bir nokta
        duration = 15.0
        dt = 0.1
        points_count = int(duration / dt)

        for i in range(points_count):
            t = i * dt
            point = JointTrajectoryPoint()

            # Robotun başlangıç pozisyonundan sıçrama yapmaması için formüller başlangıç açılarına eklendi:
            # Başlangıç: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            p1 = 0.0   + 0.5 * math.sin(t)
            p2 = -1.57 + 0.3 * math.sin(t * 0.8)
            p3 = 0.0   + 0.5 * math.sin(t * 1.2)
            p4 = -1.57 + 0.4 * math.sin(t * 0.9)
            p5 = 0.0   + 0.5 * math.sin(t * 1.1)
            p6 = 0.0   + 0.5 * math.sin(t * 1.5)

            point.positions = [p1, p2, p3, p4, p5, p6]

            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec

            msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info("Harika! Rota başarıyla gönderildi. Robot şu an hareket ediyor olmalı.")
        self.get_logger().info("Lütfen veri kaydedici (logger) terminalinizi takip edin.")

def main(args=None):
    rclpy.init(args=args)
    node = RobotDancer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
