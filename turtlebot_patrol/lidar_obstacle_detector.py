import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')
        self.declare_parameter('safe_distance', 0.5)
        self.safe_distance = self.get_parameter('safe_distance').value

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # message layout: [front, left, right, obstacle_in_front (1 or 0)]
        self.status_pub = self.create_publisher(Float32MultiArray, '/obstacle_status', 10)
        self.get_logger().info('Lidar obstacle detector started')

    def scan_callback(self, scan):
        front = []
        left = []
        right = []

        for i, r in enumerate(scan.ranges):
            # skip bad readings
            if math.isinf(r) or math.isnan(r) or r < scan.range_min or r > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            angle = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))  # -180 to 180

            if -30 <= angle <= 30:
                front.append(r)
            elif 30 < angle <= 90:
                left.append(r)
            elif -90 <= angle < -30:
                right.append(r)

        # if a sector has no valid readings, treat it as clear
        front_min = min(front) if front else scan.range_max
        left_min = min(left) if left else scan.range_max
        right_min = min(right) if right else scan.range_max

        obstacle = 1.0 if front_min < self.safe_distance else 0.0

        msg = Float32MultiArray()
        msg.data = [float(front_min), float(left_min), float(right_min), obstacle]
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
