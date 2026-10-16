import csv
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class TelemetryLogger(Node):
    def __init__(self):
        super().__init__('telemetry_logger')
        self.declare_parameter('battery_start', 100.0)
        self.declare_parameter('drain_per_10s', 1.0)
        self.declare_parameter('log_file', 'telemetry_log.csv')
        self.declare_parameter('log_period', 2.0)

        self.battery = self.get_parameter('battery_start').value
        self.drain_per_10s = self.get_parameter('drain_per_10s').value
        self.log_file = self.get_parameter('log_file').value
        log_period = self.get_parameter('log_period').value

        self.total_distance = 0.0
        self.moving_time = 0.0
        self.last_x = None
        self.last_y = None
        self.is_moving = False
        self.start_time = time.time()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.telemetry_pub = self.create_publisher(String, '/robot_telemetry', 10)

        # write the csv header (overwrites old log)
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time_s', 'x', 'y', 'distance_m', 'avg_speed_mps', 'battery_pct'])

        self.create_timer(1.0, self.update)
        self.create_timer(log_period, self.write_csv)
        self.get_logger().info(f'Telemetry logger started, writing to {self.log_file}')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_x is not None:
            step = math.hypot(x - self.last_x, y - self.last_y)
            if step < 1.0:  # ignore weird jumps (e.g. sim reset)
                self.total_distance += step
        self.last_x = x
        self.last_y = y

    def cmd_callback(self, msg):
        self.is_moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

    def average_speed(self):
        if self.moving_time == 0:
            return 0.0
        return self.total_distance / self.moving_time

    def update(self):
        # called every 1 second
        if self.is_moving:
            self.moving_time += 1.0
            self.battery -= self.drain_per_10s / 10.0
            self.battery = max(self.battery, 0.0)

        text = (f'distance={self.total_distance:.2f} m | '
                f'avg_speed={self.average_speed():.3f} m/s | '
                f'battery={self.battery:.1f} %')
        self.get_logger().info(text)

        msg = String()
        msg.data = text
        self.telemetry_pub.publish(msg)

    def write_csv(self):
        x = self.last_x if self.last_x is not None else 0.0
        y = self.last_y if self.last_y is not None else 0.0
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                round(time.time() - self.start_time, 1),
                round(x, 3), round(y, 3),
                round(self.total_distance, 3),
                round(self.average_speed(), 3),
                round(self.battery, 1),
            ])


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
