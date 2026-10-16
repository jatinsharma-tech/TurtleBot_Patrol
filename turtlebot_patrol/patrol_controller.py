import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray


def get_yaw(q):
    # quaternion -> yaw angle
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    # smallest difference between two angles, result in -pi..pi
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class PatrolController(Node):
    def __init__(self):
        super().__init__('patrol_controller')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('side_length', 1.5)
        self.declare_parameter('safe_distance', 0.5)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.side_length = self.get_parameter('side_length').value
        self.safe_distance = self.get_parameter('safe_distance').value

        # robot info
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_odom = False

        # lidar info
        self.front = 10.0
        self.left = 10.0
        self.right = 10.0

        self.estop = False

        # states: PATROL, AVOID, HALTED
        self.state = 'PATROL'
        # inside PATROL we either go straight or turn the 90 deg corner
        self.patrol_step = 'FORWARD'
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.turn_dir = 1.0  # used in AVOID, +1 = left, -1 = right

        self.create_subscription(Float32MultiArray, '/obstacle_status', self.obstacle_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/estop_state', self.estop_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Patrol controller started')

    def obstacle_callback(self, msg):
        if len(msg.data) >= 3:
            self.front = msg.data[0]
            self.left = msg.data[1]
            self.right = msg.data[2]

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = get_yaw(msg.pose.pose.orientation)
        if not self.got_odom:
            self.got_odom = True
            self.start_new_leg()

    def estop_callback(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn('E-STOP ACTIVE -> HALTED')
            self.estop = True
            self.change_state('HALTED')
            self.stop_robot()  # stop right away, don't wait for the timer
        elif not msg.data and self.estop:
            self.get_logger().info('E-stop cleared -> back to PATROL')
            self.estop = False
            self.start_new_leg()
            self.change_state('PATROL')

    def change_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'State: {self.state} -> {new_state}')
            self.state = new_state

    def start_new_leg(self):
        # remember where this side of the square started
        self.patrol_step = 'FORWARD'
        self.start_x = self.x
        self.start_y = self.y

    def stop_robot(self):
        self.cmd_pub.publish(Twist())  # all zeros

    def control_loop(self):
        cmd = Twist()

        if self.estop or self.state == 'HALTED':
            self.stop_robot()
            return

        if not self.got_odom:
            return

        if self.state == 'PATROL':
            if self.front < self.safe_distance:
                # obstacle ahead: stop and pick the side with more space
                self.stop_robot()
                self.turn_dir = 1.0 if self.left > self.right else -1.0
                side = 'left' if self.turn_dir > 0 else 'right'
                self.get_logger().info(f'Obstacle at {self.front:.2f} m, turning {side}')
                self.change_state('AVOID')
                return

            if self.patrol_step == 'FORWARD':
                dist = math.hypot(self.x - self.start_x, self.y - self.start_y)
                if dist >= self.side_length:
                    self.patrol_step = 'TURN'
                    self.start_yaw = self.yaw
                else:
                    cmd.linear.x = self.linear_speed

            elif self.patrol_step == 'TURN':
                turned = abs(angle_diff(self.yaw, self.start_yaw))
                if turned >= math.pi / 2:
                    self.start_new_leg()
                else:
                    cmd.angular.z = self.angular_speed

        elif self.state == 'AVOID':
            # keep rotating until the front is clear (with a small margin)
            if self.front > self.safe_distance + 0.2:
                self.start_new_leg()
                self.change_state('PATROL')
            else:
                cmd.angular.z = self.turn_dir * self.angular_speed

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PatrolController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
