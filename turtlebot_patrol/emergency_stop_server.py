import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class EmergencyStopServer(Node):
    def __init__(self):
        super().__init__('emergency_stop_server')
        self.estop_active = False

        self.state_pub = self.create_publisher(Bool, '/estop_state', 10)
        self.srv = self.create_service(SetBool, '/trigger_estop', self.handle_request)

        # keep publishing the state so new subscribers get it too
        self.timer = self.create_timer(0.2, self.publish_state)
        self.get_logger().info('E-stop server ready on /trigger_estop')

    def handle_request(self, request, response):
        self.estop_active = request.data
        self.publish_state()  # publish immediately, don't wait for the timer

        if self.estop_active:
            response.message = 'Emergency stop ACTIVATED'
            self.get_logger().warn(response.message)
        else:
            response.message = 'Emergency stop cleared'
            self.get_logger().info(response.message)

        response.success = True
        return response

    def publish_state(self):
        msg = Bool()
        msg.data = self.estop_active
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
