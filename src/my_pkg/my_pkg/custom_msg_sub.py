import rclpy
from rclpy.node import Node
from custom_interfaces.msg import MyPkgMsg  

class CustomSubscriber(Node):

    def __init__(self):
        super().__init__('custom_subscriber')
        self.subscription = self.create_subscription(
            MyPkgMsg,
            'custom_topic',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Custom subscriber has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.message}"')

def main(args=None):
    rclpy.init(args=args)
    custom_subscriber = CustomSubscriber()
    rclpy.spin(custom_subscriber)
    custom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
