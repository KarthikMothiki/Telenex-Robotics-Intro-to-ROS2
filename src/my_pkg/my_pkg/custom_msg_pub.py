import rclpy
from rclpy.node import Node
from custom_interfaces.msg import MyPkgMsg  

class CustomPublisher(Node):

    def __init__(self):
        super().__init__('custom_publisher')
        self.publisher_ = self.create_publisher(MyPkgMsg, 'custom_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Custom publisher has been started.')

    def timer_callback(self):
        msg = MyPkgMsg()
        msg.message = 'Publishing data using a custom msg'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.message}"')

def main(args=None):
    rclpy.init(args=args)
    custom_publisher = CustomPublisher()
    rclpy.spin(custom_publisher)
    custom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
