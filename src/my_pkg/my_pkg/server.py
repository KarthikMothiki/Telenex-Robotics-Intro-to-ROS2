import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MyPkgSrv  
from builtin_interfaces.msg import Time

class TimeService(Node):

    def __init__(self):
        super().__init__('demo_service')
        self.srv = self.create_service(MyPkgSrv, 'get_time', self.get_time_callback)
        self.get_logger().info('Time service has been started.')

    def get_time_callback(self, request, response):
        self.get_logger().info(f'Received request: {request.req}')
        if request.req:
            current_time = self.get_clock().now().to_msg()
            response.res = current_time
        else:
            response.res = Time()  # Send zero time as a response if req is False
        return response

def main(args=None):
    rclpy.init(args=args)
    time_service = TimeService()
    rclpy.spin(time_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
