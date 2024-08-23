import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MyPkgSrv  

class TimeClient(Node):

    def __init__(self):
        super().__init__('demo_client')
        self.client = self.create_client(MyPkgSrv, 'get_time')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.get_logger().info('Service is available.')
        self.request = MyPkgSrv.Request()

    def send_request(self):
        self.request.req = True
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    time_client = TimeClient()
    response = time_client.send_request()
    if response:
        time = response.res
        print(f'Received time: {time.sec} seconds and {time.nanosec} nanoseconds')
    else:
        print('Service call failed.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
