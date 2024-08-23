# ROS 2 Humble: Creating and Using Services

In this tutorial, we will create a simple ROS 2 service and client in Python using the ROS 2 Humble distribution. Services in ROS 2 allow for synchronous communication, where a node can request a task and wait for the result.

## Prerequisites

- ROS 2 Humble installedhttps://github.com/KarthikMothiki/Telenex-Robotics-Intro-to-ROS2/blob/main/src/custom_interfaces/ReadMe.md
- Basic understanding of ROS 2 concepts
- Python 3.8 or later

## Step 1: Create a New Package

First, create a new package for your service and client nodes.

```bash
cd ~/ros2_ws/src
ros2 pkg create --license Apache-2.0 --build-type ament_cmake custom_interfaces
```

## Step 2: Define the Service

Create a new directory for your service definitions.

```bash
mkdir -p ~/ros2_ws/src/custom_interfaces/srv
```
Inside this directory, create a new file named MyPkgSrv.srv with the following content:

```
bool req
---
builtin_interfaces/Time res
```

This service takes a bool as an input and returns the time. Check the following [link](https://docs.ros2.org/galactic/api/builtin_interfaces/msg/Time.html) for reference

## Step 3: Update Package Files
`CmakeLists.txt`
Update the setup.py file to include the service files.
```
cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyPkgMsg.msg"
  "srv/MyPkgSrv.srv"
  DEPENDENCIES std_msgs 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```
### Now update the following file
`package.xml`
Update the package.xml to include the build dependencies.
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="km@todo.todo">km</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend><depend>geometry_msgs</depend>
  
  <!-- Add these following lines-->
  <depend>std_msgs</depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <build_depend>builtin_interfaces</build_depend>
  <exec_depend>builtin_interfaces</exec_depend>


  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

## Step 4: Create the Service Node
Create a server.py file in the `src/my_pkg/my_pkg` directory.
```
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

```

## Step 5: Create the Client Node
Create a client.py file in the `src/my_pkg/my_pkg` directory.
```
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
```

## Step 6: Build the Package
Navigate to your workspace directory and build the package.
```
cd ~/ros2_ws
colcon build --packages-select my_pkg
```

Source the setup script to overlay your workspace on top of your environment.

```bash
source install/setup.bash
```

## Step 7: Run the Service and Client
In one terminal, start the service node:
```
ros2 run my_pkg server
```

In another terminal, start the client node with two integer arguments:

```
ros2 run my_pkg client
```
You should see the client node send a request to the service node and receive the time in seconds and nanoseconds format.
