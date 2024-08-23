# ROS 2 Humble: Creating and Using Custom messages

This tutorial will create a simple ROS 2 publisher and subscriber using a custom message in Python using the ROS 2 Humble distribution. 
## Prerequisites

- ROS 2 Humble installed
- Python 3.8 or later


### NOTE
- Skip steps 1 & 2 if you already created the custom_interfaces package and don't forget to change the message definition.
- Skip the steps 1 through 5 if you installed the package using git


## Step 1: Create a New Package

First, create a new package for your service and client nodes.

```bash
cd ~/ros2_ws/src
ros2 pkg create --license Apache-2.0 --build-type ament_cmake custom_interfaces
```

## Step 2: Define the Message

Create a new directory for your custom message definitions.

```bash
mkdir -p ~/ros2_ws/src/custom_interfaces/msg
```
Inside this directory, create a new file named MyPkgMsg.msg with the following content:

```
string message
```

This message contains a singular string message (You can further add more data types as you need). Check the following [link](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html) for reference

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

## Step 4: Create the Publisher Node
Create a custom_msg_pub.py file in the `src/my_pkg/my_pkg` directory.
```
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

```

## Step 5: Create the Subscriber Node
Create a custom_msg_sub.py file in the `src/my_pkg/my_pkg` directory.
```
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
ros2 run my_pkg custom_msg_pub
```

In another terminal, start the client node with two integer arguments:

```
ros2 run my_pkg custom_msg_sub
```

