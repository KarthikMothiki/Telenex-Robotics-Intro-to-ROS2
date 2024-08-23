This package contains the nodes to run
- pub-sub
- pub-sub with custom message
- service-client with custom srv

Open two terminals and source the same.
```
source /opt/ros/humble/setup.bash
```
Build your workspace and source it as well
```
colcon build
source install/setup.bash
```

### 1. Pub - Sub
In one terminal run the publisher node:
```
ros2 run my_pkg publisher
```
In another terminal run the subscriber node:
```
ros2 run my_pkg subscriber
```
