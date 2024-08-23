# Telenex-Robotics-Intro-to-ROS2

This repository contains the nodes used for **Telenex Robotics's Training Module**. 
Download the repo using the following command in your workspace directory:
```
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/KarthikMothiki/Telenex-Robotics-Intro-to-ROS2.git
```
Your src folder should contain these two ros2 packages (along with your other packages - in case you have other packages)
- custom_interfaces
- my_pkg
  
Now build your workspace and source the terminal
```
colcon build
source install/setup.bash
```

## Tutorials 
- Checkout the ros2 topics and service implementations [here](https://github.com/KarthikMothiki/Telenex-Robotics-Intro-to-ROS2/tree/main/src/my_pkg)
- Checkout the tutorial for custom message [here](https://github.com/KarthikMothiki/Telenex-Robotics-Intro-to-ROS2/tree/main/src/custom_interfaces/msg)
- Checkout the tutorial for custom service [here](https://github.com/KarthikMothiki/Telenex-Robotics-Intro-to-ROS2/tree/main/src/custom_interfaces/srv)
