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
The result should look like this
![Screenshot from 2024-08-24 01-49-27](https://github.com/user-attachments/assets/abf8401e-7767-4c33-87b8-93b8a06b80f6)

### 2. Pub - Sub with Custom Message
In one terminal run the publisher node:
```
ros2 run my_pkg custom_pub
```
In another terminal run the subscriber node:
```
ros2 run my_pkg custom_sub
```
The result should look like this
![Screenshot from 2024-08-24 01-53-21](https://github.com/user-attachments/assets/8f630e89-42ca-4b47-993e-914377bf2a3f)


Now publishing custom data to your topic via your custom message
Run the following command in your terminal:
```
ros2 topic pub /custom_topic custom_interfaces/msg/MyPkgMsg "{message: 'Karthik is sending this message using custom message'}"
```
![Screenshot from 2024-08-24 01-57-42](https://github.com/user-attachments/assets/951e7833-4238-4384-92b1-cc1799a3fa0f)

### 3. Service - Client using Custom srv
In one terminal run the publisher node:
```
ros2 run my_pkg server
```
In another terminal run the subscriber node:
```
ros2 run my_pkg client
```
As the request parameter is set to true by default a response is given by the service. 

We can call the service with various parameters from the CLI too:
```
ros2 service call /get_time custom_interfaces/srv/MyPkgSrv req:\ true\ 
```
Returns a response with time
```
ros2 service call /get_time custom_interfaces/srv/MyPkgSrv req:\ false\ 
```
Returns a response with time 0 seconds and 0 nanoseconds
![Screenshot from 2024-08-24 02-03-28](https://github.com/user-attachments/assets/e289ed1d-c704-43cc-9ba7-b9143bedbdf3)
