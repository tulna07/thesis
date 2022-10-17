# Odometry information and image pixel converter

```
python ROS_odom_pixel_converter.py
python ROS_odom_pixel_converter.py -sm turtlebot3_world -ox -2 -oy -0.5 -px 180 -py 180
```
* sm: slam map, default = turtlebot3_world
* ox, oy: odometry pose of (x, y), which is get from /odom topic in ROS, type = float, default = -2.0, -0.5
* px, py: next point of (x, y), type = float, default = 200.0, 150.0