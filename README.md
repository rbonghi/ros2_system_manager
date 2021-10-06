# 👨‍💻 ros_system_manager
ros2 system manager for your robot

Make available in docker shutdown and other system commands directly from ROS2 services


docker build -t ros_system_manager .

docker run -it --rm -v /run/ros_system_manager.sock:/run/ros_system_manager.sock ros_system_manager:latest