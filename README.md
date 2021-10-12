# 👨‍💻 ros2_system_manager
ros2 system manager for your robot

Make available in docker shutdown and other system commands directly from ROS2 services

# Test manager
```
docker run -it --rm -v /run/ros2sm.sock:/run/ros2sm.sock rbonghi/ros2_system_manager:latest
```

# Develop

Install in dev mode the package

```
sudo -H pip3 install -v -e .
```

## Build ros2 package

```
colcon build --symlink-install
```

## Docker

You can test the docker image build the image

```
docker build -t rbonghi/ros2_system_manager:latest .
```