# Bobik the Robot - High Level package

# Installation

## .bashrc
Bacause of [bug in rviz to display cylinders](https://answers.ros.org/question/389967/urdf-and-rviz2-cylinder-not-showing/)
```
echo 'export LC_NUMERIC="en_US.UTF-8"' >>~/.bashrc
```
# Startup
```
clear && ros2 launch bobik_robot bobik_robot.launch.py
```

# Teleop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
