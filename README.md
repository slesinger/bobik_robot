# Bobik the Robot - High Level package

# Installation

## .bashrc
Bacause of [bug in rviz to display cylinders](https://answers.ros.org/question/389967/urdf-and-rviz2-cylinder-not-showing/)
```
echo 'export LC_NUMERIC="en_US.UTF-8"' >>~/.bashrc
```

# CAD

## Mesh edits
Use Meshalab to reduce number of triangles. Filters > Remeshing, Simplification and Reconstruction > Simplification: Quadratic Edge Collapse Decimation.

No idea how to export textures with Collada dae files. Alternative is set URDF ```<material texture="medvidek.jpg" />``` in ```<visual>```. No UV mapping is possible, though.

# Startup
```
clear && ros2 launch bobik_robot bobik_robot.launch.py
```

# Teleop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
