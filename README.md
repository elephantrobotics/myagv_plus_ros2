# myagv_plus_ros2

ROS2 packages for myagv plus

> Software environment for Jetson Orin Nano

```
ubuntu 22.04
ros2 humble
```

# Installation

Create workspace and clone the repository.

```
git clone https://github.com/elephantrobotics/myagv_plus_ros2.git myagv_plus_ros2/src
```

Install dependencies

```
cd ~/myagv_plus_ros2

rosdep install --from-paths src --ignore-src -r -y
```

Build workspace

```
cd ~/myagv_plus_ros2

colcon build
```

Setup the workspace

```
source ~/myagv_plus_ros2/install/local_setup.bash
```

# Update to new version

```
cd ~/myagv_plus_ros2/src

git pull

cd ..

colcon build
```