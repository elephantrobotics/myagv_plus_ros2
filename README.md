# myagv_plus_ros2

ROS2 packages for myagv plus

<img width="1536" height="1024" alt="01dd6f3f-7aaf-48b1-be11-5e857bd7050c" src="https://github.com/user-attachments/assets/ecabaa9a-ec51-460f-b1ed-ea66060729a7" />

> Software environment for Jetson Orin Nano

```
ubuntu 24.04
ros2 jazzy
jetpack 7.2
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

```bash
cd ~/myagv_plus_ros2/src
git pull
cd ..
colcon build
```

When compiling large packages such as `rtabmap` or `nav2` on Jetson Orin Nano, it is recommended to limit the build concurrency:

```bash
cd ~/myagv_plus_ros2/src
git pull
cd ..
export MAKEFLAGS="-j5"
colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

