# myagv_plus_ros2

ROS2 packages for myagv plus

<img width="1536" height="1024" alt="01dd6f3f-7aaf-48b1-be11-5e857bd7050c" src="https://github.com/user-attachments/assets/ecabaa9a-ec51-460f-b1ed-ea66060729a7" />

<img width="1313" height="1053" alt="9541674ca549e64a6909dbd0df34105e" src="https://github.com/user-attachments/assets/a13a8a42-bbfd-4952-82e4-0eed1acca0bb" />


> Software environment for Jetson Orin Nano

```
ubuntu 22.04
ros2 humble
```

<img width="1195" height="408" alt="d3db3801-7463-4b94-9617-36cd2cc4a553" src="https://github.com/user-attachments/assets/c7abae79-c9b5-4a5f-8767-0328f1dbba8e" />

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

