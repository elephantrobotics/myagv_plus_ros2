# myagv_plus_ros2

ROS2 packages for myagv plus

<img width="1536" height="1024" alt="01dd6f3f-7aaf-48b1-be11-5e857bd7050c" src="https://github.com/user-attachments/assets/ecabaa9a-ec51-460f-b1ed-ea66060729a7" />

<img width="1313" height="1053" alt="9541674ca549e64a6909dbd0df34105e" src="https://github.com/user-attachments/assets/a13a8a42-bbfd-4952-82e4-0eed1acca0bb" />


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
