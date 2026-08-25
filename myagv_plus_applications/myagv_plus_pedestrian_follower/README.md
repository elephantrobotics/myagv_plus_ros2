# MYAGV Plus 行人跟随功能

## 一、如何启动

启动命令：

```
ros2 launch myagv_plus_pedestrian_follower pedestrian_follower.launch.py
```



## 二、依赖安装

1.安装mediapipe

```
python3.10 -m pip install --user "numpy<2" mediapipe==0.10.14
```

查看是否安装成功

```
python3.10 -c "import mediapipe; print(mediapipe.__version__)"
```

2.安装手势识别

```
mkdir -p ~/.mediapipe/models
cd ~/.mediapipe/models

wget https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

查看是否安装成功

```
ls -lh ~/.mediapipe/models
```

