# ros_client.py (AGVIOClient) 调用示例

## 环境配置(首次)

```bash
bash /home/elephant/myagv_plus_ros2/src/myagv_plus_msgs/scripts/setup_pythonpath.sh
source ~/.bashrc
python3 -c "from ros_client import AGVIOClient; print('ok')"   # 输出 ok 即配置成功
```

## 调用示例

```python
import rclpy
from ros_client import AGVIOClient

rclpy.init()
agv = AGVIOClient()                       # 自动等待 /query_device 服务

agv.set_led_mode(1)                       # 切 DIY 模式
agv.set_led_color(100, (255, 0, 0))       # 红色,亮度100
print(agv.get_system_version())           # -> 1.0
print(agv.get_robot_status())             # -> [0, 0, 0]  [电池, 陀螺仪, 电量]
agv.power_on()                            # 开机(电机供电)
agv.set_fan_state(1)                       # 开风扇

rclpy.shutdown()
```
