# ros_client.py (AGVIOClient) 使用说明

> 经 `/query_device` 服务下发底部协议(esp32_node 为服务端)。
> 写类命令成功返回 `int` 1、失败返回 0;读类命令失败统一返回 `None`。

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

## API 函数说明

### 1. 系统 & 产品信息

#### get_system_version()

- **cmd_id:** `0x02`
- **功能:** 获取主固件版本号(原始值已 /10)
- **返回值:** `float` (版本号)

#### get_modify_version()

- **cmd_id:** `0x01`
- **功能:** 获取次固件版本号(原始值已 /10),用于区分测试迭代
- **返回值:** `float` (版本号)

#### get_robot_status()

- **cmd_id:** `0x05`
- **功能:** 读取硬件异常状态
- **返回值:** `list[int]` `[电池状态, 陀螺仪状态, 电量水平]`
  - 电池状态: 0-正常, 1-异常
  - 陀螺仪状态: 0-正常, 1-异常
  - 电量水平: 0-正常, 1-警告(≤19.6V), 2-低电量(<19V)

### 2. 电机供电

#### power_on()

- **cmd_id:** `0x10` (state=1)
- **功能:** 电机上电
- **返回值:** `int` (1: 成功, 0: 失败)

#### power_off()

- **cmd_id:** `0x10` (state=0)
- **功能:** 电机断电
- **返回值:** `int` (1: 成功, 0: 失败)

#### is_power_on()

- **cmd_id:** `0x12`
- **功能:** 检查电机是否已上电
- **返回值:** `int` (1: 开机, 0: 关机)

### 3. 自动上发

#### set_auto_report_state(state)

- **cmd_id:** `0x23`
- **功能:** 设置底层数据的自动上发状态(开启后底板每 50ms 主动上报电池/陀螺仪)
- **参数:**
  - `state` (int): 0-关闭, 1-开启
- **返回值:** `int` (1: 成功, 0: 失败)

#### get_auto_report_state()

- **cmd_id:** `0x24`
- **功能:** 获取当前自动上发状态
- **返回值:** `int` (0: 关闭, 1: 开启)

### 4. LED 灯带

#### set_led_color(brightness, color)

- **cmd_id:** `0x34`
- **功能:** 设置灯带颜色(需先 `set_led_mode(1)` 切到 DIY 模式)
- **参数:**
  - `brightness` (int): 亮度 0-255
  - `color` (tuple): 颜色 `(R, G, B)`,各 0-255
- **返回值:** `int` (1: 成功, 0: 失败)

#### set_led_mode(mode)

- **cmd_id:** `0x3A`
- **功能:** 设置灯带模式
- **参数:**
  - `mode` (int): 0-电量显示(默认), 1-DIY 自定义
- **返回值:** `int` (1: 成功, 0: 失败)

### 5. IO 控制 & 风扇

#### set_pin_output(pin, state)

- **cmd_id:** `0x40`
- **功能:** 设置输出引脚电平(pin 1-6 对应 base 板丝印 24/22/23/27/18/17)
- **参数:**
  - `pin` (int): 1-6,0 表示全部
  - `state` (int): 1-高电平, 0-低电平
- **返回值:** `int` (1: 成功, 0: 失败)

#### get_pin_input(pin)

- **cmd_id:** `0x41`
- **功能:** 读取输入引脚电平(pin 1-6 对应 base 板丝印 7/11/8/9/25/10)
- **参数:**
  - `pin` (int): 1-6,0 表示一次读全部
- **返回值:** 单引脚 `int` (0/1;无效引脚或失败返回 -1);`pin=0` 时返回 `list[int]`(1-6 号引脚状态)

#### set_fan_state(state)

- **cmd_id:** `0x42`
- **功能:** 设置风扇开关(默认开)
- **参数:**
  - `state` (int): 1-开, 0-关
- **返回值:** `int` (1: 成功, 0: 失败)

### 6. 吸泵

#### set_pump_state(state)

- **cmd_id:** `0x43`
- **功能:** 吸泵开关(固件内部处理破真空)
- **参数:**
  - `state` (int): 1-开, 0-关
- **返回值:** `int` (1: 成功, 0: 失败)

#### set_pump_io(pin, state)

- **cmd_id:** `0x44`
- **功能:** 直接控制吸泵 IO(最新吸泵 5 号脚低电平工作、高电平关闭)
- **参数:**
  - `pin` (int): 2 或 5(对应吸泵 2、5)
  - `state` (int): 0-低电平, 1-高电平
- **返回值:** `int` (1: 成功, 0: 失败)
