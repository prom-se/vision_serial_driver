# vision_serial_driver
ROS2视觉串口驱动包

默认打开/dev/ttyACM0串口
## 用例

```
ros2 run vision_serial_driver vision_serial_driver_node <device_name>
```
## 步兵数据包定义
### 1.visionArray - TX
| Byte | Data |
| - | - |
| 0 | 0xA5 |
| 1-4 | aimYaw |
| 5-8 | aimPitch |
### 2.robotArray - RX
| Byte | Data |
| - | - |
| 0 | 0xA5 |
| 1 | Mode |
| 2 | foeColor |
| 3-4| Empty |
| 5-8 | robotYaw |
| 9-12 | robotPitch |
| 13-16 | muzzleSpeed |