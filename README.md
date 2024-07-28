# vision_serial_driver
NEXT E视觉组串口驱动
## 数据包结构
### TX - vision_pack
| Byte | Data |
| - | - |
| 0 | 0xA5 |
| 1 | fire |
| 2-5 | aimYaw |
| 6-9 | aimPitch |

### RX - robot_pack
| Byte | Data |
| - | - |
| 0 | 0xA5 |
| 1 | foeColor |
| 2-5 | robotYaw |
| 6-9 | robotPitch |
| 10-13 | muzzleSpeed |