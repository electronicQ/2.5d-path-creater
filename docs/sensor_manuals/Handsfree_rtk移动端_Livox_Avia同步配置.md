# Handsfree RTK 移动端 Livox Avia 同步配置

用途：把 Handsfree RTK 配成流动站，并给 Livox Avia 同步链路提供 `1PPS + GPRMC` 时间源。

说明：这套配置以 `Handsfree_rtk移动端出厂配置.md` 为基础，重新配置 COM1、COM2、COM3 的串口和输出项，并新增 PPS 输出配置。COM1 对应 Type-C/USB 口，波特率按 `115200` 配置，输出 1Hz `GPRMC` 给电脑，由 Livox SDK/ROS driver 解析后将 UTC 时间发送给 Avia。GH1.25 7Pin 接口对应 COM2，其中 `PPS` 直接接 Avia 同步输入；COM2 的串口 TX 输出保留原厂输出内容，但不作为 Avia 的 UTC 时间来源。

## 推荐串口发送配置

逐行发送下面命令，发送完成后断电重启 RTK，再检查输出。

```text
configcom1115200
configcom2115200
configcom3115200
unlogcom1
unlogcom2
unlogcom3
moderover
CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0
saveconfig
gpggacom10.1
gprmccom11
gpvtgcom10.1
gpthscom10.1
agricacom10.1
gpgsacom10.1
gpgstcom10.1
gpzdacom10.1
uniheadingacom10.1
saveconfig
gpggacom20.1
gprmccom20.1
gpvtgcom20.1
gpthscom20.1
agricacom20.1
gpgsacom20.1
gpgstcom20.1
gpzdacom20.1
uniheadingacom20.1
saveconfig
gpggacom30.5
gprmccom30.5
gpvtgcom30.5
gpthscom30.5
agricacom30.5
gpgsacom30.5
gpgstcom30.5
gpzdacom30.5
uniheadingacom30.5
saveconfig
```

## 标准空格格式

如果串口工具或固件不接受简化写法，使用下面的标准空格格式逐行发送。

```text
CONFIG COM1 115200
CONFIG COM2 115200
CONFIG COM3 115200
UNLOG COM1
UNLOG COM2
UNLOG COM3
MODE ROVER
CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0
SAVECONFIG
#设置TypeC接口输出
GPGGA COM1 0.1
GPRMC COM1 1
GPVTG COM1 0.1
GPTHS COM1 0.1
AGRICA COM1 0.1
GPGSA COM1 0.1
GPGST COM1 0.1
GPZDA COM1 0.1
UNIHEADINGA COM1 0.1
SAVECONFIG
#设置GH1.257pin接口输出
GPGGA COM2 0.1
GPRMC COM2 0.1
GPVTG COM2 0.1
GPTHS COM2 0.1
AGRICA COM2 0.1
GPGSA COM2 0.1
GPGST COM2 0.1
GPZDA COM2 0.1
UNIHEADINGA COM2 0.1
SAVECONFIG
#设置GH1.254pin和GH1.255pin接口输出
GPGGA COM3 0.5
GPRMC COM3 0.5
GPVTG COM3 0.5
GPTHS COM3 0.5
AGRICA COM3 0.5
GPGSA COM3 0.5
GPGST COM3 0.5
GPZDA COM3 0.5
UNIHEADINGA COM3 0.5
SAVECONFIG
```

## 与 Avia 接线

COM1 的 Type-C/USB 口接电脑，电脑侧串口波特率设为 `115200`，供 Livox SDK/ROS driver 读取 1Hz `GPRMC`。

GH1.25 7Pin 从左向右为 `SDA、SCL、PPS、RX、TX、GND、5V`。

| RTK GH1.25 7Pin | 连接对象 |
| --- | --- |
| PPS | Livox Avia 同步线蓝色 `PPS/Sync+` |
| GND | Livox Avia 同步线黑色 `GND`，并与上位机串口地共地 |
| TX | 不作为 Avia 同步使用；`GPRMC` 从 COM1 的 Type-C/USB 口给电脑 |
| 5V | 不接 Avia 同步口 |

不要把 RTK 的 `TX` 直接接到 Avia 同步线白色线。Avia 转接盒同步口是 `GND/PPS/NC`，白色线为未定义信号；UTC 时间应由电脑通过 COM1/USB 读取 `GPRMC` 后，经 Livox SDK/ROS driver 发给 Avia。

## 配置含义

- `MODE ROVER`：配置为流动站。
- `CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0`：启用 GPS 时基 PPS，上升沿有效，脉宽 100 ms，周期 1000 ms。
- `GPRMC COM1 1`：Type-C/USB 口输出 1Hz `GPRMC`，用于给电脑上的 Livox 驱动提供 UTC 时间。
- COM2 和 COM3 输出内容保持原出厂配置，方便原有 GH1.25 7Pin、4Pin、5Pin 使用方式继续工作；Avia 同步只使用 COM2 对应接口上的 PPS 引脚，PPS 不受 COM2 串口波特率影响。

## Livox ROS driver 配置

如果 RTK 的 Type-C/USB 口在系统里是 `/dev/HFRobotRTK`，并且波特率是 `115200`，把 `src/sensors/livox_ros_driver/livox_ros_driver/config/livox_lidar_config.json` 里的 `timesync_config` 改成：

```json
"timesync_config": {
    "enable_timesync": true,
    "device_name": "/dev/HFRobotRTK",
    "comm_device_type": 0,
    "baudrate_index": 6,
    "parity_index": 0
}
```

其中 `baudrate_index: 6` 对应 `115200`。如果实际设备名是 `/dev/ttyUSB0`、`/dev/ttyACM0` 或其他 udev 软链接，只改 `device_name`，不要改成和实际设备不一致的名字。

## 配置后检查

1. 用电脑连接 RTK 的 Type-C/USB 口，波特率 `115200`，确认每秒能看到 1 条 `GPRMC`。
2. 用示波器或逻辑分析仪测 GH1.25 7Pin 的 PPS 与 GND，确认周期约 1000 ms，正脉宽约 100 ms。
3. 确认 PPS 高电平不超过 Avia 同步口可接受的 3.3V LVTTL 电平。如果实际是 5V，需要加电平转换。
4. 在 Livox Viewer 或 Livox SDK/ROS driver 中检查 Avia 的 PPS/UTC 同步状态。
