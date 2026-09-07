# C2 + Livox Avia 时间同步与 ROS 驱动方案

## 1. 目标

基于现有硬件：
- 北云 C2-M20&M22&M22 高精度板卡
- Livox Avia 激光雷达
- 仅使用 C2 板卡 28 针引脚

实现两件事：
1. C2 通过板卡的usb口输出 NMEA 语句，由电脑端 ROS 节点解析并发布 `sensor_msgs/NavSatFix`
2. C2 输出 PPS 与 UTC 时间，驱动 Livox Avia 实现绝对 UTC 时间同步

## 2. 结论

该方案可行，推荐采用“两路串口 + 一路 PPS”的结构：

- `C2的COM3`：输出 `GPGGA + GPRMC`，供 GNSS ROS 驱动解析成 `NavSatFix`
- `C2的COM4`：只输出 `GPRMC`，供 `livox_ros_driver` 做 UTC 时间同步
- `PPS`：输出到 Livox Avia 同步口
- 所有地线共地，串口电平按 3.3V LVTTL 处理

说明：
- `GPRMC` 提供完整 UTC 日期时间，适合生成 ROS 时间戳
- `GPGGA` 提供经纬高和定位质量，适合填充 `NavSatFix`
- Livox 时间同步不是读取 `NavSatFix`，而是直接读取串口上的 `GPRMC/GNRMC`

## 3. 硬件连接

### 3.1 C2 侧引脚

C2 板卡用到的引脚包括：
- `usb口`： C2的COM3端口对应的串口转usb输入输出
- `13号引脚`：C2的COM4端口输出对应的串口输出引脚TXD4
- `23号引脚` ： PPS输出引脚
- `GND`

注意：
- 串口电平为 3.3V LVTTL
- 电脑侧需使用 3.3V USB-TTL 转换器
- 不要直接接 RS-232 电平
- 所有设备必须共地

### 3.2 推荐接线

#### 路线 A：GNSS 驱动
- C2 `COM3 TXD2` -> USB-TTL -> 电脑
- C2 `COM3 RXD2` -> USB-TTL -> 电脑
- C2 `GND` -> 电脑侧地

#### 路线 B：Livox 时间同步
- C2 `COM4 TXD2` -> USB-TTL -> 电脑
- C2 `COM4 RXD2` -> USB-TTL -> 电脑
- C2 `PPS` -> Livox Avia 同步输入
- C2 `GND` -> Livox Avia 同步地
- 电脑上运行 `livox_ros_driver`

## 4. C2 输出配置

先通过串口进入 C2 命令行，再配置端口输出。

### 4.0 查询命令

查询串口配置：

```text
LOG COMCONFIG
```
列出当前正在输出消息的所有端口，及其正在输出的消息，输出到每个端口以及何时记录：

```text
LOG LOGLIST
```

### 4.1 串口与协议

建议将两个输出口都配置为北云 NMEA 格式：

```text
INTERFACEMODE COM3 BYNAV BYNAV
INTERFACEMODE COM4 BYNAV BYNAV
```

### 4.2 语句输出

#### COM3（软件上的COM3，配置、显示等方面都对应COM3；接线的时候对应的是COM2串口）：给 GNSS ROS 驱动
```text
LOG COM3 GPGGA ONTIME 1
LOG COM3 GPRMC ONTIME 1
```

#### COM4（软件上的COM4，配置、显示等方面都对应COM4；接线的时候对应的是COM1串口）：给 Livox 时间同步
```text
LOG COM4 GPRMC ONTIME 1
```

### 4.3 PPS 输出

```text
PPSCONTROL ENABLE POSITIVE 1 100000
```

其中脉宽为 `100000 us = 100 ms`，满足 Livox Avia 要求的 `20 ms < t1 < 200 ms`。

### 4.4 保存配置

```text
SAVECONFIG
```

说明：
- `GPRMC` 建议保持 1Hz
- `GPGGA` 也建议先用 1Hz
- 如后续确认链路稳定，可再按需要调整频率


## 5. GNSS ROS 驱动

### 5.1 处理逻辑

电脑上的 GNSS ROS 节点从 `COM3` 读取串口字节流，按 `\r\n` 切分 NMEA 语句后：
- 解析 `GPRMC`：提取 UTC 日期和时间，生成 ROS 时间戳
- 解析 `GPGGA`：提取经纬高、定位状态、卫星数、HDOP
- 融合成完整 `NavSatFix` 并发布话题

### 5.2 推荐话题

- `/rtk/navsatfix`
- `/rtk/raw_rmc`
- `/rtk/raw_gga`

### 5.3 关键规则

- 时间戳必须用 `GPRMC` 的完整日期+时间
- 不能只用时分秒
- `GPGGA` 只负责位置信息和质量信息
- 需要按 UTC 时间字段配对，不按串口到达顺序配对

## 6. Livox Avia 时间同步

### 6.1 原理

Livox ROS driver 会从 `timesync_config.device_name` 指定的串口设备中直接读取 `GPRMC/GNRMC`，然后通过 Livox SDK 的时间同步接口把 UTC 送入 Avia。

它不是读取 ROS 话题，而是读取串口设备本身。

### 6.2 配置文件

修改 `livox_lidar_config.json` 中的 `timesync_config`：

```json
"timesync_config": {
  "enable_timesync": true,
  "device_name": "/dev/ttyUSB1",
  "comm_device_type": 0,
  "baudrate_index": 6,
  "parity_index": 0
}
```

说明：
- `device_name` 改成实际连接 `C2的COM4` 的设备名
- `baudrate_index: 6` 对应 115200
- `parity_index: 0` 表示无校验

### 6.3 同步模式

Livox Avia 必须使用 `GPS (PPS + UTC)` 模式，而不是只用 PPS。

原因：
- PPS 只能保证脉冲对齐
- 只有 PPS + UTC 才能把点云时间戳变成绝对 UTC

## 7. 执行步骤

### 7.1 上电前检查
1. 确认 C2 28 针引脚已经引出 COM3、COM2、PPS、GND
2. 确认 USB-TTL 转换器为 3.3V 电平
3. 确认 Livox Avia 同步口支持 3.3V LVTTL
4. 确认所有地线共地

### 7.2 配置 C2
1. 连接电脑串口工具
2. 执行 `INTERFACEMODE` 配置
3. 执行 `LOG` 配置
4. 执行 `PPSCONTROL` 配置
5. 执行 `SAVECONFIG`
6. 重启 C2

### 7.3 启动 GNSS 驱动
1. 电脑连接 `COM3`
2. 启动 GNSS ROS 节点
3. 检查 `NavSatFix` 是否连续发布
4. 检查 `header.stamp` 是否为绝对 UTC

### 7.4 启动 Livox 时间同步
1. 电脑连接 `COM2`
2. 修改 `timesync_config.device_name`
3. 启动 `livox_ros_driver`
4. 检查 Avia 是否进入 UTC 同步状态

## 8. 测试方法

### 8.1 串口测试
- 用串口工具查看 `COM3` 是否输出 `GPGGA` 和 `GPRMC`
- 用串口工具查看 `COM2` 是否仅输出 `GPRMC`
- 检查每条语句是否带校验和
- 检查输出频率是否为 1Hz

### 8.2 GNSS 驱动测试
- 订阅 `NavSatFix`
- 检查经纬高是否正确
- 检查时间戳是否与 C2 输出 UTC 一致
- 检查消息是否连续稳定

### 8.3 Livox 同步测试
- 在 Livox Viewer 或 ROS driver 中查看同步状态
- 确认 Avia 已锁定 GPS 同步
- 观察点云时间戳是否稳定且与 UTC 对齐

### 8.4 联动测试
- 记录一段点云和 GNSS 数据
- 按同一 UTC 时间轴对齐检查
- 验证 LIGO 能按激光帧时间戳检索对应的经纬高

## 9. 风险与注意事项

- 不要把 `GPRMC` 直接接到 Avia 同步口，Avia 同步口只接 PPS
- 不要只用 PPS 做绝对时间同步
- 不要用 `GGA` 单独生成 UTC 时间戳
- 不要让两个程序同时抢占同一个串口
- 如果串口波特率不足，优先保留 `GPRMC` 给 Livox，同步链路优先级最高
