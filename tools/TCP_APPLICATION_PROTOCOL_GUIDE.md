# TCP 自定义应用层协议学习笔记

本文档整理本项目车端网关和地面端上位机之间的 TCP 通信原理，帮助理解 `vehicle_protocol.py`、`vehicle_gateway/` 以及类似 CAN Socket 的区别。

## 1. TCP 为什么需要自定义帧

TCP 提供的是可靠、有序的字节流：

```text
字节 1、字节 2、字节 3、字节 4、......
```

TCP 不知道“这一条命令”或“这一份文件”在哪里结束。因此客户端连续发送多条消息时，服务端一次读取可能得到：

```text
一条消息的一部分
一条消息的剩余部分 + 下一条消息的一部分
多条消息拼在一起
```

应用层必须自行定义消息边界。本项目用长度前缀协议解决这个问题。

## 2. 本项目的一帧长什么样

```text
+----------------------+ 4 bytes, uint32, big-endian
| frame_length         | 后续 body 的总字节数
+----------------------+
| header_length        | 4 bytes, uint32, big-endian
+----------------------+
| JSON header          | header_length bytes, UTF-8
+----------------------+
| binary payload       | 剩余字节，可为空
+----------------------+
```

长度关系：

```text
frame_length = 4 + header_length + payload_length
payload_length = frame_length - 4 - header_length
```

`frame_length` 不包括它自己的 4 字节；它包括后面的 `header_length` 字段。

## 3. 也可以使用固定二进制头协议

上面的 JSON 头协议不是 TCP 应用层协议唯一的设计方式。对于业务比较简单、消息类型固定的系统，也可以定义为一个固定格式的二进制帧：

```text
+--------+---------+--------+--------+--------+---------+---------+
| magic  | version | type   | length | seq    | payload | crc     |
| 2 B    | 1 B     | 2 B    | 4 B    | 4 B    | N B     | 2/4 B   |
+--------+---------+--------+--------+--------+---------+---------+
```

推荐把所有多字节整数统一定义为网络字节序（大端）：

| 字段 | 长度 | 作用 |
| --- | ---: | --- |
| `magic` | 2 字节 | 固定帧标识，例如 `0x56 0x47`，用于快速判断帧头并辅助重新同步 |
| `version` | 1 字节 | 协议版本，后续字段调整时可以区分不同版本 |
| `type` | 2 字节 | 消息类型编号，例如 `0x0001` 表示连接测试、`0x0002` 表示地图上传 |
| `length` | 4 字节 | `payload` 的字节数，取值为 `N`。也可以约定为整帧长度，但双方必须固定一种定义；推荐定义为 payload 长度 |
| `seq` | 4 字节 | 消息序号或分块序号，用于应答匹配、排序和断点续传 |
| `payload` | `N` 字节 | 业务数据，可以是定长二进制结构，也可以是 UTF-8 JSON 或文件分块 |
| `crc` | 2 或 4 字节 | 对约定范围内的字段做校验，常见为 CRC-16 或 CRC-32 |

如果 `length` 表示 payload 长度，则整帧长度为：

```text
frame_size = 2 + 1 + 2 + 4 + 4 + length + crc_size
```

例如使用 CRC-16 时，固定部分为 15 字节；使用 CRC-32 时，固定部分为 17 字节。CRC 的计算范围也必须写进正式协议，例如“从 `magic` 到 `payload` 的所有字节，不包含 `crc` 本身”。

这种协议同样需要先读取固定头，再根据 `length` 读取 payload，最后读取 CRC 并校验：

```text
1. 读取固定头的 13 字节（magic/version/type/length/seq）；
2. 检查 magic、version 和 length 是否有效；
3. 读取 length 个 payload 字节；
4. 读取 2 或 4 字节 CRC；
5. 校验 CRC，通过后再根据 type 解释 payload。
```

`magic` 的价值在于：如果协议实现支持丢弃无效字节并搜索下一个 magic，就可以在部分数据损坏后尝试重新找到帧边界。不过 TCP 本身保证字节可靠、有序，正常情况下不会随机丢失或修改字节；CRC 主要用于尽早发现协议实现错误、存储错误或数据拼接错误。若发现长度异常或 CRC 错误，最稳妥的处理仍然是关闭连接并重新建立连接，除非双方明确实现了 magic 重同步算法。

固定二进制协议比 JSON 头更紧凑、解析开销更低，适合命令数量少、字段稳定、对带宽和实时性要求高的场景。代价是协议扩展需要维护类型编号、字段版本和二进制布局，调试时也不如 JSON 直观。当前车端网关采用 JSON 头加二进制 payload，是因为上传、转换、规划和下载消息字段较多且经常包含嵌套参数；如果后续业务收敛，也可以将同样的业务消息改成这种固定二进制协议。

仍然不建议直接把 C/C++ 结构体的内存区域当作整帧发送。即使定义了类似下面的结构体：

```cpp
struct FrameHeader {
    uint16_t magic;
    uint8_t  version;
    uint16_t type;
    uint32_t length;
    uint32_t seq;
};
```

也必须逐字段按约定的字节序序列化。编译器可能插入 padding，不同语言的对齐规则也不同；`memcpy()` 或 `send(fd, &header, sizeof(header), 0)` 不能自动解决大端转换、结构体布局和 CRC 计算问题。可以在单一编译器和明确 ABI 的封闭环境中使用 packed 结构体，但跨 Windows/Linux、C++/C#/Python 通信时，显式序列化更可靠。

## 4. 发送和接收不是同一回事

应用层认为四个部分组成一帧：

```text
frame_length + header_length + JSON header + payload
```

发送端通常先把四部分拼成连续字节，再调用一次 `sendall()` 或 `write_frame()`。但是 TCP 底层仍可能把这次发送拆成多个网络数据段。

因此：

```text
一次 sendall() 不等于对端一次 recv()
```

接收端必须按以下顺序读取：

```text
1. 读取 4 字节 frame_length；
2. 检查长度是否在允许范围内；
3. 循环读取，直到收到 frame_length 个 body 字节；
4. 读取 body 前 4 字节 header_length；
5. 读取 JSON header；
6. 将剩余字节作为 binary payload；
7. 根据 JSON 中的 type 分发业务。
```

本项目的 `read_frame()` 使用 `readexactly()` 实现第 1～6 步，因此不依赖 TCP 的分包方式，也能处理粘包和拆包。

## 5. `vehicle_protocol.py` 中各函数的职责

```text
encode_frame()
    header + payload -> 完整帧字节

write_frame()
    调用 encode_frame()，通过 asyncio TCP writer 发送

read_frame()
    从 TCP 流先读长度，再读满一帧

decode_frame()
    解析已经完整读入内存的一帧字节
```

`encode_frame()` 是发送方向的序列化实现，`read_frame()` 是接收方向的读取和解析实现。完整协议还包括各消息 `type` 的字段约定、错误码和状态机，详见 [`docs/vehicle_gateway_protocol.md`](../docs/vehicle_gateway_protocol.md)。

## 6. 为什么控制信息用 JSON，文件用二进制

本项目消息类型较多：

```text
hello
map.upload.begin
map.upload.chunk
map.upload.commit
map.convert.start
plan.start
job.status
job.cancel
artifact.list
artifact.download
```

不同消息的字段不同，有些还包含嵌套的起点、终点和配置参数。因此采用：

```text
JSON header
    存放 type、request_id、job_id、offset、参数和状态

binary payload
    存放 PCD 或结果文件的原始字节
```

这样 C++、C#、Python 等上位机都容易实现，同时不会把大文件转成 Base64 导致额外膨胀。

## 7. 为什么 CAN 可以直接使用 `struct can_frame`

SocketCAN 提供的是 Linux 内核定义的 CAN 编程接口：

```cpp
#include <linux/can.h>
struct can_frame frame;
```

标准 CAN 数据帧的底层格式相对固定，通常包含：

```text
CAN ID
数据长度
最多 8 字节 data
```

CAN 控制器和驱动知道如何处理帧边界、仲裁、校验和发送。因此应用程序可以填写 `can_id`、`can_dlc`、`data` 后交给 SocketCAN。

但 `struct can_frame` 只定义了底层 CAN 帧，并没有定义 CANopen、J1939 或项目自定义协议中 `data[8]` 每个字节的含义。

TCP 则不同：

```text
CAN：驱动已经提供固定的底层帧接口
TCP：只提供连续字节流，应用层必须自己定义边界和内容
```

## 8. 能不能把 TCP 帧定义成结构体

可以定义逻辑结构体：

```cpp
struct Frame {
    uint32_t frame_length;
    uint32_t header_length;
    std::vector<uint8_t> json_header;
    std::vector<uint8_t> payload;
};
```

但不能直接把这个 C++ 对象裸写到 TCP：

```cpp
send(fd, &frame, sizeof(frame), 0);  // 不正确
```

原因是：

- `std::vector` 内部只是指针、长度和容量，不包含它指向的数据；
- 不同编译器和平台可能有不同的内存对齐和 padding；
- C++、C#、Python 的对象布局不一致；
- JSON 和 payload 是变长数据。

正确做法是：

```text
程序内部结构体
    -> 按协议逐字段序列化
    -> 明确的字节序和长度
    -> TCP 字节流
```

因此本项目虽然没有一个固定大小的 `struct Frame`，但有两个固定的二进制字段：

```text
uint32 frame_length
uint32 header_length
```

后面的 JSON 和 payload 根据长度动态解析。

## 9. 一帧和一个 PCD 文件的关系

一帧不等于整个 PCD 文件。由于 PCD 可能数百 MB，本项目将它拆成多帧：

```text
map.upload.chunk 第 1 帧
map.upload.chunk 第 2 帧
map.upload.chunk 第 3 帧
...
```

每个分块帧包含：

```text
JSON header：offset、分块 sha256、job_id
binary payload：PCD 当前分块字节
```

车端按照 `offset` 顺序写入临时文件，最后重新计算整个文件 SHA-256，校验通过后原子提交。

## 10. 如果协议解析出错怎么办

当前协议依赖双方从帧的第一个 4 字节开始正确读取。如果出现丢字节、错读或长度损坏，后续字节可能失去帧边界对齐。

当前推荐处理方式是：

```text
检测到 ProtocolError
-> 关闭当前 TCP 连接
-> 记录 request_id/job_id 和错误
-> 重新建立连接
-> 使用上传 offset 或 job.status 恢复任务
```

如果未来需要在损坏的字节流中重新寻找边界，可以在帧头增加固定 magic、版本和 CRC 字段；当前版本没有设计自动重同步字段。

## 11. 本项目通信栈总览

```text
地面端上位机
    -> GatewayClient
    -> vehicle_protocol.py
    -> TCP 字节流
    -> vehicle_gateway/service.py
    -> validation.py / storage.py / configuration.py
    -> runner.py
    -> ROS launch、PGM 保存器、路径规划器
```

可以把各层职责概括为：

```text
TCP
    可靠、有序地传输字节

vehicle_protocol.py
    定义并恢复应用层帧边界

vehicle_gateway
    解析消息、校验参数、执行任务、返回状态

地面端上位机
    提供界面、选择文件、发送请求、显示反馈和下载结果
```
