# vehicle_gateway

`vehicle_gateway` 是运行在车辆工控机上的常驻 TCP 服务。它接收地面端上位机发送的 PCD、ENU 原点、地图转换和路径规划请求。

与外部上位机开发公司对接时，以 [`docs/vehicle_gateway_protocol.md`](../../docs/vehicle_gateway_protocol.md) 为正式协议规范。该文档定义了：

- TCP 长度前缀帧格式和网络字节序；
- JSON header 与二进制 payload；
- `hello` 认证；
- PCD 分块上传、SHA-256 校验和断点续传；
- 地图转换和规划请求；
- `job.status` 状态查询与推送；
- `artifact.list` 和 `artifact.download` 结果下载；
- 错误码和完整交互时序。

## 车端启动

```bash
cd /home/cclg/muti_path_creater
source devel/setup.bash
python3 -m tools.vehicle_gateway.main \
  --config tools/vehicle_gateway/gateway.yaml
```

默认监听 `0.0.0.0:19090`。正式部署前修改 `gateway.yaml` 中的 `token`，并在车端防火墙中只允许指定地面端 IP 访问该端口。

也可以使用 systemd：

```bash
sudo cp tools/vehicle_gateway/vehicle-gateway.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now vehicle-gateway.service
```

车端必须先运行 `roscore`，并确保 `devel/setup.bash` 能找到 `roslaunch`、`rostopic` 和本工程 ROS 包。
