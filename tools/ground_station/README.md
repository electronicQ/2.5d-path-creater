# ground_station

这是运行在操作人员笔记本上的跨平台上位机，使用 PySide6 连接车辆工控机的 `vehicle_gateway`。

## 安装运行

Windows PowerShell：

```powershell
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r tools/ground_station/requirements.txt
python -m tools.ground_station.main
```

Linux：

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r tools/ground_station/requirements.txt
python3 -m tools.ground_station.main
```

在“连接”页填写车端 IP、`19090` 端口和网关令牌，点击测试连接。两台电脑必须能在同一 Wi-Fi 内互相访问；如果 Wi-Fi 启用了客户端隔离，需要关闭隔离或改用有线网络。

## 操作顺序

1. 在“地图上传”页选择 `.pcd`，填写地图 ID 和 ENU 原点，上传完成后等待 `READY`。
2. 在“地图转换”页填写 PGM 名称，启动转换并等待 `MAP_READY`。
3. 在“路径规划”页填写 PGM 名称、起点、目标点和高级 JSON 参数。
4. 规划成功后，上位机会自动下载轨迹 CSV、三个 `road_map_*.xml`、`overlay.png` 和解析配置到 `~/vehicle_ground_station/jobs/<job_id>/`。

点云和结果文件均支持断点续传；文件完成后会校验 SHA-256。服务端仍会做最终路径、参数和权限校验。

## 打包

在目标系统上安装 `pyside6-deploy`，然后执行：

```bash
pyside6-deploy tools/ground_station/main.py --name vehicle-ground-station --force
```

必须在目标操作系统上分别构建 Windows 和 Linux 包。构建产物不要包含车端工程文件，只需要上位机程序和 Qt 运行库。
