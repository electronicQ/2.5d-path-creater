# Ubuntu 18.04 / ROS Melodic 原生部署

本分支为 `kunpengfinal_18.04`。目标是保留车上已有 Melodic/Python 2 环境，在独立 Python 3 环境运行 TCP 服务和规划算法。协议和九个轨迹文件的定义保持一致，地面端无需更改二进制协议。

## 1. 环境与依赖

- Ubuntu 18.04，ROS1 Melodic；退出 conda/virtualenv，不要加载 Noetic 或 ROS2。
- 普通用户有 sudo 权限，可访问 Ubuntu/ROS 软件源、GitHub raw、python.org、PyPI。
- 默认并发 2，内存不足时 `--jobs 1`。首次部署需要为 Python 源码、PCL/ROS 依赖和 catkin 产物预留磁盘空间。
- RViz 插件使用 `qtbase5-dev` 声明 Qt 依赖，避免当前 rosdep 将旧 `libqt5-*` 名称映射为 Bionic 不存在的 `*t64` 包。
- 源码使用 Melodic 与 Noetic 都提供的 `filters/*.h` 头文件；修正 `grid_map_to_occupancy` 对不存在的 include 目录和库的导出，支持干净下载后直接配置。
- C++ 使用系统 GCC/CMake 以及 Eigen、Glog、PCL、OpenCV、经典 TBB、yaml-cpp、OctoMap、Qt5。
- ROS 节点和 catkin 使用 `/usr/bin/python2.7`。保存器依赖系统 `python-numpy`、`python-pathlib2` 和 ROS `nav_msgs/rospy`，直接写二进制 PGM，不依赖 Python OpenCV/cv_bridge。
- TCP 服务和规划器使用 `.runtime/planner/bin/python`（Python 3.8.20），依赖固定在 `requirements-melodic.txt`。

Python 从官方源码下载并验证 SHA256，安装到 `.runtime/python38`，再创建 `.runtime/planner` 虚拟环境。不使用 update-alternatives，不替换 `/usr/bin/python`、`/usr/bin/python3`，不把 Python 3 包装到系统 Python 2 环境。Python 3.8 和 Melodic 均属于旧版本栈，本分支用于兼容既有工控机，不代表恢复其上游维护。

## 2. 下载与自动部署

```bash
git clone --branch kunpengfinal_18.04 --single-branch git@github.com:electronicQ/2.5d-path-creater.git
cd 2.5d-path-creater
bash scripts/deploy_vehicle.sh --setup-ros-repo --jobs 2
```

已有有效 ROS 软件源可省略 `--setup-ros-repo`。该选项仅在未发现官方 ROS 源时添加 bionic 源，不会覆盖已有源；失效密钥或错误发行版的旧源需要先修复。GitHub SSH 需要已配置公钥，也可以改用 HTTPS 克隆地址。

流程：系统检查 → apt 安装 ROS/库和 Python 构建依赖 → rosdep 安装并检查依赖（包含 EOL 发行版）→ 校验系统 Python 2 的 ROS 导入 → 编译本地 Python 3.8 → 安装锁定版本的二进制 wheels → catkin 编译 → 检查节点和服务入口。

```bash
# 分阶段执行
bash scripts/deploy_vehicle.sh --deps-only
bash scripts/deploy_vehicle.sh --build-only --jobs 1
```

部署日志：`log/deploy/melodic-日期-进程号.log`。失败后修复错误再运行；不会忽略错误继续编译。`--build-only` 不下载安装依赖。

从别的机器复制工程时，不要复制 `build/`、`devel/`、`.runtime/`。这些目录记录了原机器的路径和解释器信息。已有这些目录时，先备份移走，再部署。地图可单独保留。

## 3. 准备地图并启动

1. 将局部 ENU 点云放到 `map/pcd/`，例如 `scans_Global_ENU.pcd`。
2. 在 `tools/path_creater/config/map_enu.meta.yaml` 填写该地图的实际原点经纬高。
3. 检查 `tools/ground_vehicle_tcp/config.yaml` 中的 PCD 文件名、路径、端口、转换超时。ROS 转换器默认读取 `map/pcd/`，不要仅修改 `pcd_path` 的目录。
4. 在项目根目录启动：

```bash
bash scripts/run_vehicle.sh
```

运行脚本自动加载 `/opt/ros/melodic/setup.bash` 和 `devel/setup.bash`，保留 ROS Python 2 路径给 ROS 子进程，再清除服务进程的 `PYTHONPATH/PYTHONHOME`，用独立 Python 3.8 启动 TCP 服务。

内部 `VEHICLE_ROS_PYTHON` 指定保存器解释器（Melodic 为 `/usr/bin/python2.7`），`VEHICLE_ROS_PYTHONPATH` 保留 ROS 模块路径。规划子进程使用服务自身的 `sys.executable`，不会误用系统 Python 3.6。通常无需手工设置这些变量。

## 4. 客户端和测试

```bash
# 另开车端终端，位于项目根目录
# 以下坐标仅示意，必须换成地图内的可通行点
env -u PYTHONPATH -u PYTHONHOME .runtime/planner/bin/python -m tools.ground_vehicle_tcp.client \
  --host 127.0.0.1 --port 8079 \
  --start-x 0 --start-y 0 --goal-x 10 --goal-y 5 \
  --timeout 120 --response-timeout 2400

# Python 3 服务、规划、跨环境边界测试
env -u PYTHONPATH -u PYTHONHOME .runtime/planner/bin/python -m unittest \
  tests.test_ground_vehicle_tcp tests.test_melodic_runtime tests.test_geo \
  tests.test_map_saver_conversion tests.test_map_metadata tests.test_map_semantics \
  tests.test_planner_gray_cost tests.test_path_smoothing tests.test_curve_smoothing \
  tests.test_trajectory_outputs tests.test_grid_map_filter_chain tests.test_overlay_orientation \
  tests.test_time_tools -v

# 系统 Python 2 的 PGM 保存格式与经典 TBB 编译冒烟测试
/usr/bin/python2.7 tests/deployment/verify_legacy_saver.py
mkdir -p /tmp/vehicle-tbb-smoke
cd /tmp/vehicle-tbb-smoke
cmake /实际路径/2.5d-path-creater/tests/deployment
cmake --build .
./classic_tbb_smoke
```

TCP 字段仍是局部 UTM 偏移（毫米编码），不是 ENU；成功后在车端 `output/` 读取轨迹。跨机器对接和错误码详见 [TCP 手册](../tools/ground_vehicle_tcp/README.md)。

## 5. 开机服务

先手动运行并完成一次地图转换和规划，再安装服务。仓库单元文件已配置为账号 `union`、项目路径 `/home/union/2.5d-path-creater`：

```bash
sudo install -m 0644 tools/ground_vehicle_tcp/ground-vehicle-tcp.service \
  /etc/systemd/system/ground-vehicle-tcp.service
```

如果实际项目路径或账号不同，安装前编辑单元文件中的以下配置：

```ini
User=union
WorkingDirectory=/home/union/2.5d-path-creater
ExecStart=/bin/bash /home/union/2.5d-path-creater/scripts/run_vehicle.sh --config /home/union/2.5d-path-creater/tools/ground_vehicle_tcp/config.yaml
```

使用同一个部署账号保证文件权限一致，不需要在 systemd 中激活虚拟环境。服务会把 ROS 日志写到 `/home/union/.ros/log`。若 ROS master 位于远端，通过 systemd drop-in 设置与手动环境一致的 `ROS_MASTER_URI`、`ROS_IP` 等网络变量。

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now ground-vehicle-tcp.service
sudo journalctl -u ground-vehicle-tcp.service -f
```

## 6. 排障

| 问题 | 处理 |
| --- | --- |
| apt update 失败或找不到 Melodic 包 | 核对系统为 bionic、ROS 源/签名密钥有效和网络可达；不要改为 focal 或添加不受信任选项 |
| rosdep update 失败 | 检查 GitHub raw/ROS 索引访问；脚本已启用 EOL 发行版 |
| Python 源码 SHA256 校验失败 | 删除 `.runtime/downloads/Python-3.8.20.tar.xz` 后重新下载，不要跳过校验 |
| Python 缺少 ssl/bz2/lzma | 确认构建依赖安装成功，移走 `.runtime/python38`、`.runtime/planner` 和 `.runtime/build/Python-3.8.20` 后重新部署 |
| pip 提示无匹配 wheel | 核对 CPU 架构、Python 3.8、pip 索引是否包含锁定版本；脚本不自动退回庞大的源码编译 |
| 系统 Python3 报 future annotations 错误 | 误用了 3.6；改用 `scripts/run_vehicle.sh` 或 `.runtime/planner/bin/python` |
| Python 3 导入 ROS Python 2 的 yaml 报语法错误 | 使用运行脚本；单独跑规划器/测试时加 `env -u PYTHONPATH -u PYTHONHOME` |
| 缺少 TBBConfig.cmake | 本分支包含经典 TBB 查找模块；确认已安装 `libtbb-dev` 且使用本分支源码 |
| task_scheduler_init.h 缺失 | 使用 18.04 经典 TBB，不要混入 oneTBB 2021 的头文件 |
| catkin 报 Python 或 CMake 缓存不匹配 | 检查 `build/CMakeCache.txt` 的 `PYTHON_EXECUTABLE`；脚本接受解析到系统 Python 2.7 的别名。若是另一解释器或已失效路径，备份移走 `build/`、`devel/` 后运行 `--build-only`；保留地图和 `.runtime/` |
| 编译进程被 killed | 通常是内存不足，先用 `--jobs 1` 并检查系统内存/内核日志 |
| 服务能连上但地图转换失败 | 检查任务 `job.log`、PCD 路径、原点和 ROS 话题，确认没有其他转换节点冲突 |

支持状态应以实际编译及本机地图联调结果为准。容器中的验证也不能替代工控机上的真实点云、内存和车地网络验收。

## 7. 本分支验证记录（2026-10-26）

- 在 Ubuntu 18.04 amd64 容器中，以系统 Python 2.7.17 验证地图保存器的 PGM 字节、图像翻转、未知值 205 和 YAML 原点。
- 在 Ubuntu 18.04、GCC 7.5、系统经典 TBB 环境中，验证自带 FindTBB 模块、C++17 编译及 parallel_for/task_scheduler_init 执行。
- 在 Ubuntu 18.04/Melodic 容器中，使用实际 Python 3.8.20 和 `requirements-melodic.txt` 的锁定依赖，通过 TCP、坐标、地图和轨迹输出测试；包含真正启动规划子进程、隔离错误 Python 路径并校验三份 XML 与叠加图的测试。
- 全量测试发现 5 项历史失败：`test_gnss_timestamp_compensator`、`test_ligo_nmea_init_keyframes` 引用已移除的源码，`test_project_paths` 中一项引用不存在的 `banfu2.meta.yaml`。这些不作为本次适配通过的测试项。

- 最终自动部署脚本已在目标容器中重新完整执行成功，包含 apt/rosdep 检查、Python 环境检查、catkin 编译和服务入口检查。
- 整个 catkin 工作区在 Ubuntu 18.04 / Melodic / GCC 7.5 / CMake 3.10 下编译通过，服务启动入口检查通过。
- 在隔离的 Melodic 工作区，通过合成 PCD 的真实 ROS 转换和两次 TCP 规划请求；均返回成功，三份 XML 和叠加图通过格式校验，第二次请求复用地图缓存。对应测试为 `tests/deployment/smoke_melodic_pipeline.py`，文件开头附专用测试工作区中的执行方式。该测试使用固定转换节点名和输出话题，只在隔离工作区运行。

以上验证使用 amd64 容器；真实工控机上的原始点云、内存容量、网络和下游循迹仍需现场验收。其他 CPU 架构尚未完成同等验证。


## 8. 大地图滤波长时间运行：先检查编译优化

若 `filters_demo` 持续占用约一个 CPU 核心、没有交换读写但十几分钟仍未完成，先检查：

```bash
grep -E '^(CMAKE_BUILD_TYPE|CMAKE_CXX_FLAGS(_RELEASE)?):' build/CMakeCache.txt
```

`CMAKE_BUILD_TYPE:STRING=` 为空时，不能因为存在 `CMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG` 就认为已启用优化；这些 Release 参数仅在选择 Release 构建时生效。大量 Eigen 运算和邻域迭代在未优化构建下可能非常慢。

先用 Ctrl+C 停止正在运行的两个地图转换/滤波 roslaunch，退出 conda，然后在项目根目录执行：

```bash
source /opt/ros/melodic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python2.7 \
  -DCATKIN_ENABLE_TESTING=OFF -j2 -l2
source devel/setup.bash
```

通常无需删除 `build/` 和 `devel/`，更改构建类型会触发重新编译。检查缓存中的构建类型已变成 `Release`，再重启节点。若整个部署环境已经准备好，也可运行 `bash scripts/deploy_vehicle.sh --build-only --jobs 2`，其中已显式设置 Release。

先保持原地图分辨率、邻域半径及 `area` 法向量算法。先用原串行配置确认 Release 重编译带来的改善；若仍需加速，再显式启用法向量的 `parallelization_enabled: true` 和 `thread_number: 4`。均值平滑和滑窗表达式阶段仍为串行。接收地图、主要滤波阶段开始/结束和输出发布均增加耗时日志，用于区分等消息、均值平滑、法向量、边缘计算与序列化开销。

地图点云转换和可通行地图导出无需打开 RViz，可在手动 roslaunch 时传入 `enable_rviz:=false`。关闭 RViz 后，应先在另一个已加载 `devel/setup.bash` 的终端运行 `/usr/bin/python2.7 tools/pgm_saver/save_continuous_map.py`，再启动滤波 launch；可视化节点需要有输出订阅者才会订阅并转换地图，保存器提供占据栅格订阅。不要仅凭整体 CPU 空闲率断言节点未工作：12 个逻辑 CPU 中一个核满载，进程通常显示约 100% CPU。
