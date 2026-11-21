# Chapt4 项目上下文（承接 Chapt3）

> 本文件是 Chapt4（Litra LTME-02A + SLAM Toolbox 建图）的结论摘要，供后续会话直接继承。
> 完整说明见 `README.md`；Chapt3 的硬件、底盘、IMU、EKF、URDF 和 RPLIDAR 结论见 `/home/orangen/chapt3/AGENTS.md`。

## 1. 边界与依赖

- **Chapt3 目录只读**：`/home/orangen/chapt3` 不得修改。
- Chapt4 通过 overlay 复用 Chapt3：
  ```text
  /opt/ros/jazzy
    -> ~/chapt3/chapt3_ws/install
      -> ~/chapt4/chapt4_ws/install
  ```
- `start_mapping.sh`、`save_map.sh` 会依次 source 两个工作空间；`CHAPT3_WS` 可覆盖 Chapt3 路径。
- 运行时接口保持：`/scan`、`/odom`、`/imu`、`/odometry/filtered`、`odom -> base_link`、`/cmd_vel`。
- 树莓派网络：WiFi 为 `192.168.8.144`（虚拟机同网段，IP 用 `hostname -I` 查询）；`eth0` 未配置 IP，预留给 Litra 网口雷达。

## 2. 已完成内容

- 包：`chapt4_ws/src/chapt4_slam`、`chapt4_ws/src/chapt4_bringup`（ament_cmake）。
- 官方 Litra ROS 2 驱动 vendored 到 `chapt4_ws/src/vendor/`：
  - `ltme_interfaces`
  - `ltme_node`（LitraTech ROS2 v1.0.2-20240921，支持 LTME-02A / LT-R1 / LT-R2 / LT-I1 / LT-I2）
- 默认雷达：**Litra LTME-02A**，RJ45 以太网，默认 IP `192.168.10.160`，TCP 2105，12V DC/1A 供电，不支持 PoE。
- SLAM：`async_slam_toolbox_node`，`mode: mapping`。
- 配置：`chapt4_slam/config/slam_toolbox_mapping.yaml`。
- Litra 配置：`chapt4_bringup/config/ltme_02a.yaml`。
- 启动：`chapt4_bringup/launch/mapping.launch.py`
  - `lidar_type:=litra`（默认）→ 关闭 Chapt3 RPLIDAR，启动 `ltme_node`
  - `lidar_type:=rplidar` → 回退 Chapt3 RPLIDAR A1
  - `lidar:=false` → 两种雷达都不启动
  - `slam:=false` → 只启动底盘/雷达
  - `rviz:=true` → 树莓派本机 RViz
- 单独雷达测试：`chapt4_bringup/launch/lidar_litra.launch.py`
- 网络配置：顶层 `setup_litra_network.sh`（给 eth0 加 `192.168.10.100/24` 并 ping 雷达）。
- 地图保存：`chapt4_bringup/scripts/save_map.sh` + 顶层 `save_map.sh`。
- 虚拟机：`vm_tools/`（`chapt4.rviz`、`start_rviz.sh`、`start_teleop.sh`、`fastdds_no_shm.xml`）。

## 3. 坐标系与参数

SLAM Toolbox 与 Cartographer 参数对应：

| Cartographer | SLAM Toolbox | 值 |
|---|---|---|
| `tracking_frame` | `base_frame` | `base_link` |
| `published_frame` | `odom_frame` | `odom` |
| `map_frame` | `map_frame` | `map` |
| `scan_topic` | `scan_topic` | `/scan` |

TF：

```text
map -> odom (slam_toolbox)
odom -> base_link (EKF)
base_link -> litra_link (Chapt4 static TF, default 0.08, 0, 0.21; Litra)
base_link -> laser_link (URDF, 0.08, 0, 0.21; RPLIDAR fallback)
base_link -> gyro_link (URDF)
```

Litra 的 `LaserScan.frame_id=litra_link`。如果实际安装位置或朝向不同，用 `litra_x/y/z/yaw/pitch/roll` 调整，不要改 Chapt3 URDF。

关键 SLAM 参数：`resolution=0.05`、`max_laser_range=20.0`、`minimum_time_interval=0.10`、`minimum_travel_distance=0.10`、`minimum_travel_heading=0.10`、`do_loop_closing=true`。

## 4. 使用

```bash
# Litra 网络（每次树莓派重启后）
~/chapt4/setup_litra_network.sh eth0
# 默认给 eth0 加 192.168.10.100/24，并 ping 192.168.10.160

# 单独测试 Litra
source /opt/ros/jazzy/setup.bash
source ~/chapt4/chapt4_ws/install/setup.bash
ros2 launch chapt4_bringup lidar_litra.launch.py
# 另一个终端
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
ros2 run tf2_ros tf2_echo base_link litra_link

# 完整建图
~/chapt4/start_mapping.sh
~/chapt4/start_mapping.sh lidar_type:=rplidar   # 回退 RPLIDAR
~/chapt4/start_mapping.sh litra_address:=192.168.10.161
~/chapt4/start_mapping.sh litra_model:=LT-I1
~/chapt4/start_mapping.sh slam:=false

# 保存地图
~/chapt4/save_map.sh room
# -> ~/chapt4/maps/room.yaml + room.pgm

# 虚拟机（同步一次）
mkdir -p ~/chapt4_vm
scp orangen@192.168.8.144:/home/orangen/chapt4/vm_tools/* ~/chapt4_vm/
chmod +x ~/chapt4_vm/*.sh
~/chapt4_vm/start_rviz.sh
~/chapt4_vm/start_teleop.sh
```

编译：

```bash
cd ~/chapt4/chapt4_ws
source /opt/ros/jazzy/setup.bash
source ~/chapt3/chapt3_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

编译包：`chapt4_slam`、`chapt4_bringup`、`ltme_interfaces`、`ltme_node`。

## 5. 本机验证结果

### 2026-11-16：RPLIDAR 回退链路

- `chapt4_slam`、`chapt4_bringup` 编译通过。
- `start_mapping.sh` 成功启动 Chapt3 bringup + SLAM。
- `/scan` 约 6.9Hz；SLAM 日志 `Registering sensor: [Custom Described Lidar]`。
- `/map` 发布正常；冒烟测试地图 166×82 @0.05m。
- `map -> base_link` TF 可查询。
- `save_map.sh` 在临时目录成功生成 `smoke.yaml` / `smoke.pgm`。

### 2026-11-18：Litra 驱动

- `ltme_interfaces`、`ltme_node`、`chapt4_slam`、`chapt4_bringup` 4 个包编译通过。
- `lidar_litra.launch.py` 可启动 `ltme_node`，并发布 `base_link -> litra_link` 静态 TF（默认 0.08, 0, 0.21）。
- `ros2 topic info /scan -v` 显示 `ltme_node` Publisher count 1；参数 `device_model=LTME-02A`、`frame_id=litra_link`。
- 未接雷达时日志为 `Waiting for device... [192.168.10.160]`，驱动会重试。
- 2026-11-20 实机验证：mDNS 识别为 `LTME-02_0197` / `model=LTME-02A`；`/scan` 约 15.05Hz，`frame_id=litra_link`，有有效距离数据；参数 `LTME-02A`、`range_max=20.0`。

### 2026-11-21：完整建图尝试

- IMU 未插时 `/imu Publisher count: 0`，`ahrs_driver_node` 启动后退出；EKF 只剩轮式角速度，地图明显漂移。
- 插回 N100 后 `/imu` 约 100Hz，`gyro z bias calibrated` 正常。
- 错误传入 `litra_x:=... litra_y:=... litra_z:=...`，`static_transform_publisher` 退出，SLAM 报 `Message Filter ... queue is full`；改为默认数值后恢复。
- 保存过 1511×1431 地图（`maps/room.yaml/.pgm`），但右侧有射线伪影，说明建图仍需外参/IMU/速度调优。

## 6. 已踩过的坑

- **脚本不能用 `set -u`**：ROS 2 的 `setup.bash` 会引用未定义变量（`AMENT_TRACE_SETUP_FILES`），脚本使用 `set -eo pipefail`。
- **launch 参数名冲突**：Chapt3 `bringup.launch.py` 也声明 `lidar`。如果 `litra_lidar` include 放在 Chapt3 include 之后，Chapt3 会把全局 `lidar` 配置改成 false，导致 Litra 条件为假。当前已把 `litra_lidar` 放在 `bringup` 之前，改动顺序时要小心。
- **Litra 不是 USB 雷达**：LTME-02A 是 RJ45 网口，默认 IP `192.168.10.160`，需要树莓派 `eth0` 有 `192.168.10.x` 地址；不要插 `/dev/rplidar`。
- **NetworkManager 会清掉手工 IP**：`ip addr add` 的 `192.168.10.100/24` 可能被 NM 的 DHCP profile 移除；当前已用 `/etc/netplan/99-litra-eth0.yaml` 固定。
- **LTME-02A 只允许一个 TCP 客户端**：另一个 `ltme_node` 或 LVS2D 占用 2105 时，新驱动会一直 `Waiting for device`；`pgrep -af ltme_node` 清理旧进程或重启雷达。
- **Litra 供电 12V/1A**：不能直接接 5V USB；LTME-02A 不支持 PoE。
- **`/map` 是 Transient Local**：`ros2 topic hz /map` 可能没有输出，用 `ros2 topic echo /map --once --field info`。
- **不要同时启动 Chapt3 `display.launch.py` 和 Chapt4 的 RPLIDAR 回退**：两个 RPLIDAR 节点会抢 `/dev/rplidar`。
- **IMU 必须在线**：`/dev/wheeltec_FDI_IMU_GNSS` 缺失时 EKF 只剩轮式角速度，地图会越来越歪；启动后前 2~3 秒必须静止等 `gyro z bias calibrated`。
- **静态 TF 参数必须是数字**：`litra_x:=...` 会让 `static_transform_publisher` 退出，SLAM 因缺少 `base_link -> litra_link` 报 `Message Filter ... queue is full`。
- **RPLIDAR A1 扫描冻结**：沿用 Chapt3 结论，`Standard` 模式，必要时拔插 USB。
- **跨机 Fast DDS**：`start_mapping.sh` 和 VM 脚本都设置 UDP-only；VM 若报 `Bad alloc ParticipantEntitiesInfo`，先 `ros2 daemon stop`。
- **WiFi 屏蔽组播**：当前 `GDDQSDC-5G` 网络下，虚拟机可能只看到 `/scan`/`/map` 而看不到 `/tf`（RViz 报 `No tf data`）。VM 脚本已默认 `ROS_STATIC_PEERS=192.168.8.144`；如仍不行，Pi 启动时也设置 `ROS_STATIC_PEERS=<虚拟机IP>`。
- **VMware 必须桥接 WiFi**：`ens33 no carrier` 通常是 VMnet0 不存在或桥接到了有线 Realtek 网卡；在虚拟网络编辑器添加 VMnet0 并桥接当前 WiFi，虚拟机关机再开机。NAT 模式不能用于 ROS 2 跨机建图。
- **地图保存**：`map_saver_cli` 只保存栅格图；若要 Phase 5 定位模式，额外调用 `/slam_toolbox/serialize_map` 保存 `.posegraph`。

## 7. 资产位置

| 内容 | 路径 |
|---|---|
| Chapt4 工作空间 | `/home/orangen/chapt4/chapt4_ws` |
| Litra 驱动（vendored） | `chapt4_ws/src/vendor/ltme_node`、`ltme_interfaces` |
| Litra 参数 | `chapt4_ws/src/chapt4_bringup/config/ltme_02a.yaml` |
| Litra 单独启动 | `chapt4_ws/src/chapt4_bringup/launch/lidar_litra.launch.py` |
| 网络配置 | `/home/orangen/chapt4/setup_litra_network.sh` |
| SLAM 配置 | `chapt4_ws/src/chapt4_slam/config/slam_toolbox_mapping.yaml` |
| 建图 bringup | `chapt4_ws/src/chapt4_bringup/launch/mapping.launch.py` |
| 地图保存 | `chapt4_ws/src/chapt4_bringup/scripts/save_map.sh` |
| 地图输出 | `/home/orangen/chapt4/maps` |
| 虚拟机工具 | `/home/orangen/chapt4/vm_tools` |
| Chapt3（只读） | `/home/orangen/chapt3` |

## 8. 后续

- Litra 实机已接通：网线接 `eth0`、12V 上电、`setup_litra_network.sh`、`/scan` 约 15Hz。
- 重新标定雷达外参并接回 IMU，重新建一张无漂移地图。
- 根据实际安装测量 `litra_x/y/z/yaw/pitch/roll`，必要时调整 `invert_frame`。
- 实车走完整房间，生成 `maps/room.yaml` + `maps/room.pgm`。
- Nav2 导航（代价地图、AMCL、路径规划）。
- 可选的 SLAM Toolbox 定位模式：`mode: localization` + `map_file_name`（需要 `.posegraph`）。
- 工程化：systemd 常驻、网络地址持久化、日志分级。
