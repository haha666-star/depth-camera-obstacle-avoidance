# 交接文档 · 基于深度相机的 ROS2 自动避障

> 接手的人请按顺序读：**第 1 节（是什么）→ 第 2 节（环境）→ 第 4 节（怎么跑）→ 第 8 节（坑）**。
> 其余章节按需查阅。文中所有数据均为虚拟机内实测所得，标注了来源。

---

## 1. 项目是什么 / 做到了哪一步

### 1.1 一句话

在 **不依赖 Gazebo** 的轻量仿真环境里，实现并验证了「**深度相机自动避障**」，
算法以纯 Python 模块的形式与 ROS 解耦，可直接迁移到轮趣 WHEELTEC 真车。

### 1.2 项目定位

这是一个**算法验证 + 教学**用的仿真包，不是产品。它的价值在于：

1. 用最少的依赖（只需要 `rclpy` + `numpy`）跑通一条完整的「感知 → 决策 → 控制」链路；
2. **决策逻辑与 ROS 完全解耦** —— `depth_decision.py` 是纯 Python，不 import rclpy，
   可以脱离 ROS 单独测试，也可以整段搬到真车；
3. 把仿真里踩过的坑都留在注释里（每条注释都对应一次真实故障），避免重复踩。

### 1.3 当前状态

| 能力 | 状态 | 说明 |
|---|---|---|
| 深度相机自动避障 | ✅ 完成 | `/cmd_vel` 稳定 10 Hz，含三层自救机制 |
| 激光雷达 2D 建图（slam_toolbox） | ✅ 完成 | 建成 324×241 格 = 16.2 × 12.1 m，与房间真值吻合 |
| 深度相机建图（RTAB-Map） | ⚠️ 可用但有前提 | 仿真场景是纯色块、零纹理，视觉回环必然失败；已在 launch 里主动关掉回环 |
| 深度相机 3D 点云 | ✅ 完成 | 116,732 点带 RGB |
| 真车迁移 | ❌ **未做** | 已调研完毕，三个改造点见第 10 节 |

---

## 2. 开发环境

| 项 | 值 |
|---|---|
| 系统 | Ubuntu 22.04（运行在 VMware Workstation 虚拟机里） |
| ROS | **ROS 2 Humble** |
| 虚拟机镜像 | WHEELTEC 官方 Ubuntu 22.04 镜像 |
| 工作空间 | `~/wheeltec_ros2` |
| 包路径 | `~/wheeltec_ros2/src/sim_robot` |
| 虚拟机账号 | 用户名 `wheeltec` |

### 2.1 依赖

```bash
# 必需
rclpy sensor_msgs nav_msgs geometry_msgs std_msgs tf2_ros numpy

# 可选（按需）
slam_toolbox        # 激光建图
rtabmap_ros         # 深度建图
nav2_bringup        # 导航
rviz2               # 可视化
```

### 2.2 ⚠️ 编译方式（必须照做，否则一定失败）

这个镜像有一个**已知特性**：`ament_python` 包编译后，控制台脚本只装到
`install/sim_robot/bin/`，而 `ros2 run` / `ros2 launch` 实际去
`install/sim_robot/lib/sim_robot/` 找可执行文件。

**症状**：`colcon build` 明明成功，运行时却报 `No executable found`。

**解决**：包内自带 `build.sh`，它在 `colcon build` 之后补了一步复制。**以后改代码一律用它重编**：

```bash
bash ~/wheeltec_ros2/src/sim_robot/build.sh
```

---

## 3. 文件清单

```
sim_robot/
├── package.xml                  # 包描述（依赖清单）
├── setup.py                     # 安装脚本，定义 4 个可执行入口
├── build.sh                     # ⚠️ 专用编译脚本（含 VM 兼容补丁，见 2.2）
├── resource/sim_robot           # ament 索引占位文件（空文件，必须有）
│
├── sim_robot/                   # ← Python 源码
│   ├── __init__.py
│   ├── sim_robot.py     (658 行) # 【仿真器主体】世界、机器人运动学、激光、深度相机、odom
│   ├── world.py         (11 KB)  # 世界几何定义（墙体 / 圆桶 / 柜子 / 立柱 / 物料箱）
│   ├── depth_avoider.py (283 行) # 【避障节点】ROS 接线层：订阅深度图、发 /cmd_vel
│   ├── depth_decision.py(493 行) # 【决策核心】纯 Python 状态机（decide / AvoidPolicy）
│   ├── explore.py                # 独立探索节点
│   └── save_map.py               # 把地图存成 .pgm / .yaml
│
├── launch/
│   ├── sim_depth.launch.py (13 KB) # 【主启动】避障 / 建图 全靠它，参数见第 6 节
│   ├── sim_slam.launch.py          # 单独的 SLAM 启动
│   └── sim_nav.launch.py           # Nav2 导航启动
│
├── config/
│   ├── slam_params.yaml         # slam_toolbox 参数
│   └── sim_nav_params.yaml      # Nav2 参数
│
├── rviz/sim_depth.rviz          # RViz 预置配置（注意 Fixed Frame 的坑，见 8.7）
├── urdf/robot.urdf              # 车体模型（0.3×0.3×0.2 方块 + 激光雷达）
└── maps/README.md               # 说明地图存放位置
```

### 3.1 四个可执行入口（setup.py 里定义）

| 命令 | 对应文件 | 作用 |
|---|---|---|
| `ros2 run sim_robot sim_robot` | `sim_robot.py` | 启动仿真世界 |
| `ros2 run sim_robot depth_avoider` | `depth_avoider.py` | 启动避障节点 |
| `ros2 run sim_robot explore` | `explore.py` | 启动探索节点 |
| `ros2 run sim_robot save_map` | `save_map.py` | 保存地图 |

### 3.2 话题一览（由 `sim_robot.py` 定义）

**发布：**

| 话题 | 类型 | 频率（实测） | 说明 |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | ~4.8 Hz | 里程计（仿真里是**真值**，零漂移） |
| `/scan` | `sensor_msgs/LaserScan` | ~5.3 Hz | 激光雷达，240 线，最大 5 m |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | ~5.0 Hz | 深度图，**编码 `32FC1`（米）** |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | — | 彩色图（纯色块、零纹理） |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | — | 深度点云 |
| `/world_markers` | `visualization_msgs/MarkerArray` | — | RViz 里的世界可视化 |
| `/camera/rgb/camera_info`<br>`/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | — | 相机内参 |

**订阅：** `/cmd_vel`（`geometry_msgs/Twist`）

**避障节点额外发布：**

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `Twist` | 速度指令，10 Hz |
| `/obstacle_detected` | `std_msgs/Bool` | 当前是否检测到障碍 |
| `/stuck` | `std_msgs/Bool` | 是否判定为卡死 |

---

## 4. 快速上手

### 4.1 三种运行模式

```bash
source ~/wheeltec_ros2/install/setup.bash

# ① 只跑「深度相机自动避障」（最常用）
ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=false use_slam:=false

# ② 激光雷达建图（slam_toolbox）
ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=false use_slam:=true

# ③ 深度相机建图（RTAB-Map）
ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=true use_slam:=false \
  rtabmap_db:=/home/wheeltec/.ros/rtabmap_demo.db
```

> ⚠️ **`use_rtabmap` 和 `use_slam` 绝不能同时为 `true`。**
> 两者都要发布 `map → odom` 变换，同时开必定互相打架，表现为地图乱跳、TF 报错。
> 注意 `use_rtabmap` 的**默认值是 true**，所以想只跑避障必须显式写 `use_rtabmap:=false`。

### 4.2 怎么确认它真的在跑

```bash
# ① 看启动日志（应该出现这一行）
#    depth_avoider 已启动：深度相机避障，安全距离 0.6m，视场 120°，检测扇区 ±26°

# ② 看它有没有在发速度指令（应该稳定 10 Hz）
ros2 topic hz /cmd_vel

# ③ 看决策日志（避障 / 探索 / 卡死自救 都会打）
#    前方过近→避障(turn) 锁定右转 (左0.54/右0.54)
#    【探索模式】在原地打转 → 改朝"没走过"的方向开 (第 1 次，目标方位300°)
```

### 4.3 关掉后台进程（标准配方）

```bash
# ① 先看后杀（-a 显示完整命令行，-f 匹配整条命令行）
pgrep -af "ros2 launch sim_robot|lib/sim_robot/|rviz2 -d|robot_state_publisher"

# ② 优雅退出：先杀 launch 父进程，子节点会跟着退
pkill -f "ros2 launch sim_robot"
pkill -f "lib/sim_robot/sim_robot"
pkill -f "lib/sim_robot/depth_avoider"
pkill -f robot_state_publisher
pkill -f "rviz2 -d"

# ③ 等 5 秒确认
sleep 5; pgrep -af "sim_robot|depth_avoider|rviz2" || echo "✅ 已清空"

# ④ 清 DDS 共享内存残片
rm -f /dev/shm/fastrtps_*
```

**三条铁律：**

- **绝不** `pkill python3` / `pkill ros2` —— 会误杀虚拟机里所有 Python 程序 / 所有 ros2 命令。
- **别一上来 `-9`**：`SIGTERM` 能让 launch 优雅收尾（含释放 DDS 共享内存）；
  `-9` 暴力砍会留下 `/dev/shm/fastrtps_*` 残片，下次启动报怪错。
- **`rtabmap` 在跑时绝对不能用 `-9`** —— 它正在写 `.db` 数据库，强杀可能损坏几百 MB 的建图成果。

还有一个容易踩的：**`pgrep` / `pkill` 默认只匹配被内核截断到 15 字符的进程名**
（`robot_state_publisher` → `robot_state_pub`），所以想按完整命令行匹配**必须加 `-f`**。
另外 `pkill` 是**静默失败**的 —— 名字不匹配它不报错，只是返回码非 0，所以务必先 `pgrep -af` 确认名单。

---

## 5. 核心原理

### 5.1 两条独立链路（最重要的一张图）

```
避障（反应式，不需要地图）：
  深度相机 ──► depth_avoider ──► /cmd_vel ──► 车动起来
                （只看当前一帧）

建图（记忆式，产出地图）：
  激光雷达 ──► slam_toolbox ──► /map ──► Nav2 用
  深度相机 ──► RTAB-Map    ──► /rtabmap/map（3D 点云 + 2D 栅格，给人看）
```

**这两条链路互不依赖。** 关掉建图，避障照样工作。

### 5.2 深度图是怎么变成「距离剖面」的

深度图是一张 H×W 的二维数组，**每一列对应机器人正前方一个水平方向**。
取每列的最小值，就得到一条「地面前方各方向的距离剖面」：

```python
valid = np.isfinite(arr) & (arr > 0.0)   # ① 先剔掉无效像素
plane = np.where(valid, arr, np.inf)     # ② 置成 +inf
row = plane.min(axis=0)                  # ③ 再取列最小
row = row[::-1]                          # ④ 左右翻转，对齐角度约定
```

**这四步的顺序非常关键，每一步都对应一次真实故障：**

| 步骤 | 不做会怎样 |
|---|---|
| ① 有效性过滤 | 真机/仿真都会用 `0` 表示"没测到"。画面下半部分"看地面"的无效像素全是 0，直接取最小会把整列压成 0 —— **连这一列明明打到的墙也一起被吃掉，那一列彻底失明** |
| ② 置 `+inf` 而非大数 | 决策模块的 `is_valid()` 会把 `inf` 当无效过滤掉；整列都没测到时，扇区统计会回退成"很空"，语义正好 |
| ④ 左右翻转 | 画面第 0 列是**车体左侧**，而决策模块的约定是第 0 列 = **车体右侧**。不翻转的话，日志里说"锁定左转"其实是右转 |

### 5.3 决策状态机 `AvoidPolicy`（在 `depth_decision.py`）

核心思路：**一旦选定转向方向就锁定，持续转，不因为噪声抖动而左右反复翻**。

```
检测正前方扇区（±26°）的最小距离
  ├─ 距离 < safe_distance (0.6m) ──► 转向避障
  │     ├─ 左侧更空 → 锁定左转；右侧更空 → 锁定右转
  │     ├─ 距离 < near_distance (0.3m) ──► 一边倒车一边转
  │     └─ 同一方向转满 hold_sec (8s) 仍无出路 ──► 换边 + 倒车 escape_sec (1.5s)
  └─ 前方安全 keep_sec (1.5s) ──► 忘掉锁定方向，恢复直行
```

**关键设计：转向时是「纯原地转」（`lin = 0`），绝不"边转边走"。**
`turn_drift` 参数默认 `0.0`。这一点见 8.4。

### 5.4 三层自救机制

车如果只会"撞了才躲"，钻进死胡同就只会两端掉头，永远出不来。所以做了三层兜底：

| 层 | 名称 | 触发条件 | 动作 | 相关参数（5 个 / 5 个 / 4 个） |
|---|---|---|---|---|
| 第一层 | **转向脱困** | 前方过近 | 锁定方向纯原地转；转满 8s 无出路则换边 + 倒车 | `escape_speed` `hold_sec` `escape_sec` `keep_sec` `near_distance` |
| 第二层 | **探索逃生** | 往复检测：15s 窗口内轨迹包围盒 < 3m → 判定"在原地打转" | 朝没走过、走得通的方向开一段（最简版 frontier 探索） | `explore` `explore_win` `explore_bbox` `explore_sec` `explore_cool` |
| 第三层 | **卡死看门狗** | 订阅 `/odom`：连续下令移动 1.5s，但**既没位移、又没转角** | 强制倒车 + 转向 2s 挣脱 | `stall_sec` `stall_dist` `stall_yaw` `stall_escape_sec` |

> 第三层是**通用兜底**，不依赖"深度相机有没有看到障碍" —— 贴墙、幽灵障碍、轮胎打滑都能兜住。
> 注意：`/odom` 超过 1s 没收到时，看门狗会**自动关闭**，避免"没有里程计"被误判成"卡死"。

第二层探索是这样选方向的：`explore.py` 风格的 `_fresh_dir()` —— 在候选方向里挑
"深度值最大（最空）且最近没走过"的那个。

---

## 6. 参数表

### 6.1 `depth_avoider` 的 20 个参数

用 `ros2 param set /depth_avoider <名字> <值>` 可以**动态生效，无需重启节点**（`drive()` 每轮都重读）。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `safe_distance` | 0.6 | 正前方多近开始转向（米） |
| `forward_speed` | 0.35 | 直行速度（米/秒） |
| `turn_speed` | 1.0 | 转向角速度（弧度/秒） |
| `camera_fov` | 2.094 | 相机水平视场（弧度 ≈120°），**必须与 `sim_robot` 的 `depth_hfov` 一致** |
| `front_angle` | 0.45 | 正前方检测扇区半角（弧度 ≈26°） |
| `escape_speed` | 0.15 | 脱困倒车速度（米/秒） |
| `hold_sec` | 8.0 | 同一方向转多久无出路 → 换边 |
| `escape_sec` | 1.5 | 换边后倒车秒数 |
| `keep_sec` | 1.5 | 前方持续安全多久才忘掉锁定方向 |
| `near_distance` | 0.30 | 比这更近就一边倒车一边转 |
| `explore` | True | 探索总开关 |
| `explore_win` | 15.0 | 往复检测窗口（秒） |
| `explore_bbox` | 3.0 | 窗口内轨迹包围盒小于此值 → 判定打转 |
| `explore_sec` | 8.0 | 单次探索持续秒数 |
| `explore_cool` | 30.0 | 两次探索之间的冷却 |
| `turn_drift` | 0.0 | 转向时叠加的前进速度系数。**0.0 = 纯原地转** |
| `stall_sec` | 1.5 | 看门狗考察窗口 |
| `stall_dist` | 0.02 | 窗口内位移小于此值 → 可能卡死 |
| `stall_yaw` | 0.12 | 窗口内累计转角小于此值才算卡死 |
| `stall_escape_sec` | 2.0 | 判定卡死后强制脱困时长 |

**两条参数约束（源码里有校验）：**

1. `front_angle` 不能大于 `camera_fov / 2` —— 否则会读到相机范围外的无效区，表现为"该躲不躲"。
   源码会在越界时打一条 warn。
2. `turn_drift` 强烈建议保持 `0.0` —— 见 8.4。

### 6.2 `sim_depth.launch.py` 的 launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `launch_rviz` | `false` | 是否开 RViz |
| `use_rtabmap` | **`true`** | 是否启动 RTAB-Map（注意默认是开的） |
| `use_slam` | `false` | 是否启动 slam_toolbox |
| `fresh_db` | `true` | 是否清空旧的 RTAB-Map 数据库 |
| `rtabmap_db` | `~/.ros/rtabmap_sim.db` | RTAB-Map 数据库路径 |
| `rgb_texture` | `true` | 是否给点云上色 |
| `rtabmap_proximity` | `true` | ⚠️ **实际无效** —— 见 9.2 |
| `rtabmap_opt_error` | `3.0` | 位姿图优化最大误差阈值 |
| `grid_ray_tracing` | `true` | 栅格光线追踪 |
| `laser_max` | `5.0` | 激光最大测距（米） |
| `camera_forward` | `0.12` | 相机相对车头往前偏移（米） |
| `camera_height` | `0.30` | 相机离地高度（米） |
| `camera_tf_offset` | `true` | 是否额外发一份相机 TF |

### 6.3 `sim_robot` 的参数（20 个，节选常用）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `rate` | 30.0 | 主循环频率 |
| `depth_width` / `depth_height` | 160 / 100 | 深度图分辨率 |
| `depth_hfov` / `depth_vfov` | 2.094 / 0.785 | 水平 / 垂直视场（弧度） |
| `depth_max` | 8.0 | 深度相机最大测距 |
| `depth_rate` | 10.0 | 深度图发布频率 |
| `laser_rays` / `laser_max` | 240 / 5.0 | 激光线数与量程 |
| `robot_radius` | 0.15 | 机器人半径 |

> ⚠️ **参数归属陷阱**：`laser_max` 在 `sim_robot` 的 `__init__` 里只读一次，
> 运行时 `ros2 param set` **改不动**；而且 launch 里另传了 `'laser_max': '5.0'` 会覆盖节点默认值。
> **改参数前先确认"这个参数到底谁说了算"**（节点默认值 / launch 传参 / 运行时 set）。

---

## 7. 实测成果

> 以下数据全部在虚拟机内实测，产物图见 `docs/images/`。

### 7.1 三种模式实测数据

| 指标 | ① 深度避障 | ② 激光建图 | ③ 深度建图 |
|---|---|---|---|
| `/cmd_vel` | **10.0 Hz** ✅ | — | — |
| `/scan` | — | 5.3 Hz | — |
| `/map` | — | **0.5 Hz** | — |
| 地图尺寸 | — | **324×241 格 = 16.2 × 12.1 m** | 386×278 格 |
| 房间真值 | 16.2 × 12.2 m | ← 吻合 ✅ | — |
| 占用 / 空闲 / 未知 | — | 3.2% / 49.5% / 47.4% | 17.90% / 16.83% / **65.28%** |
| 点云 | — | — | **116,732 点（带 RGB）** |

### 7.2 关键结论：深度相机不适合建 2D 栅格图

| | 激光建的图 | 深度相机建的图 |
|---|---|---|
| 尺寸 | 324×241 = 16.2×12.1 m | 386×278 = 19.2×19.35 m |
| 占用率 | 3.2% | **22.4%** |
| 占用包围盒 | **16.2 × 12.0 m**（与真值吻合） | 18.2 × 18.1 m（虚胖） |
| 肉眼看 | 房间是干净的 16×12 长方形 + 4 个圆桶 | 噪点糊成一片，看不出轮廓 |

**原因**：激光是 360° 平面扫描，一圈就是一条干净的墙线；深度相机是 **120° 锥形视野**，
点云投影到地面会互相覆盖成噪点。

👉 **分工原则：激光建 2D 栅格图给 Nav2 用；深度相机做避障 + 3D 点云。不要混用。**
（对比图：`docs/images/compare_pgm.png`）

### 7.3 SLAM 漂移问题的定位与修复

**症状**：`slam_toolbox` 建图时 `map → odom` 剧烈抖动，yaw 在 −150° ~ +158° 之间乱跳。

**判据**：用 `map → odom` 的**摆幅**（不是终点差值）来量化。

| 组 | 配置 | `map→odom` 摆幅（60 s） |
|---|---|---|
| A 基线 | `use_scan_matching: true` | **15.94 × 16.80 m** |
| B | `use_scan_matching: false` | 0.00 × 0.00 m |
| C | `use_scan_barycenter: false` | 16.50 × 17.11 m（排除重心嫌疑） |
| **E 修复后** | 射线角度加 `self.yaw`，仍开匹配 | **0.11 × 0.25 m** ✅ |

**真凶**：`sim_robot.py` 第 213 行，生成激光射线角度时 `angs` **漏加了机器人自身朝向 `self.yaw`**。
A 组和 E 组只差这一件事，结果差 **390 倍**。

修复后漂移量：**0.041 m / 路程 16.61 m = 0.2%**。

**排除掉的两个错误猜想**（避免后人重走）：
- `/scan` 与系统时钟只差 4 ms → 不是时间戳问题
- 无返回光束仅 15% → 不是数据质量问题

### 7.4 深度相机建图的正确做法

RTAB-Map 是视觉 SLAM，而仿真 RGB 是**纯色块、零纹理**，日志自己会报
`a wrong loop closure has been detected`（误差比 3.689 > 阈值 3.0）。调参后漂移
2.218 m → 1.639 m，**仍然漂** —— 因为问题不在参数，在它的**角色定位**。

**正解**：位姿交给 `/odom`（仿真里它就是真值），点云只负责刷地图。自研 `depth_mapper`：

- 订阅 `/camera/depth/points` + TF，涂 6 cm 3D 体素
- 实测：**64,204 格，x −6.81~7.95（14.76 m）、y −6.13~6.11（12.24 m）、z 0.03~3.21 m**
  （z 范围 = 房间高度 3.20 m ✅）
- **`map → odom` 漂移 0.000 m**，点云不重影

**教训**：`depth_width` 从 160 提到 320 后，`sim_robot` CPU 打满 100%、漂移从 0 变成 0.328 m。
**分辨率不是"好看"的关键，位姿精确才是。**

---

## 8. 踩坑记录（重点，全是真金白银换来的）

> 每一条都对应一次真实故障。**改代码前先读这一节**，能省掉大量时间。

### 8.1 ⚠️ 编译必须用 `build.sh`

见 2.2。直接 `colcon build` 会报 `No executable found`。

### 8.2 ⚠️ 深度图无效像素必须先置 `+inf`

见 5.2。直接用 `arr.min(axis=0)` 会让画面下半部分"看地面"的 0 值把整列压成 0，
**那一列彻底失明**，表现为"该躲不躲"。

### 8.3 ⚠️ 深度图必须左右翻转

见 5.2 步骤 ④。不翻转的话，日志里的"锁定左转"其实是右转，
探索模式选方向也会选反。

### 8.4 ⚠️ 转向必须是"纯原地转"（`turn_drift = 0`）

**症状**：车看着在动，但每圈都回到原地，在死胡同里转好久、地图半天不长。

**原因**：旧版 `turn_drift = 0.5`，于是避障时下发
`lin = 0.35 × 0.5 = 0.175`、`ang = 1.0`，转弯半径 `r = |v|/|ω| = 17.5 cm` ——
车在一个极小的圈里打转。

**修复**：`turn_drift = 0.0`（`lin = 0`）。**这是"死循环打转"的头号真凶。**

### 8.5 ⚠️ `hold_sec` 必须 ≥ 转满一整圈的时间

`hold_sec` 是"同一方向转多久仍无出路就换边"的阈值。如果它小于 `2π / turn_speed`，
车会在"离唯一出口还差几度"的地方掉头，来回摆动 = 原地转圈。

- `turn_speed = 1.0 rad/s` → `2π / 1.0 ≈ 6.3 s`
- 所以取 **8.0 s**（× 1.0 rad/s = 458° > 360°），留足余量。

### 8.6 ⚠️ `map_saver_cli` 内部只等 2 秒（时序陷阱）

**症状**：`ros2 run nav2_map_server map_saver_cli -f xxx` 报
`Failed to spin map subscription`，即使话题名对、QoS 也完全匹配。

**原因**：`map_saver_cli` 内部硬编码只等 **2 秒**。而 `/map` 的发布周期是 0.5 Hz（= 2 秒），
一旦撞上这个窗口 + DDS 发现延迟，就失败。

**正解（实测通过）**：用 slam_toolbox 自带的**服务**，它直接写内存里的图，无时序风险：

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/wheeltec/maps/mymap'}}"
# 返回 result=0 即成功
```

**选型判断**：**有服务就用服务，别用可执行文件去订阅话题。**

### 8.7 ⚠️ RViz 3D 视图全黑 —— `Fixed Frame` 的坑

**症状**：RViz 里 3D 视图一片空白，连 1 米参考网格都不显示。

**真因**（试了 7 次才定位）：
- 当 `Fixed Frame = map` 时，**RTAB-Map 的 map 原点不在机器人起点** → 俯视 (0,0) 什么都看不到 → 全黑
- 而 **slam_toolbox 的 map 原点 = 机器人启动位置** → 同一份 RViz 配置立刻能看到点云铺满 ✅

**连参考网格都不显示 = 可视范围问题，不是数据/渲染问题。** 别去查话题。

**顺带一个 `.rviz` 文件里的陷阱**：文件里有两个同名字段

```
      Use Fixed Frame: true     ← 6 空格，显示项的属性，值只能 true/false
    Fixed Frame: map            ← 4 空格，全局坐标系
```

用宽松正则 `Fixed Frame: \S+` 替换会命中**前者**，把属性写成 `base_link`（非法值）。
正解：用 `re.M` + `^    Fixed Frame:` 行首缩进锚定，改完 `grep -an` 回读验证。

### 8.8 ⚠️ `/map` 是僵尸话题

`ros2 topic list` 里能看到 `/map`、`ros2 topic info` 却说 `Unknown topic`
→ 这是 `ros2 daemon` 的**列表缓存残留**，不是真有发布者。

**关键区分**：RTAB-Map **根本不发 `/map`**，它的 2D 图发在 **`/rtabmap/map`**；
只有 `slam_toolbox` 才发 `/map`。存图时话题名搞错就会失败。

清缓存：`ros2 daemon stop && ros2 daemon start`。

### 8.9 ⚠️ 算力满载导致 TF 丢帧

`slam_toolbox` 刷
`Message Filter dropping message: ... timestamp is earlier than all the data in the transform cache`。

**原因**：CPU 满载（`sim_robot` 跑到 99.9%）导致处理延迟超过 TF 缓存窗口。
**不影响地图完整性**（地图最终覆盖全房间），只影响精细度。想缓解就降分辨率或降频率。

### 8.10 ⚠️ 虚拟机截图 / 操作的坑（供调试用）

- `vmrun captureScreen` 会把桌面**降采样**（2358×1360 → 1080×648），字看不清。
  正解是客户机内 `export DISPLAY=:0 && xwd -root -silent` 抓原始帧。
  **漏 `DISPLAY` 时 `xwd` 输出 0 字节且不报错。**
- 用 `vmrun runProgramInGuest` 执行命令**不回传 stdout** → 一律先
  `命令 > /tmp/out.txt`，再用 `copyFileFromGuestToHost` 取回来读。

---

## 9. 已知问题 / 未完成

### 9.1 深度相机建 2D 栅格图效果差

不是 bug，是**物理限制**（120° 锥形视野 vs 360° 平面扫描）。见 7.2。
**不要试图"调参让它变好"，直接改用激光建图。**

### 9.2 `rtabmap_proximity` 参数实际无效

`sim_depth.launch.py` 里声明了 `rtabmap_proximity` 这个 launch 参数，
但节点里该值被**硬编码成 `'false'`**，所以这个参数是摆设。**已知未修。**

### 9.3 仿真 RGB 零纹理导致视觉回环必然失败

仿真场景是纯色块。已在 launch 里主动关掉视觉回环
（`Rtabmap/DetectionRate=0.0`、`RGBD/ProximityBySpace=false`），
理由是：**仿真 `/odom` 是真值、零漂移，视觉回环只会把准的位姿拉歪。**

> 但要注意：这条理由**只在仿真成立**。真车的 `/odom` 会漂，回环检测从"负资产"变成"必需品"。

### 9.4 真车迁移未实施

见第 10 节。

---

## 10. 真机（实车）迁移 —— 未做，但已调研完

### 10.1 好消息：实车驱动全都是现成的

虚拟机镜像 `~/wheeltec_ros2/src/` 下已有一整套，**不用另装**：

| 目录 / 文件 | 作用 |
|---|---|
| `turn_on_wheeltec_robot/launch/turn_on_wheeltec_robot.launch.py` | 实车总启动 |
| `turn_on_wheeltec_robot/launch/base_serial.launch.py` | 底盘串口驱动 |
| `turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py` | 相机驱动（**默认选中 `Astra_S`**） |
| `turn_on_wheeltec_robot/launch/wheeltec_lidar.launch.py` | 激光 |
| `wheeltec_robot_rtab/` | 实车版 RTAB-Map 建图 / 导航 |
| `wheeltec_robot_slam/`、`wheeltec_robot_nav2/` | 实车版 SLAM / Nav2 |
| `ros2_astra_camera/` | 奥比中光 Astra 相机 ROS2 驱动 |

### 10.2 ⚠️ 三个必须改的地方

**① 【致命】深度图编码不匹配 → 车一步都不动，且不报错**

- 实车 Astra 相机输出：`16UC1`（**16 位整数，单位毫米**）
  依据 `astra_camera/src/ob_camera_node.cpp:228-229`
- 而 `depth_avoider.py:143` 只接受 `32FC1 / 32FC2 / 32FC4`（32 位浮点，米）→ 直接 `return`

**后果链**：`has_depth` 永远 False → `drive()` 一直发零速 → **车完全不动，不崩不报错**，
只在终端刷 `不支持的深度图编码: 16UC1`，极易误判成底盘故障。

**改法**：让 `depth_cb` 同时兼容两种编码（`16UC1` 时除以 1000 转米）。
注意本镜像里 **`depth_image_proc` 没装**（`ros2 pkg list` 查不到），
所以不能用标准的 `convert_metric` 节点绕过去。

**② 水平视场差一倍**

| | 仿真 | 实车 Astra |
|---|---|---|
| `camera_fov` | 2.094 rad（**120°**） | ≈1.019 rad（**58.4°**） |

`rel_angles()` 靠这个参数把"第几列"换算成"哪个方向"，**不改则左右判断全乱**。

顺带注意：`front_angle` 默认 0.45 rad，而实车 `fov/2 = 0.51 rad` —— 勉强不触发越界警告，
但已经是临界了。

**③ 左右翻转约定要实测**

`depth_avoider.py:163` 的 `row = row[::-1]` 是为仿真相机的列序写的（仿真第 0 列 = 车体左侧）。
真机不一定同序 → **实测方法**：在车左侧放个箱子，看日志说"锁定左转"还是"右转"。

### 10.3 仿真 vs 实车 对照表

| 项 | 仿真 | 实车 |
|---|---|---|
| 深度话题 | `/camera/depth/image_raw` | **同名** ✅ |
| 深度编码 | `32FC1`（米） | **`16UC1`（毫米）** ❌ |
| 深度分辨率 | 320×200 | 640×480 @30fps |
| 水平视场 | 120° | 58.4° |
| RGB 话题 | `/camera/rgb/image_raw` | **`/camera/color/image_raw`**（rgb→color） |
| 相机装配 | 前 0.12 / 高 0.30 | 前 0.190 / 高 0.220 / rpy=0 |
| 相机 frame_id | `base_link` | `camera_link` |
| `/odom` | 真值、零漂移 | 编码器+IMU，**会漂** |
| 底盘 | 直接收 `/cmd_vel` | 走串口（`base_serial`） |
| `depth_align` | — | `false`（硬对齐关）→ 跑 RTAB-Map 需注意 |

### 10.4 虚拟机连实车的网络问题

**ROS2 不需要填对方 IP** —— 它靠 DDS **多播自动发现**。只要满足三条硬前提：

| 前提 | 说明 |
|---|---|
| **同一二层网络** | 双方同网段 + `ping` 通。多播跨不了路由器/NAT |
| **同一 `ROS_DOMAIN_ID`** | 未设置 = 0（出厂车通常也是 0） |
| **`ROS_LOCALHOST_ONLY` ≠ 1** | 为 1 时只在本机发现，跨机永远看不到 |

**实测到的现状（2026-09-27）**：

| 检查项 | 结果 |
|---|---|
| VMware 网络模式 | `bridged`（桥接）✅ 模式是对的 |
| 虚拟机 IP | `ens33 192.168.0.136/24`，网关 `192.168.0.1`（手工静态，档案名"静态ip"） |
| 虚拟机连通性 | ping 网关 / 宿主机 / 公网 **全部 100% 丢包**，DNS 失败 → **网络孤岛** |
| 宿主机 WLAN | 连着校园网 `GUET-WiFi`（`10.34.124.217/17`），**不在实车网段** |
| 宿主机有线网卡 | Disabled |
| 实车 `192.168.0.100` | ping 不通（车没开机 / 没连） |

**根因**：虚拟机早就按实车参数配好了（桥接 + 静态 `192.168.0.136` + 网关 `192.168.0.1`），
但宿主机 WiFi 连的是校园网 `10.34.x`，两边不同网段 → 虚拟机等于插了一根没接设备的网线。

**三条连通路线**：

1. **代码直接部署到车上跑**（最稳）—— 车本身就是一台完整 Ubuntu + 全套驱动，
   `ssh wheeltec@192.168.0.100` 把代码拷过去跑，**根本不需要跨机 DDS**。
2. **有线同网段** —— 网线把电脑和车接进同一路由器/直连。⚠️ 宿主机有线网卡要先插网线启用。
3. **无线桥接** —— 宿主机 WiFi 连车的热点。⚠️ **VMware 桥接无线网卡时 DDS 多播常转发不全**，
   会出现 `ping` 通但 `ros2 topic list` 看不到对方。这是无线桥接的已知限制，不是配置错。

**连上后的六层递进验证**：

```bash
ping -c 3 192.168.0.100                          # ① 物理层
ros2 topic list                                  # ② DDS 发现
ros2 topic info /camera/depth/image_raw          # ③ 有没有发布者
ros2 topic hz   /camera/depth/image_raw          # ④ 数据在不在流
ros2 topic echo /camera/depth/image_raw --once   # ⑤ 编码对不对（会暴露 16UC1）
ros2 run tf2_tools view_frames                   # ⑥ TF 树全不全
```

**卡在第 ⑥ 层 = 两机时钟不同步** —— 虚拟机挂起后时钟会漂，TF 报 `message too old`。
VMware 里勾选「与主机同步时间」，或 `sudo systemctl restart systemd-timesyncd`。

> ⚠️ `192.168.0.100` 是轮趣出厂惯例，与镜像里那套静态 IP 配置高度吻合 —— 但**未在真车上验证过**。

---

## 11. 命令速查

```bash
# ---- 编译 ----
bash ~/wheeltec_ros2/src/sim_robot/build.sh
source ~/wheeltec_ros2/install/setup.bash

# ---- 运行 ----
ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=false use_slam:=false  # 只避障
ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=false use_slam:=true   # 激光建图
ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=true  use_slam:=false  # 深度建图

# ---- 诊断 ----
ros2 node list                            # 有哪些节点
ros2 topic list                           # 有哪些话题
ros2 topic hz /cmd_vel                    # 频率（避障应该是 10 Hz）
ros2 topic echo /obstacle_detected        # 是否在避障
ros2 param list /depth_avoider            # 列出所有参数
ros2 param set /depth_avoider safe_distance 0.8   # 动态改参（无需重启）

# ---- 建图存档（正解：用服务，不用 map_saver_cli）----
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/wheeltec/maps/mymap'}}"

# ---- 清理进程 ----
pgrep -af "ros2 launch sim_robot|lib/sim_robot/|rviz2 -d"   # 先看
pkill -f "ros2 launch sim_robot"                            # 后杀
rm -f /dev/shm/fastrtps_*                                   # 清 DDS 残片
```

**关键路径**

| 内容 | 位置 |
|---|---|
| 源码 | `~/wheeltec_ros2/src/sim_robot` |
| 编译产物 | `~/wheeltec_ros2/install/sim_robot` |
| 自己建的地图 | `~/maps/` |
| RTAB-Map 数据库 | `~/.ros/rtabmap_sim.db`（几百 MB） |
| 厂商出厂图 | `~/wheeltec_ros2/src/wheeltec_robot_nav2/map/WHEELTEC.pgm` |

---

## 12. 交接清单

### 12.1 已经做完的

- [x] 深度相机自动避障节点（含三层自救：转向脱困 / 探索逃生 / 卡死看门狗）
- [x] 不依赖 Gazebo 的轻量仿真环境（世界 / 运动学 / 激光 / 深度相机 / odom）
- [x] 激光雷达 2D 建图链路（slam_toolbox）
- [x] 深度相机 RTAB-Map 建图链路
- [x] SLAM 漂移问题定位并修复（`sim_robot.py` 射线角度缺 `self.yaw`）
- [x] 三种模式全部实测跑通，数据已存档
- [x] 实车迁移调研（驱动清单 + 三个改造点 + 网络排查）

### 12.2 待办 / 可以继续做的

- [ ] **让 `depth_cb` 兼容 `16UC1`（毫米→米）** ← 真车迁移的**前置必做项**
- [ ] 把 `camera_fov` / `front_angle` 做成真机可配置（默认 58.4° 场景）
- [ ] 实测确认 `row[::-1]` 在真机上的左右约定是否正确
- [ ] 修掉 `rtabmap_proximity` 参数失效的问题（9.2）
- [ ] 真车实跑验证

### 12.3 建议的接手顺序

1. 先按第 2、4 节把仿真跑起来，确认三种模式都能复现；
2. 读 `depth_decision.py`（纯 Python，无 ROS 依赖，最容易看懂）；
3. 再读 `depth_avoider.py`（看它怎么把 ROS 数据喂给决策模块）；
4. 最后读 `sim_robot.py`（仿真器，最复杂，658 行）；
5. 动手做第 12.2 节的第一个待办（编码兼容），这是通往真车的必经之路。

---

## 附：本次开发用到的调试产物

`docs/images/` 下：

| 文件 | 内容 |
|---|---|
| `compare_pgm.png` | **激光 vs 深度 建图并排对比**（最有说服力的一张） |
| `rtab_cloud_3d.png` | 深度相机 3D 点云（真彩色 / 高度着色 / 俯视 三联图） |
| `depth_map_3d.png` | 自研 `depth_mapper` 产出的零漂移 3D 地图 |
| `legend_map.png` | 地图图例标注（讲清"哪块是扫的、哪块是设置的"） |
| `coverage_map.png` | 地图覆盖率可视化 |
| `demo_laser.pgm` / `.yaml` | slam_toolbox 产出的激光地图成品（可直接给 Nav2 用） |

---

*文档整理：CZA13 · 2026-09-27 · 所有数据来自虚拟机内实测*
