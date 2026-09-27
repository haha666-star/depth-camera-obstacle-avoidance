# 基于深度相机的 ROS2 自动避障（轮趣 WHEELTEC 小车）

一个**不依赖 Gazebo** 的轻量 2D/3D 仿真机器人包，用来学习与验证「深度相机自动避障」，
并可作为真机（轮趣 WHEELTEC 系列）的算法验证环境。

配套的深度相机避障节点 `depth_avoider` 只订阅深度图 `/camera/depth/image_raw`，
只发布速度指令 `/cmd_vel`，**不依赖激光雷达、不依赖 Nav2**，可以整段搬到真车上复用。

---

## 这个仓库里有什么

| 目录 | 内容 |
|---|---|
| `sim_robot/` | **ROS2 功能包本体**（Python，ament_python 构建） |
| `docs/HANDOVER.md` | **交接文档** —— 先读这个，环境、原理、参数、踩过的坑都在里面 |
| `docs/images/` | 实测成果图（激光 vs 深度建图对比、点云、地图等） |

---

## 环境要求

- Ubuntu 22.04 + **ROS 2 Humble**
- 依赖：`rclpy` `sensor_msgs` `nav_msgs` `geometry_msgs` `tf2_ros` `numpy`
- 可选：`slam_toolbox`（激光建图）、`rtabmap_ros`（深度建图）、`nav2_bringup`、`rviz2`
- 本工程在 **VMware 虚拟机（WHEELTEC Ubuntu 22.04 镜像）** 中开发验证

> ⚠️ **编译有坑**：本镜像里 ament_python 包的可执行文件会装到 `install/sim_robot/bin/`，
> 而 `ros2 run` 去 `install/sim_robot/lib/sim_robot/` 找 —— 直接 `colcon build` 会报
> `No executable found`。**请统一用包里自带的 `build.sh`**，它多做了这一步复制。

---

## 快速开始

```bash
# 1. 放进工作空间
cp -r sim_robot ~/wheeltec_ros2/src/

# 2. 编译（必须用自带脚本，不要直接 colcon build）
bash ~/wheeltec_ros2/src/sim_robot/build.sh

# 3. source 并运行：只跑「深度相机自动避障」
source ~/wheeltec_ros2/install/setup.bash
ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=false use_slam:=false
```

正常的话，终端会打印：

```
深度相机避障，安全距离 0.6m，视场 120°，检测扇区 ±26°
```

看到这行就说明节点活着。再看 `/cmd_vel` 有没有 10 Hz 输出，就确认它在动了。

**三种运行模式**（用 launch 参数切换，别同时开两个抢 `/map`）：

| 目的 | 命令 |
|---|---|
| 深度相机自动避障 | `ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=false use_slam:=false` |
| 激光雷达建图（slam_toolbox） | `ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=false use_slam:=true` |
| 深度相机建图（RTAB-Map） | `ros2 launch sim_robot sim_depth.launch.py launch_rviz:=false use_rtabmap:=true use_slam:=false` |

> ⚠️ `use_rtabmap` 和 `use_slam` **不能同时为 true** —— 两者都要抢 `map → odom` 的发布权，
> 同时开必定打架。

---

## 一句话理解这个项目

**避障走深度相机，建图走激光雷达，两者独立、不混用。**

- 避障 = **反应式**：只看当前这一帧深度图 → 直接发 `/cmd_vel`，不需要地图
- 建图 = **记忆式**：把历史观测累积起来 → 产出 `/map` 给 Nav2 用

深度相机是 120° 锥形视野，投影成 2D 栅格会糊成噪点；激光是 360° 平面扫描，一圈就是一条干净的墙线。
所以：**激光建 2D 栅格图给 Nav2；深度相机做避障 + 3D 点云。** 详见 `docs/HANDOVER.md`。

---

## 文档

- **接手的人请先读 [`docs/HANDOVER.md`](docs/HANDOVER.md)** —— 包含文件清单、算法原理、
  参数表、实测数据、踩坑记录、真机迁移注意事项。

---

## 作者

- Git 用户名：CZA13
- 邮箱：3459856511@qq.com
- GitHub：[@haha666-star](https://github.com/haha666-star)

## 许可

MIT
