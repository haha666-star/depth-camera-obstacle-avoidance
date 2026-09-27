#!/usr/bin/env python3
"""3D 仿真机器人节点（不依赖 Gazebo）。

它扮演"真实小车"的角色，对外暴露的话题和真车一模一样：
  - 订阅 /cmd_vel                 : 来自避障节点(或键盘/Nav2) 的速度指令
  - 发布 /odom                    : 自己靠"轮子"算出来的里程（位置/朝向）
  - 发布 /scan                    : 激光雷达（顺带显示，本次避障不靠它）
  - 发布 /camera/depth/image_raw  : 模拟深度相机，一张 2D 深度图(H×W，32FC1)
  - 发布 /camera/depth/points     : 由深度图反算的 3D 点云（RViz 里看立体场景用）
  - 发布 /world_markers           : 立体房间(地面/墙/障碍)的 3D 模型，RViz 里显示
  - 发布 /tf                      : 坐标变换 odom->base_link->camera

关键点：深度相机是用 world.py 的 3D 射线检测"凭空算"出来的，等价于真实深度相机
（RealSense 等）拍到的画面。所以避障节点看到的接口和真车一致。
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

import sim_robot.world as world


class SimRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')

        # ---- 可调参数（ros2 run 时可用 --ros-args -p 名字:=值 修改）----
        self.declare_parameter('rate', 30.0)            # 控制/里程频率
        self.declare_parameter('scan_rate', 12.0)       # 激光雷达频率（仅显示用）
        self.declare_parameter('laser_rays', 240)       # 激光射线数
        self.declare_parameter('laser_max', 5.0)        # 激光最大测距
        self.declare_parameter('robot_radius', 0.15)    # 机器人半径（碰撞用）
        # ---- 模拟深度相机（3D 版：输出 H×W 的 2D 深度图）----
        self.declare_parameter('depth_width', 160)      # 深度图像素宽（水平采样数，64→160 提升建图精细度）
        self.declare_parameter('depth_height', 100)     # 深度图像素高（竖直采样数，40→100）
        self.declare_parameter('depth_hfov', 2.094)     # 水平视场（弧度，~120°）
        self.declare_parameter('depth_vfov', 0.785)     # 竖直视场（弧度，~45°）
        self.declare_parameter('depth_max', 8.0)        # 深度相机最大测距（米）
        self.declare_parameter('depth_rate', 10.0)      # 深度图发布频率
        self.declare_parameter('camera_height', 0.30)   # 相机离地高度（米）
        self.declare_parameter('camera_forward', 0.12)  # 相机相对车头往前偏移（米）
        # 【v6】彩色相机画面模式：
        #   True  = 世界坐标锚定纹理（默认）——给 RTAB-Map 提特征用，闭环才生效
        #   False = 旧的距离伪彩（近蓝远绿），好看但没纹理，RTAB-Map 会一直报
        #           `Not enough features in images`，地图永远不做闭环修正
        self.declare_parameter('rgb_texture', True)
        # 【v9 A/B 开关】只控制"静态 TF 里写不写装配偏移"，不动传感器本身。
        # true（默认）= TF 如实写 cam_x/cam_z（v7 修正后的行为）；
        # false       = TF 平移写 0（复现 v7 之前 bug3 的行为）。
        # 注意别用 camera_forward/camera_height 设 0 来做这个对照：
        # 那两个值同时决定"射线从哪出发"，把 camera_height 设 0 会让相机贴着地面、
        # 深度图直接废掉 —— 那是"动了传感器"而不是"只动了 TF"，对照不成立。
        self.declare_parameter('camera_tf_offset', True)

        self.rate = self.get_parameter('rate').value
        self.scan_rate = self.get_parameter('scan_rate').value
        self.rays = int(self.get_parameter('laser_rays').value)
        self.laser_max = float(self.get_parameter('laser_max').value)
        self.radius = self.get_parameter('robot_radius').value
        self.depth_w = int(self.get_parameter('depth_width').value)
        self.depth_h = int(self.get_parameter('depth_height').value)
        self.depth_hfov = float(self.get_parameter('depth_hfov').value)
        self.depth_vfov = float(self.get_parameter('depth_vfov').value)
        self.depth_max = float(self.get_parameter('depth_max').value)
        self.depth_rate = self.get_parameter('depth_rate').value
        self.cam_z = float(self.get_parameter('camera_height').value)
        self.cam_x = float(self.get_parameter('camera_forward').value)
        self.rgb_texture = bool(self.get_parameter('rgb_texture').value)
        self.cam_tf_offset = bool(self.get_parameter('camera_tf_offset').value)

        # ---- 2D 占用栅格（用于"机器人会不会撞墙"的碰撞检测）----
        self.grid, self.x0, self.y0, self.res, self.gw, self.gh = world.occupancy_grid(0.05)

        # ---- 机器人状态（在 odom 坐标系下）----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        # ---- 订阅 / 发布 ----
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, 'camera/depth/points', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'world_markers', 10)
        self.caminfo_pub = self.create_publisher(CameraInfo, 'camera/rgb/camera_info', 10)
        self.caminfo_pub2 = self.create_publisher(CameraInfo, 'camera/depth/camera_info', 10)
        self.tf_br = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._send_static_camera_tf()

        self._running = True
        self.context.on_shutdown(self._on_shutdown)

        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self.control_loop)
        self.create_timer(1.0 / self.scan_rate, self.publish_scan)
        self.create_timer(1.0 / self.depth_rate, self.publish_depth)
        self.create_timer(2.0, self.publish_world_markers)   # 立体房间，每 2 秒刷一次（RViz 不掉）
        self.create_timer(1.0, self.publish_camera_info)    # 相机内参(静态)，1Hz 即可

        self.get_logger().info(
            f'sim_robot 已启动(真实车间3D)：'
            f'车间 {world.ROOM_MAX_X*2:.0f}x{world.ROOM_MAX_Y*2:.0f}x{world.ROOM_HEIGHT:.1f}m，'
            f'实体 {len(world.SOLIDS)} 个（墙/养殖桶/养殖池/控制柜/货架/立柱/物料箱），'
            f'深度图 {self.depth_w}x{self.depth_h}，最大测距 {self.depth_max}m')

    # ---------------- 回调 ----------------
    def _on_shutdown(self):
        self._running = False

    def cmd_cb(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    # ---------------- 主控制循环 ----------------
    def control_loop(self):
        if not self._running:
            return
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.rate

        if (now - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self.cmd = Twist()    # 0.5s 没收指令就当停车，防止失控

        v = self.cmd.linear.x
        w = self.cmd.angular.z

        # 差分小车运动学积分（在平面上移动）
        if abs(w) < 1e-4:
            nx = self.x + v * math.cos(self.yaw) * dt
            ny = self.y + v * math.sin(self.yaw) * dt
        else:
            nx = self.x + (v / w) * (math.sin(self.yaw + w * dt) - math.sin(self.yaw))
            ny = self.y - (v / w) * (math.cos(self.yaw + w * dt) - math.cos(self.yaw))
        nyaw = self.yaw + w * dt

        # 碰撞检测：下一位置空闲才移动；卡住时仍允许转向脱困
        if world.circle_free(self.grid, self.x0, self.y0, self.res, nx, ny, self.radius):
            self.x, self.y, self.yaw = nx, ny, nyaw
        else:
            self.yaw = nyaw

        self.publish_odom(v, w)
        self.publish_tf()

    # ---------------- 里程 ----------------
    def publish_odom(self, v, w):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        try:
            self.odom_pub.publish(odom)
        except RuntimeError:
            self._running = False

    # ---------------- TF: odom -> base_link ----------------
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        try:
            self.tf_br.sendTransform(t)
        except RuntimeError:
            self._running = False

    # ---------------- 激光雷达（仅显示用）----------------
    def publish_scan(self):
        if not self._running:
            return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self.rays
        # 【v8 修正·bug4 附带】angle_max 必须 = angle_min + increment*(N-1)。
        # 原来写死 math.pi，与声明的 increment 差一个增量（240 条射线差 0.0262rad），
        # 是 LaserScan 的"自相矛盾"参数，部分消费端会据此算出错误的每束角度。
        scan.angle_max = scan.angle_min + scan.angle_increment * (self.rays - 1)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_rate
        scan.range_min = 0.05
        scan.range_max = self.laser_max

        angs = scan.angle_min + np.arange(self.rays) * scan.angle_increment
        cos_a = np.cos(angs)
        sin_a = np.sin(angs)
        step = self.res
        max_t = self.laser_max
        t = np.arange(0.0, max_t, step)

        # 【v8 关键修正·bug4 —— /scan "没打到" 的语义错了】
        #   原来: ranges 初值 = max_t(=laser_max=5.0)，循环里只有"打到了"才改写，
        #         最后再 np.clip(..., range_min, max_t)。
        #         结果【没打到任何东西的射线也发一个 5.00 的合法读数】。
        #   后果(实测, 150s 单趟建图): 站在房间中央时四面墙都 >5m，全部 240 束
        #         都报 5.00 → slam_toolbox 把"5m 处有障碍"当真，同时把 0~5m 全清成
        #         可通行 → 自由空间越过 x=8 的东墙一路铺到 x=13.45（10124 格，
        #         占全部可通行格的 21.6%），而房间外根本不该有空间。
        #         LaserScan 语义里"没测到"必须是【严格大于 range_max】的值，惯例发 inf。
        #   现在: 初值 inf；只有真正命中的射线才写入距离、加噪声、做 clip。
        #   （本节点避障不依赖 /scan，改动不影响避障链路。）
        ranges = np.full(self.rays, np.inf, dtype=float)
        for i in range(self.rays):
            px = self.x + t * cos_a[i]
            py = self.y + t * sin_a[i]
            occ = world.sample_grid(self.grid, self.x0, self.y0, self.res, px, py)
            idx = np.argmax(occ)
            if occ[idx]:
                d = t[idx] + step
                ranges[i] = d if d > scan.range_min else scan.range_min
        hit = np.isfinite(ranges)
        ranges[hit] += np.random.normal(0.0, 0.01, int(hit.sum()))
        ranges[hit] = np.clip(ranges[hit], scan.range_min, max_t)
        scan.ranges = ranges.tolist()
        try:
            self.scan_pub.publish(scan)
        except RuntimeError:
            self._running = False

    # ---------------- 静态 TF: base_link -> camera ----------------
    def _send_static_camera_tf(self):
        # 【v7 修正】相机相对 base_link 的装配偏移必须如实写进 TF。
        # 之前这里发的是【纯单位变换】（旋转=0 且平移=0），但仿真里相机实际装在
        # 车头前方 cam_x(0.12m)、离地 cam_z(0.30m) 处。RTAB-Map 是按 TF 把
        # "相机系点云"变换到 base_link 的，TF 少了这个偏移 → 整张点云被系统性地
        # 往后拖 0.12m、往下压 0.30m（随车头朝向不同还会在水平方向被抹开 0.12m）。
        # 旋转部分仍然是单位阵 —— 这里必须是单位阵，不能用标准的"光学系"
        # (z 前 / x 右 / y 下)：RTAB-Map 内部会自己做"光学系 → 车体系"的换算，
        # 而仿真的射线本身就是按【前=+x / 左=+y / 上=+z】生成的，两者正好对上。
        # （实测佐证：点云的 z 范围一直落在 0~3.2m 的墙高区间内，若旋转写错会整个立起来。）
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera'
        if self.cam_tf_offset:
            t.transform.translation.x = self.cam_x
            t.transform.translation.y = 0.0
            t.transform.translation.z = self.cam_z
        # else：平移保持 0，就是 v7 之前那份 TF 的行为（bug3），
        #       仅用于 A/B 对照，把"TF 有没有如实写装配偏移"这一个变量单独拎出来。
        t.transform.rotation.w = 1.0
        try:
            self.static_tf.sendTransform(t)
        except RuntimeError:
            self._running = False

    # ---------------- 3D 深度相机 + 点云 ----------------
    def publish_depth(self):
        if not self._running:
            return
        now = self.get_clock().now()

        # 相机在世界里的位置（车头前方一点、离地 cam_z）
        cy_ = math.cos(self.yaw)
        sy_ = math.sin(self.yaw)
        ox = self.x + cy_ * self.cam_x
        oy = self.y + sy_ * self.cam_x
        oz = self.cam_z

        # 给每个像素算一条"看向哪"的光线方向
        # 【v6 修正】两件事一起改对了：
        #   A) 方向不再是"绕光轴反装 180°"：图像上方=抬头、画面左=车体左。
        #   B) 用【真针孔模型】生成射线，和 publish_camera_info 发布的 fx/fy/cx/cy
        #      【严格自洽】。
        #      原来用的是"等角度"分配（每列固定角度增量），而 camera_info 是针孔模型。
        #      RTAB-Map 按针孔反投影，水平方向最大会差 10.5°（2 米处 0.37 米，
        #      8 米远处约 1.5 米）—— 同一面墙在不同帧里被贴到不同位置，
        #      云就被"糊"成一片、还会铺到房间外面去。这就是"点云糊、乱"的根因。
        #      真车上的 RealSense 本来就是针孔相机，所以这么改也和真车对得上了。
        W, H = self.depth_w, self.depth_h
        fx = (W / 2.0) / math.tan(self.depth_hfov / 2.0)
        fy = (H / 2.0) / math.tan(self.depth_vfov / 2.0)
        ccx = (W - 1) / 2.0
        ccy = (H - 1) / 2.0
        UU, VV = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        # 相机系：前 / 左 / 上（画面左 = 车体左；画面上 = 上）
        xl = -((UU - ccx) / fx)
        xu = -((VV - ccy) / fy)
        xf = np.ones_like(UU)
        nrm = np.sqrt(xf * xf + xl * xl + xu * xu)
        fwd, left, up = xf / nrm, xl / nrm, xu / nrm
        # 转到世界坐标（前=车头方向，左=车头左 90°=+y 方向）
        Dx = fwd * cy_ + left * (-sy_)
        Dy = fwd * sy_ + left * (cy_)
        Dz = up
        D = np.stack([Dx.ravel(), Dy.ravel(), Dz.ravel()], axis=1)
        D = D / np.linalg.norm(D, axis=1, keepdims=True)   # 单位化

        # 求每条光线撞到障碍的距离（不含地面/天花板，它们不是障碍）
        # 【v7 关键修正 —— 两个独立的真 bug，均用 190s 真机轨迹复算验证过】
        #   本段代码原来发的深度图，语义与 camera_info【不一致】，导致 RTAB-Map
        #   建出来的点云有一半糊在房间外面（实测：房间外占比 49.7%）。
        #
        #   bug1【像素值语义 ≠ camera_info】
        #        raycast 给的是"沿射线的欧氏距离" range；可 camera_info 是【针孔模型】，
        #        RTAB-Map 会把像素值当成 Z（前方分量）用，反投影出的点是
        #            O + D * (Z * norm),  norm = √(1 + xl² + xu²)
        #        画面正中央 norm=1 没事，到画面【角落】norm 高达 2.02 ——
        #        8 米处的墙会被贴到 16.2 米，整张点云被"撑大"糊出房间。
        #        真机（RealSense）发的深度图本来就是 Z，所以这里改成发 Z = range*fwd。
        #
        #   bug2【没打中 = 8m】
        #        原来把"什么都没撞到"的像素写成 depth_max(8.0)，等于告诉 RTAB-Map
        #        "我在 8 米处看到了东西"。于是越过 3.2m 墙顶、穿过门洞的射线会在
        #        房间外凭空造出一大片假墙（这是离群点的大头）。
        #        真机对无效像素发的就是 0，RTAB-Map 会直接丢弃。
        #
        #   两条一起修完，同一段轨迹复算：房间外占比 49.71% → 0.00%，
        #   点云范围回到 x 12.5m / y 12.2m / z 0.00~3.20m，与房间真值一字不差。
        O = np.array([ox, oy, oz], dtype=float)
        rng_ = world.raycast_batch(O, D, self.depth_max).reshape(H, W).astype(np.float32)
        rng_ += np.random.normal(0.0, 0.01, (H, W)).astype(np.float32)  # 一点点噪声更真实
        rng_ = np.clip(rng_, 0.05, self.depth_max)

        # 命中掩码在 range 空间判定（和上面两家共用，不受编码影响）
        hit = rng_ < (self.depth_max - 0.05)
        hitm = hit.ravel()

        # 1) 发深度图（2D，H×W，32FC1，单位米）——避障节点订阅它
        #    像素值语义 = 前方分量 Z（与 camera_info 的针孔模型【严格一致】）；
        #    没测到的像素 = 0（无效值；避障端 is_valid() 会按"未知/可通行"忽略）
        depths_pub = rng_ * fwd.astype(np.float32)
        depths_pub[~hit] = 0.0
        img = Image()
        img.header.stamp = now.to_msg()
        img.header.frame_id = 'camera'
        img.encoding = '32FC1'
        img.height = H
        img.width = W
        img.is_bigendian = 0
        img.step = W * 4
        img.data = depths_pub.tobytes()
        try:
            self.depth_pub.publish(img)
        except RuntimeError:
            self._running = False

        # 每条射线"命中点"的世界坐标（RGB 纹理和 3D 点云共用同一份计算）
        pts_los = O + D * rng_.ravel()[:, None]

        # 【v6】彩色图里还要画【地面】：真实相机能看到地面，而射线检测只认"障碍物"
        # （地面不是障碍，所以原来那一片全被涂成中性灰 —— 既不像真实画面，
        #   又让 RTAB-Map 少了半张图的可提特征区域）。
        # 只给彩色图用，深度图 / 点云完全不动，所以【不影响避障】。
        pts_rgb = pts_los.copy()
        Dzz = D[:, 2]
        down = (~hitm) & (Dzz < -1e-6)
        tf = -O[2] / np.where(np.abs(Dzz) < 1e-12, 1e-12, Dzz)
        okf = tf > 0.0
        idx = np.where(down & okf)[0]
        if idx.size:
            pts_rgb[idx] = O + D[idx] * tf[idx][:, None]

        # 2) 发"彩色图"，给 RTAB-Map 当 RGB 输入（v6 默认世界坐标纹理）
        #    这里传 range 空间的 rng_：_depth_color() 的"无效=灰"判据是按 range 写的
        self._publish_rgb(now, rng_, pts_rgb, hitm | (down & okf))

        # 3) 反算 3D 点云（只用真命中障碍物的点），用于 RViz 立体显示
        self._publish_cloud(now, pts_los[hitm])

    def _publish_cloud(self, now, pts):
        if pts.size == 0:
            return
        try:
            import struct
            n = pts.shape[0]
            # 手写 PointCloud2（不依赖 sensor_msgs.point_cloud2，Humble 某些环境没有该模块）
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            buf = b''.join(struct.pack('<fff', float(p[0]), float(p[1]), float(p[2])) for p in pts)
            pc = PointCloud2()
            pc.header.stamp = now.to_msg()
            pc.header.frame_id = 'odom'    # 点直接是世界坐标，画在 odom 下
            pc.height = 1
            pc.width = n
            pc.fields = fields
            pc.is_bigendian = False
            pc.point_step = 12
            pc.row_step = 12 * n
            pc.is_dense = True
            pc.data = buf
            self.cloud_pub.publish(pc)
        except Exception as e:
            self.get_logger().warn(f'点云发布失败(不影响避障): {e}', throttle_duration_sec=5.0)

    # ---------------- 彩色图：两种模式（给 RTAB-Map / 真车彩色相机用）----------------
    @staticmethod
    def _hash01(k1, k2, k3, salt):
        """把三个整数"格号" + 一个盐 哈希成 [0,1) 的伪随机数（纯 numpy、可向量化）。

        用它给每个空间格子刷一个固定的"花色"：同一个世界点永远得到同一个颜色。
        """
        h = (k1 * 73856093) ^ (k2 * 19349663) ^ (k3 * 83492791) ^ (salt * 2654435761)
        h &= 0x7FFFFFFF
        h = ((h ^ (h >> 13)) * 1274126177) & 0x7FFFFFFF
        h ^= (h >> 16)
        return (h & 0xFFFF).astype(np.float64) / 65535.0

    def _world_texture(self, pts, hit):
        """按【世界坐标】给画面刷程序化纹理 —— 相当于给墙面/设备"贴上壁纸"。

        为什么必须这样：RTAB-Map 靠图像里的特征点（角点、色块边界）去匹配两帧画面，
        才能做闭环修正。之前喂给它的是"按距离上色的伪彩图"，而同一面墙上的点到相机
        距离几乎相同 → 整面墙一片纯色 → 一个特征点都提不出来 → 日志一直刷
        `Not enough features in images`，地图永远不闭环、跑第二趟不对齐、墙有重影。

        纹理由三层叠加而成，全部只依赖【世界坐标】（与视角、距离无关）：
          1) 0.5m 立方体色块：哈希出伪随机 RGB —— 提供大量唯一"花色"；
          2) 0.5m 格线：格边界压暗 —— 色块交界处产生强边，便于提角点；
          3) 2m 大区块亮度调制：让大片墙面也有明暗差异，避免长直线产生歧义匹配。
        """
        P = pts
        out = np.full((P.shape[0], 3), 110, dtype=np.uint8)   # 未命中(地面/天空) → 中性灰
        flat = np.asarray(hit).reshape(-1).astype(bool)
        if not flat.any():
            return out
        Q = P[flat]

        c = 0.5
        ix = np.floor(Q[:, 0] / c).astype(np.int64)
        iy = np.floor(Q[:, 1] / c).astype(np.int64)
        iz = np.floor(Q[:, 2] / c).astype(np.int64)
        r = 0.22 + 0.78 * self._hash01(ix, iy, iz, 1)
        g = 0.22 + 0.78 * self._hash01(ix, iy, iz, 2)
        b = 0.22 + 0.78 * self._hash01(ix, iy, iz, 3)

        # 2m 大区块亮度调制（非周期）
        jx = np.floor(Q[:, 0] / 2.0).astype(np.int64)
        jy = np.floor(Q[:, 1] / 2.0).astype(np.int64)
        jz = np.floor(Q[:, 2] / 2.0).astype(np.int64)
        amp = 0.70 + 0.60 * self._hash01(jx, jy, jz, 4)

        # 0.5m 格线（格边界 ±6cm 压暗）
        fx, fy, fz = Q[:, 0] / c, Q[:, 1] / c, Q[:, 2] / c
        line = ((np.abs(fx - np.round(fx)) < 0.12) |
                (np.abs(fy - np.round(fy)) < 0.12) |
                (np.abs(fz - np.round(fz)) < 0.12))

        rgb = np.stack([r, g, b], axis=1) * amp[:, None]
        rgb[line] *= 0.18
        out[flat] = (np.clip(rgb, 0.0, 1.0) * 255.0).astype(np.uint8)
        return out

    def _depth_color(self, depths):
        """旧模式：按距离上色（近=蓝、中=青、远=绿，无效=灰）。好看，但没纹理。"""
        t = np.clip((depths - 0.1) / max(self.depth_max - 0.1, 1e-3), 0.0, 1.0)
        R = np.where(t < 0.5, 0.1, 0.1 + (t - 0.5) * 2 * (0.6 - 0.1))
        G = np.where(t < 0.5, 0.4 + t * 2 * (0.9 - 0.4), 0.9)
        B = np.where(t < 0.5, 0.9, 0.9 - (t - 0.5) * 2 * (0.9 - 0.2))
        rgb = np.stack([R * 255, G * 255, B * 255], axis=-1).astype(np.uint8)
        rgb[depths >= (self.depth_max - 0.05)] = (110, 110, 110)
        return rgb

    def _publish_rgb(self, now, depths, pts, hit):
        """发彩色图。默认发【世界纹理】（让 RTAB-Map 能提特征、能闭环）；
        rgb_texture:=false 时退回旧的"距离伪彩"。真车上这里就是真实彩色画面。"""
        H, W = depths.shape
        if self.rgb_texture:
            rgb = self._world_texture(pts, hit).reshape(H, W, 3)
        else:
            rgb = self._depth_color(depths)
        img = Image()
        img.header.stamp = now.to_msg()
        img.header.frame_id = 'camera'
        img.encoding = 'rgb8'
        img.height = H
        img.width = W
        img.is_bigendian = 0
        img.step = W * 3
        img.data = rgb.tobytes()
        try:
            self.rgb_pub.publish(img)
        except RuntimeError:
            self._running = False

    # ---------------- 相机内参（RTAB-Map 必须，告诉它像素怎么对应到真实射线）----------------
    def publish_camera_info(self):
        if not self._running:
            return
        W, H = self.depth_w, self.depth_h
        fx = (W / 2.0) / math.tan(self.depth_hfov / 2.0)
        fy = (H / 2.0) / math.tan(self.depth_vfov / 2.0)
        cx = (W - 1) / 2.0
        cy = (H - 1) / 2.0
        for pub in (self.caminfo_pub, self.caminfo_pub2):
            ci = CameraInfo()
            ci.header.stamp = self.get_clock().now().to_msg()
            ci.header.frame_id = 'camera'
            ci.height = H
            ci.width = W
            ci.distortion_model = 'plumb_bob'
            ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            try:
                pub.publish(ci)
            except RuntimeError:
                self._running = False

    # ---------------- 立体车间（地面/墙/障碍/标签/机器人）的 3D 模型 ----------------
    def publish_world_markers(self):
        if not self._running:
            return
        ma = MarkerArray()
        now = self.get_clock().now()
        Rx0, Rx1 = world.ROOM_MIN_X, world.ROOM_MAX_X
        Ry0, Ry1 = world.ROOM_MIN_Y, world.ROOM_MAX_Y
        Hh = world.ROOM_HEIGHT
        aid = 0

        def add(m):
            nonlocal aid
            m.header.frame_id = 'odom'
            m.header.stamp = now.to_msg()
            m.action = Marker.ADD
            ma.markers.append(m)
            aid += 1

        # 地面（一块很薄的板，浅灰，像车间地坪）
        m = Marker(); m.ns = 'floor'; m.id = aid
        m.type = Marker.CUBE
        m.pose.position.x = (Rx0 + Rx1) / 2.0
        m.pose.position.y = (Ry0 + Ry1) / 2.0
        m.pose.position.z = -0.02
        m.scale.x = (Rx1 - Rx0) + 0.4
        m.scale.y = (Ry1 - Ry0) + 0.4
        m.scale.z = 0.04
        m.color.r, m.color.g, m.color.b = (0.80, 0.82, 0.85); m.color.a = 1.0
        add(m)

        # 所有实体（墙/养殖桶/养殖池/控制柜/货架/立柱/物料箱）
        for s in world.SOLIDS:
            if s['kind'] == 'box':
                m = Marker(); m.ns = 'solid'; m.id = aid
                m.type = Marker.CUBE
                m.pose.position.x = s['cx']; m.pose.position.y = s['cy']
                m.pose.position.z = s['h'] / 2.0
                m.scale.x = s['sx']; m.scale.y = s['sy']; m.scale.z = s['h']
            else:
                m = Marker(); m.ns = 'solid'; m.id = aid
                m.type = Marker.CYLINDER
                m.pose.position.x = s['cx']; m.pose.position.y = s['cy']
                m.pose.position.z = s['h'] / 2.0
                m.scale.x = s['r'] * 2.0; m.scale.y = s['r'] * 2.0; m.scale.z = s['h']
            m.color.r, m.color.g, m.color.b = s['color']; m.color.a = 1.0
            add(m)

            # 有 water 标记的：在顶部加一层半透明蓝色"水面"
            if s.get('water'):
                w = Marker(); w.ns = 'water'; w.id = aid
                w.type = Marker.CYLINDER
                w.pose.position.x = s['cx']; w.pose.position.y = s['cy']
                w.pose.position.z = s['h'] + 0.02
                if s['kind'] == 'box':
                    w.scale.x = s['sx'] * 0.96; w.scale.y = s['sy'] * 0.96
                else:
                    w.scale.x = w.scale.y = s['r'] * 1.96
                w.scale.z = 0.04
                w.color.r, w.color.g, w.color.b = (0.20, 0.60, 0.90); w.color.a = 0.55
                add(w)

            # 文字标签（空字符串不显示），悬浮在物体顶部
            if s.get('label'):
                t = Marker(); t.ns = 'label'; t.id = aid
                t.type = Marker.TEXT_VIEW_FACING
                t.pose.position.x = s['cx']; t.pose.position.y = s['cy']
                t.pose.position.z = s['h'] + 0.35
                t.scale.z = 0.45                      # 字高（米）
                t.color.r, t.color.g, t.color.b = (1.0, 1.0, 1.0); t.color.a = 1.0
                t.text = s['label']
                add(t)

        # -------- 机器人本体（画在 base_link 下，会跟着车走）--------
        # 车身：绿色方块
        rb = Marker(); rb.ns = 'robot'; rb.id = aid
        rb.header.frame_id = 'base_link'
        rb.header.stamp = now.to_msg(); rb.action = Marker.ADD
        rb.type = Marker.CUBE
        rb.pose.position.x = 0.0; rb.pose.position.y = 0.0; rb.pose.position.z = 0.15
        rb.scale.x = 0.4; rb.scale.y = 0.3; rb.scale.z = 0.25
        rb.color.r, rb.color.g, rb.color.b = (0.10, 0.80, 0.30); rb.color.a = 1.0
        ma.markers.append(rb); aid += 1
        # 相机：黄色小方块（在车头前方、离地 cam_z）
        cam = Marker(); cam.ns = 'robot'; cam.id = aid
        cam.header.frame_id = 'base_link'
        cam.header.stamp = now.to_msg(); cam.action = Marker.ADD
        cam.type = Marker.CUBE
        cam.pose.position.x = self.cam_x; cam.pose.position.y = 0.0
        cam.pose.position.z = self.cam_z
        cam.scale.x = 0.1; cam.scale.y = 0.12; cam.scale.z = 0.1
        cam.color.r, cam.color.g, cam.color.b = (0.95, 0.90, 0.20); cam.color.a = 1.0
        ma.markers.append(cam); aid += 1
        # 车头方向：红色箭头（指向 +x，方便一眼看出机器人朝哪）
        arr = Marker(); arr.ns = 'robot'; arr.id = aid
        arr.header.frame_id = 'base_link'
        arr.header.stamp = now.to_msg(); arr.action = Marker.ADD
        arr.type = Marker.ARROW
        arr.pose.position.x = 0.2; arr.pose.position.y = 0.0; arr.pose.position.z = 0.15
        arr.scale.x = 0.4; arr.scale.y = 0.1; arr.scale.z = 0.1   # 长度/箭头宽/箭头高
        arr.color.r, arr.color.g, arr.color.b = (0.95, 0.20, 0.20); arr.color.a = 1.0
        ma.markers.append(arr); aid += 1

        try:
            self.marker_pub.publish(ma)
        except RuntimeError:
            self._running = False


def main(args=None):
    rclpy.init(args=args)
    node = SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
