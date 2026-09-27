#!/usr/bin/env python3
"""深度相机自动避障节点（depth_avoider）。

只订阅深度相机话题 /camera/depth/image_raw（sensor_msgs/Image，32FC1，H×W 二维深度图），
不依赖激光雷达 /scan，也不依赖 Nav2：
  - 二维深度图里"每一列"对应机器人正前方一个水平方向；取每列最小深度，就得到一条
    "地面前方各方向的距离剖面"（等价于深度相机投影到地面）；
  - 【v7】深度图像素值口径 = 针孔模型的 Z（前方分量），无效像素 = 0。
    取列最小前必须先把 0/NaN 置成 +inf，否则底部"看地面"的无效像素会把整列压成 0，
    那一列就彻底失明（详见 depth_cb 里的注释）；
  - 取正前方一个扇区的最小深度，低于安全距离就转向，否则直行；
  - 转向方向由决策状态机 AvoidPolicy 决定：一旦选定就【锁定方向】持续转，
    避免贴墙时被噪声搞得左右反复翻转、原地抖动（详见 depth_decision.py 顶部注释）。
  - 【v5】转向时是【纯原地转】（lin=0），绝不"边转边走"。边转边走会画出
    半径 r=|v|/|ω| 的小圈，旧参数下只有 17.5cm —— 车看着在动，其实每圈都回到
    原地，这就是"在死胡同里转好久、地图半天不长"的真凶。

决策逻辑全部在纯 Python 模块 depth_decision.py 里（decide / AvoidPolicy），
本文件只负责"ROS 输入输出接线"：订阅深度图、发 /cmd_vel。
把 decide/AvoidPolicy 搬到真车即可复用（接真相机 /camera/depth/image_raw）。

它只发 /cmd_vel，不发别的。想看它工作，启动 sim_robot（带深度相机）+ 本节点即可。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from .depth_decision import AvoidPolicy, min_half


class DepthAvoider(Node):
    def __init__(self):
        super().__init__('depth_avoider')

        # ---- 可调参数（ros2 run 时可用 --ros-args -p 名字:=值 修改）----
        self.declare_parameter('safe_distance', 0.6)     # 正前方多近开始转向（米）
        # 【v4 关键】速度不能太慢：0.15m/s × 200s 只有 30m，在 16×12m 的房间里
        # 根本走不完一圈，表现就是"一直在原地附近晃"。提到 0.35m/s。
        self.declare_parameter('forward_speed', 0.35)    # 直行速度（米/秒）
        self.declare_parameter('turn_speed', 1.0)        # 转向角速度（弧度/秒）
        self.declare_parameter('camera_fov', 2.094)      # 深度相机水平视场（弧度，需与 sim_robot 一致，默认 ~120°）
        self.declare_parameter('front_angle', 0.45)      # 正前方避障检测扇区半角（弧度，默认 ~26°）
        # 卡死自救（anti-stuck）：只会原地转向的车遇到死角会陷入"转向循环"，
        # 这里用 AvoidPolicy 状态机——锁定方向持续转，超时才换边 + 倒车脱离。
        self.declare_parameter('escape_speed', 0.15)     # 脱困倒车速度（米/秒，取负值下发）
        # 【v3/v4 关键】换边周期必须 ≥ 转满一整圈的时间（2π/turn），否则会在
        # "离唯一豁口还差几度"的地方掉头，来回摆 = 原地转圈。
        # turn=1.0rad/s 时 2π/1.0 ≈ 6.3s，取 8.0s × 1.0 = 458° > 360°，留足余量。
        self.declare_parameter('hold_sec', 8.0)          # 同一方向转满多久仍无出路 → 换边倒车
        self.declare_parameter('escape_sec', 1.5)        # 换边后倒车持续秒数
        self.declare_parameter('keep_sec', 1.5)          # 前方持续安全多少秒才忘掉锁定方向
        self.declare_parameter('near_distance', 0.30)    # 比这还近就一边倒车一边转（米）
        # 【v4 新增】探索：纯反应式避障"撞了才躲"，钻进死胡同就只会两端掉头，
        # 永远绕不出来（实测 250 秒只覆盖房间 5%，轨迹全挤在右墙的窄缝里）。
        # 这里加"往复检测 → 探索逃生"：发现自己在原地打转，就朝没走过、走得通的
        # 方向开一段。这是最简版的 frontier 探索。
        self.declare_parameter('explore', True)          # 总开关（False = 退回旧行为）
        self.declare_parameter('explore_win', 15.0)      # 往复检测窗口（秒）
        self.declare_parameter('explore_bbox', 3.0)      # 窗口内轨迹包围盒小于此值 → 判定打转（米）
        self.declare_parameter('explore_sec', 8.0)       # 单次探索持续秒数
        self.declare_parameter('explore_cool', 30.0)     # 两次探索之间的冷却（秒）
        # 【v5 关键】避障/对准目标时叠加的前进速度系数。默认 0.0 = 纯原地转。
        # 旧版这里是 0.5，于是避障时下发 lin=0.35*0.5=0.175、ang=1.0，
        # 转弯半径只有 17.5cm —— 车一直在绕小圈，看起来在动，其实每次回到原地
        # （"在死胡同里转好久"的直接原因）。设成 0.5 可以复现旧现象做对比。
        self.declare_parameter('turn_drift', 0.0)
        # 卡死看门狗：订阅 /odom，如果"一直在下移动指令（平移或旋转），车却既没挪位置、
        # 又没转角度"，说明被看不见的东西卡住了（贴墙/幽灵障碍/轮胎打滑），强制倒车挣脱。
        # 这是通用兜底，不依赖"深度相机有没有看到障碍"。
        self.declare_parameter('stall_sec', 1.5)         # 考察窗口：连续下令移动多少秒
        self.declare_parameter('stall_dist', 0.02)       # 窗口内位移小于此值 + 转角也小 → 卡死（米）
        self.declare_parameter('stall_yaw', 0.12)        # 窗口内累计转角小于此值才算卡死（弧度）
        self.declare_parameter('stall_escape_sec', 2.0)  # 判定卡死后强制脱困时长（秒）

        self.safe = self.get_parameter('safe_distance').value
        self.fwd = self.get_parameter('forward_speed').value
        self.turn = self.get_parameter('turn_speed').value
        self.fov = self.get_parameter('camera_fov').value
        self.front = self.get_parameter('front_angle').value

        # 参数约束：检测扇区半角不能大于相机视场的一半，否则会读到相机范围外的无效区，
        # 表现为"该躲不躲"。
        if self.front > self.fov / 2.0:
            self.get_logger().warn(
                f'front_angle({math.degrees(self.front):.0f}°) 大于 camera_fov/2 '
                f'({math.degrees(self.fov / 2.0):.0f}°)：检测扇区超出相机视场，可能该躲不躲！')

        self.policy = AvoidPolicy(
            safe=self.safe, fwd=self.fwd, turn=self.turn,
            escape_speed=self.get_parameter('escape_speed').value,
            hold_sec=self.get_parameter('hold_sec').value,
            escape_sec=self.get_parameter('escape_sec').value,
            keep_sec=self.get_parameter('keep_sec').value,
            near=self.get_parameter('near_distance').value,
            stall_sec=self.get_parameter('stall_sec').value,
            stall_dist=self.get_parameter('stall_dist').value,
            stall_yaw=self.get_parameter('stall_yaw').value,
            stall_escape_sec=self.get_parameter('stall_escape_sec').value,
            explore=self.get_parameter('explore').value,
            explore_win=self.get_parameter('explore_win').value,
            explore_bbox=self.get_parameter('explore_bbox').value,
            explore_sec=self.get_parameter('explore_sec').value,
            explore_cool=self.get_parameter('explore_cool').value,
            turn_drift=self.get_parameter('turn_drift').value,
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, 'obstacle_detected', 10)
        self.stuck_pub = self.create_publisher(Bool, 'stuck', 10)
        # 真车深度相机（如 RealSense）常用 BestEffort + sensor_data 的 QoS 发布；
        # 这里订阅侧显式用 qos_profile_sensor_data（Humble 中等价于 SensorDataQoS），
        # 避免 QoS 不匹配时"静默收不到数据"。
        self.create_subscription(Image, 'camera/depth/image_raw', self.depth_cb,
                                 qos_profile_sensor_data)
        # 里程计：只给"卡死看门狗"用（判断车到底动没动）
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.depth = None          # 最新的深度剖面（1D numpy 数组，单位米）
        self.has_depth = False
        self._odom_stamp = None    # 最近一帧 /odom 的时间（用于判断里程计是否新鲜）
        self._running = True
        self._last_t = self.get_clock().now()
        self.context.on_shutdown(self._on_shutdown)
        self.create_timer(1.0 / 10.0, self.drive)

        self.get_logger().info(
            f'depth_avoider 已启动：深度相机避障，安全距离 {self.safe}m，'
            f'视场 {math.degrees(self.fov):.0f}°，检测扇区 ±{math.degrees(self.front):.0f}°')

    def _on_shutdown(self):
        self._running = False

    # ---------------- 深度图回调 ----------------
    def depth_cb(self, msg: Image):
        if not self._running:
            return
        import numpy as np
        if msg.encoding not in ('32FC1', '32FC2', '32FC4'):
            self.get_logger().warn(f'不支持的深度图编码: {msg.encoding}', throttle_duration_sec=2.0)
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        except Exception:
            return
        # 【v7】先剔掉无效像素（0 / 负 / NaN = 没测到，真机也发 0），再取列最小。
        # 顺序很关键：如果直接 arr.min(axis=0)，画面下半部分"看地面"的无效像素
        # 全是 0，会把整列压成 0 —— 连这一列明明打到的墙（比如 1m）也一起被吃掉，
        # 那一列就彻底失明。先置 +inf 再取最小，才拿得到"该方向最近的障碍"。
        # 置 inf 而不是大数：depth_decision.is_valid() 会把 inf 当无效直接过滤，
        # 所以整列都没测到时，扇区统计会回退成"很空"，语义正好。
        valid = np.isfinite(arr) & (arr > 0.0)
        plane = np.where(valid, arr, np.inf)
        row = plane.min(axis=0)
        # 【v6】相机修好后：画面第 0 列是【车体左侧】。而 depth_decision.rel_angles()
        # 的约定是第 0 列 = -fov/2 =【车体右侧】（它把 >0 当左侧）。
        # 所以这里左右翻转一次，让"剖面下标 j ↔ 实际角度"前后一致。
        # 不改的话，日志里的"锁定左转"其实是右转，_fresh_dir() 选探索方向也会反。
        row = row[::-1]
        self.depth = row.astype(np.float32)
        self.has_depth = True

    # ---------------- 里程计回调（喂给卡死看门狗）----------------
    def odom_cb(self, msg: Odometry):
        if not self._running:
            return
        self._odom_stamp = self.get_clock().now()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # 偏航角：四元数 → yaw（看门狗要靠它区分"原地正常转"和"转都转不动"）
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.policy.feed_odom(p.x, p.y, yaw)

    # ---------------- 主驾驶循环 ----------------
    def drive(self):
        if not self._running:
            return
        cmd = Twist()

        # ---- 时间步：用真实时钟算 dt（状态机按秒计时，不怕定时器抖动）----
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds / 1e9
        self._last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / 10.0

        # 每次循环都重新读取参数，使 `ros2 param set /depth_avoider xxx yyy`
        # 能动态生效（无需重启节点）。
        self.safe = self.get_parameter('safe_distance').value
        self.fwd = self.get_parameter('forward_speed').value
        self.turn = self.get_parameter('turn_speed').value
        self.fov = self.get_parameter('camera_fov').value
        self.front = self.get_parameter('front_angle').value

        if not self.has_depth:
            # 还没收到第一帧深度，先停车等待，避免盲动
            self.cmd_pub.publish(cmd)
            return

        # 把参数同步给状态机（保持 ros2 param set 动态生效）
        self.policy.safe = self.safe
        self.policy.fwd = self.fwd
        self.policy.turn = self.turn
        self.policy.escape_speed = self.get_parameter('escape_speed').value
        self.policy.hold_sec = self.get_parameter('hold_sec').value
        self.policy.escape_sec = self.get_parameter('escape_sec').value
        self.policy.keep_sec = self.get_parameter('keep_sec').value
        self.policy.near = self.get_parameter('near_distance').value
        self.policy.stall_sec = self.get_parameter('stall_sec').value
        self.policy.stall_dist = self.get_parameter('stall_dist').value
        self.policy.stall_yaw = self.get_parameter('stall_yaw').value
        self.policy.stall_escape_sec = self.get_parameter('stall_escape_sec').value
        self.policy.explore = self.get_parameter('explore').value
        self.policy.explore_win = self.get_parameter('explore_win').value
        self.policy.explore_bbox = self.get_parameter('explore_bbox').value
        self.policy.explore_sec = self.get_parameter('explore_sec').value
        self.policy.explore_cool = self.get_parameter('explore_cool').value
        self.policy.turn_drift = self.get_parameter('turn_drift').value

        # 里程计不新鲜（>1s 没收到）就关掉看门狗，避免"没里程计"被误判成"卡死"
        if self._odom_stamp is None or (now - self._odom_stamp).nanoseconds / 1e9 > 1.0:
            self.policy.odom_ok = False

        lin, ang, obs, tdir, note = self.policy.update(
            self.depth.tolist(), self.fov, self.front, dt)

        if note == 'stuck':
            self.get_logger().warn(
                '【卡死看门狗】下令移动但既没位移、又没转角 → 强制倒车+转向脱困 '
                f'(方向={"左" if tdir > 0 else "右"})', throttle_duration_sec=1.0)
        elif note == 'explore':
            self.get_logger().info(
                '【探索模式】在原地打转 → 改朝"没走过"的方向开 '
                f'(第 {self.policy.n_escape} 次，目标方位'
                f'{math.degrees(self.policy.esc_w) % 360:.0f}°)',
                throttle_duration_sec=1.5)
        elif note == 'flip':
            self.get_logger().warn(
                f'【转满一圈仍无出路】换边并倒车 {self.policy.escape_sec:.1f}s 脱困 '
                f'(第 {self.policy.hard_flips} 次，方向改为'
                f'{"左" if tdir > 0 else "右"})', throttle_duration_sec=0.5)
        elif obs:
            side = '左' if tdir > 0 else '右'
            self.get_logger().warn(
                f'前方过近→避障({note}) 锁定{side}转 '
                f'(左{min_half(self.depth, self.fov, +1):.2f}/'
                f'右{min_half(self.depth, self.fov, -1):.2f})',
                throttle_duration_sec=1.0)

        cmd.linear.x = lin
        cmd.angular.z = ang
        st = Bool(); st.data = obs
        self.status_pub.publish(st)
        sk = Bool(); sk.data = (note == 'stuck')
        self.stuck_pub.publish(sk)
        try:
            self.cmd_pub.publish(cmd)
        except RuntimeError:
            self._running = False


def main(args=None):
    rclpy.init(args=args)
    node = DepthAvoider()
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
