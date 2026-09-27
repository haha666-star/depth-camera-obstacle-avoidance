#!/usr/bin/env python3
"""深度避障决策（纯 Python，不依赖 ROS / numpy）。

分两层，各司其职：

  1) decide()      —— 单帧、无状态。给一条深度剖面，判断"这一帧该直行还是该转向"。
  2) AvoidPolicy   —— 带记忆的小状态机。把单帧结论变成"可持续执行的驾驶指令"。

===========================================================================
v2 修正：为什么必须要有"锁定方向"这一层（原来"小车贴着墙不动了"的根因）

    深度图每一列对应机器人正前方一个水平角度，所以"左半区最小距离"和
    "右半区最小距离"在机器人正对一面大墙时会【几乎相等】（差别只有 1e-3 米级，
    远小于传感器噪声 0.01 米）。旧代码每一帧都重新比较 left/right 来决定转向，
    于是"谁大谁小"完全由噪声决定：这一帧往左 3.4°、下一帧往右 3.4° —— 原地抖动，
    永远累计不出足够转角。现象：车跑到墙边停住，/cmd_vel 一直在发却不动。

    解法：一旦决定往哪边转，就【锁定方向】持续转下去。
      - 前方变空 → 直行；持续 keep_sec 秒后才忘掉锁定方向。
      - 同一方向累计转 hold_sec 秒还没脱困 → 换边 + 倒车一小段。
===========================================================================
v3 修正：为什么"转圈"要调 hold_sec

    实测（虚拟机真车、真世界）：车被围在 (7.29, 4.40)，各朝向最近障碍是
    0°→0.46m、60°→0.39、110°→0.34、160°→0.31、210°→0.37、
    220~280°→1.0~1.5m（唯一出路）、300°→0.65、350°→0.47。
    唯一出路离车头最远要转 243°，而 hold_sec=5.0s × 0.6rad/s 只有 172°，
    在离出口还差 71° 的地方就掉头 → 来回摆 172°，净位移为 0 = 原地转圈。

    修法：换边周期必须 ≥ 转满一圈（360°）。v3 把 hold_sec 提到 12s（413°）。
    另外补【旋转看门狗】：旧版只在 |lin|>0 时计时，避障时 lin==0 被直接跳过；
    现在"下了移动指令却既没位移、又没转角"才算卡死。
===========================================================================
v4 修正：为什么"车来回重复走走过的路，不去新地方"

    实测轨迹（真世界闭环仿真，250 秒）：车从原点一律【向右直冲 7.5 米】，
    然后被卡在右墙外侧那条窄缝里上下往复，250 秒再没离开过 —— 覆盖率仅 5%。

    根因：纯反应式避障【没有记忆、没有目标】，只会"撞了才躲"。
    它靠深度图看到的那一点信息做局部决策，一旦钻进"两端封闭的窄走廊"，
    就只能在两端掉头，永远绕不出来。这不是参数没调好，是缺少探索能力。

    修法（v4 三件事，全部可开关、可回退）：
      1. 【往复检测】记录最近 explore_win 秒轨迹；若一直被关在 explore_bbox
         米的小框里（=在原地打转），判定"进死胡同了"，触发探索模式。
      2. 【探索模式】朝「explore_range 米内没走过 且 净空 > explore_clear」
         的方向开过去，持续 explore_sec 秒，之后 explore_cool 秒冷却。
      3. 【开阔度选向】原来用"扇区最小深度"比左右谁更空，在窄缝口两者
         都接近 0.6m 分不出来；改用"扇区平均深度"，能区分"窄缝(两侧都是墙,
         均值低)"和"开阔区(均值高)"，避免一头扎进窄缝。
    另把速度提上去（0.15→0.35 m/s）：0.15m/s × 200s 只有 30m，
    在 16×12m 的房间里根本走不完一圈，这也是"感觉一直在原地"的原因。
    转向也提到 1.0 rad/s，hold_sec 随之改为 8s（2π/1.0 ≈ 6.3s，留余量）。

    效果（真世界闭环仿真，250 秒）：覆盖率 5.0% → 12.0%，
    重复率 3.05 → 1.11，终点从右墙变到房间左上角。详见附录/记录。
===========================================================================
v5 修正：为什么"转向前进"必须取消（"在死胡同里转好久"的真凶）

    实测（真车抓 /cmd_vel）：避障时下发 lin=0.175、ang=1.0，
    转弯半径 r = v/ω = 0.175/1.0 = 0.175m —— 只有 17.5 厘米。
    再连采 10 秒 /odom 位置，净位移不到 5 厘米。

    也就是说：它一边"走"一边以 17.5cm 的半径绕圈，绕一圈回到原地。
    现象就是"车一直在动、地图却半天不长"，实际是【在追自己的尾巴】。
    更糟的是在墙外那条只有 0.3m 宽的夹缝里，17.5cm 半径的圆根本转不开
    → 每一步都被碰撞检测挡回 → 位置完全冻结（实测 8 秒一滴不动）。

    根源是 v4 那句 lin = abs(fwd) * 0.5（0.35 × 0.5 = 0.175）。当初加它是为了
    "看起来不像卡住不动"，结果做出了一个更隐蔽的原地打转。

    修法（v5）：避障转向时 lin = 0 —— 纯原地转，转弯半径 0。
    纯转有明确的角速度（turn=1.0 rad/s），看门狗靠"转角在变"不会误判；
    而且纯转不浪费前进距离，转完一圈朝出口直行，路径最省。
    （保留 turn_drift 系数，把它设成 0.5 即可复现旧现象，方便 A/B 对比。）
===========================================================================
v6 修正：实测记录（哪些改了、哪些【没改】）

    本轮把 v5 的"纯原地转"和几个探索参数都做了多样本对照实验
    （6 个随机起点 × 200 秒，深度图 160×20，真源码闭环）：

      配置                          覆盖率          原地空转   待在房间外
      A 弧线转(drift0.5) 原窗口   16.9%±2.3        9.6s      23.7s
      B 纯原地转(drift0.0) 原窗口 15.0%±3.0        9.2s       4.7s
      C 纯原地转 + 快窗口(10/15)  13.4%±5.0       29.5s      18.9s
      D 弧线转   + 快窗口(10/15)  14.9%±4.4       16.7s      21.8s

    1) 【采用】B 的纯原地转：覆盖率与 A 在 1σ 内无差别（15.0 vs 16.9），
       但"被困在房间外那条 0.3m 窄缝"的时间从 23.7s 掉到 4.7s —— 这才是
       "在死胡同里转好久"的真正来源。窄缝宽 0.3m，车半径 0.15m，
       半径 r 的转弯圆要 (r + 0.15) 的宽度才转得开，所以只有 r=0 能过。
    2) 【驳回】把 explore_win 从 15 降到 10（C/D）：覆盖率反而降、原地空转翻 3 倍。
       原因是"探索"本身也靠边转边找路，窗口太短会让它频繁重启探索模式而越转越久。
       所以 explore_win / explore_cool 保持 15 / 30 不改。
    3) 【删除】曾经加过一个"hard_flips>=2 就立刻触发探索"的捷径：实测它
       【从未触发过】（15 秒窗口总是先到），对照组数据一字不差，属于死代码，已删。
===========================================================================
"""

import math


# ---------------------------------------------------------------- 基础工具
def is_valid(d):
    """深度值是否有效（可被当作真实距离使用）。

    真实深度相机常把"测不到"的位置填成 0、NaN 或 inf：
      - 0 或负数：测距失败，不能当"距离很近"处理（否则会急停/乱转）；
      - NaN / inf：会污染速度计算，导致异常输出。
    所以无效值一律按"未知 / 可通行"处理。
    d == d 这一句用来排除 NaN（NaN 不等于自身）。
    """
    return d is not None and d == d and d > 0.0 and d < 1e6


def rel_angles(w, fov):
    """列 j 相对机器人正前方的水平角（弧度，>0 为左）。

    【v6】相机是【标准针孔模型】，所以"列号 → 角度"不是线性的，必须用 atan 算：
        画面第 0 列 = 车体左侧；而深度剖面第 0 列 = 画面第 w-1 列（车体右侧），
        因为 depth_avoider 收到图后把剖面左右翻转了一次
        （让"剖面下标 ↔ 角度"的约定 = 历史约定：下标 0 是右侧）。
    参数：fx = (w/2)/tan(fov/2)、cx = (w-1)/2（与 sim_robot 发布的 camera_info 一致）。
    """
    fx = (w / 2.0) / math.tan(fov / 2.0)
    cx = (w - 1) / 2.0
    return [-math.atan2((w - 0.5 - j) - cx, fx) for j in range(w)]


def sector_min(depth, rel, lo, hi, max_range=15.0):
    """取"相对角度落在 [lo, hi]"区间内所有【有效】深度的最小值。

    没有任何有效值时回退成 max_range（视为很空）。
    """
    vals = [depth[j] for j in range(len(depth)) if lo <= rel[j] <= hi and is_valid(depth[j])]
    return min(vals) if vals else max_range


def sector_mean(depth, rel, lo, hi, cap=6.0):
    """扇区内有效深度的【均值】（截顶到 cap），用来判断"这一侧有多开阔"。

    为什么不能只看最小值：在窄缝口，"最左边"和"最右边"的最小深度可能都是
    0.6m（都是墙），完全分不出哪边是出口；但均值差别很大 —— 窄缝里两侧全是墙
    （均值低），开阔区大部分射线打到远处（均值接近 cap）。v4 用它来避免
    "一头扎进窄缝"。
    """
    vals = [min(depth[j], cap) for j in range(len(depth))
            if lo <= rel[j] <= hi and is_valid(depth[j])]
    return sum(vals) / len(vals) if vals else cap


def min_half(depth, fov, side):
    """取左(side>0)/右(side<0)半区最小【有效】深度，供日志显示。"""
    rel = rel_angles(len(depth), fov)
    vals = [depth[j] for j in range(len(depth))
            if is_valid(depth[j]) and (rel[j] > 0.0 if side > 0 else rel[j] < 0.0)]
    return min(vals) if vals else 999.0


def wrap_pi(a):
    """把角度归一化到 (-pi, pi]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------- 单帧决策
def decide(depth, fov, front, safe, fwd, turn, max_range=15.0):
    """单帧决策：返回 (linear_x, angular_z, obstacle_bool)。无状态，纯函数。

    depth : 长度 w 的序列（米），列 j 对应相对正前方的水平角 rel_angles()[j]
    fov   : 深度相机水平视场（弧度）
    front : 正前方避障检测扇区半角（弧度），须 ≤ fov/2
    safe  : 安全距离（米）
    fwd   : 直行速度（米/秒）
    turn  : 转向角速度（弧度/秒）
    """
    w = len(depth)
    if w == 0:
        return 0.0, 0.0, False
    rel = rel_angles(w, fov)

    # 只看"有效"的深度值（忽略 0 / NaN / inf），无效按"远处/可通行"
    min_front = sector_min(depth, rel, -front, front, max_range)

    if min_front < safe:
        # 朝更空（均值更大）的一侧转，绕开障碍
        turn_dir = 1.0 if sector_mean(depth, rel, 0.0, fov / 2.0) >= \
                          sector_mean(depth, rel, -fov / 2.0, 0.0) else -1.0
        return 0.0, turn_dir * turn, True
    return fwd, 0.0, False


# ---------------------------------------------------------------- 带记忆的策略
class AvoidPolicy:
    """避障状态机：把单帧决策升级成"有记忆、会探索、不抖"的驾驶指令。

    调用方每帧只要喂进深度剖面和 dt，就能拿到 (linear_x, angular_z, obstacle,
    direction, note)；另外用 feed_odom(x, y, yaw) 喂里程计（探索/看门狗需要）。

    可调参数（由节点从 ROS 参数同步进来）：
      safe / fwd / turn / escape_speed  —— 与节点同名参数一一对应
    状态机参数：
      hold_sec  : 同一方向累计转向多少秒还没脱困，就换边并倒车
                  【必须 ≥ 转一圈的时间 = 2π/turn ≈ 6.3s；默认 8.0s】
      escape_sec: 换边后倒车持续多少秒（默认 1.5s）
      keep_sec  : 前方持续安全多少秒，才忘掉锁定方向（默认 1.5s）
      near      : 前方比这还近就"一边倒车一边转"（默认 0.30m）
    看门狗参数（需要外部用 feed_odom 喂真实里程计）：
      stall_sec        : 考察窗口：连续下令移动多少秒（默认 1.5s）
      stall_dist       : 窗口内位移小于此值（默认 0.02m）
      stall_yaw        : 且累计转角小于此值（默认 0.12rad）→ 判定卡死
      stall_escape_sec : 判定卡死后强制"倒车+转向"的时长（默认 2.0s）
    探索参数（v4 新增；explore=False 可整体关掉，退回 v3 行为）：
      explore          : 是否开启"往复检测 → 探索逃生"
      explore_win      : 往复检测窗口（秒，默认 15.0）
      explore_bbox     : 窗口内轨迹包围盒对角线小于此值 → 判定在原地打转（米）
      explore_sec      : 探索模式持续秒数（默认 8.0）
      explore_cool     : 两次探索之间的冷却（秒，默认 30.0）
      visit_cell       : 访问记忆的格边长（米，默认 0.25）
      explore_clear    : 探索方向要求的最小净空（米，默认 1.2）
      explore_range    : 探索方向考察的半径（米，默认 6.0）
    v5 新增：
      turn_drift       : 避障/对准目标时叠加的前进速度系数（默认 0.0 = 纯原地转）
    """

    def __init__(self, safe=0.6, fwd=0.35, turn=1.0, escape_speed=0.15,
                 hold_sec=8.0, escape_sec=1.5, keep_sec=1.5, near=0.30,
                 stall_sec=1.5, stall_dist=0.02, stall_yaw=0.12,
                 stall_escape_sec=2.0, max_range=15.0,
                 explore=True, explore_win=15.0, explore_bbox=3.0,
                 explore_sec=8.0, explore_cool=30.0,
                 visit_cell=0.25, explore_clear=1.2, explore_range=6.0,
                 turn_drift=0.0):
        self.safe = safe
        self.fwd = fwd
        self.turn = turn
        self.escape_speed = escape_speed
        self.hold_sec = hold_sec
        self.escape_sec = escape_sec
        self.keep_sec = keep_sec
        self.near = near
        # 卡死看门狗：下了移动指令却既没位移、又没转角 → 强制脱困
        self.stall_sec = stall_sec
        self.stall_dist = stall_dist
        self.stall_yaw = stall_yaw
        self.stall_escape_sec = stall_escape_sec
        self.max_range = max_range
        # 探索（v4）
        self.explore = explore
        self.explore_win = explore_win
        self.explore_bbox = explore_bbox
        self.explore_sec = explore_sec
        self.explore_cool = explore_cool
        self.visit_cell = visit_cell
        self.explore_clear = explore_clear
        self.explore_range = explore_range
        # v5：避障/对准时叠加的前进速度系数。0.0 = 纯原地转（默认）。
        # 之所以留成参数，是为了能一键复现旧 bug（设 0.5 就是"半径 17.5cm 的小圈"），
        # 也是为了能一键跑 A/B 对照实验（见文件顶部 v6 实测记录）。
        self.turn_drift = turn_drift
        self.reset()

    # -- 状态 --
    def reset(self):
        self.dir = 0            # 锁定的转向方向：+1 左 / -1 右 / 0 未锁定
        self.hold = 0.0         # 当前方向已累计避障(转向)时长
        self.escape = 0.0       # 剩余倒车时长
        self.clear = 0.0        # 前方已连续安全时长
        self.hard_flips = 0     # 累计"转满一圈仍无路→换边"次数（给日志看）
        # --- 看门狗状态 ---
        self.odom_ok = False    # 是否有新鲜的里程计输入（没有就不启用看门狗）
        self._last_xy = None
        self._last_yaw = None
        self.moved = 0.0        # 本轮考察期内累计位移
        self.dyaw = 0.0         # 本轮考察期内累计转角（弧度，取绝对值累加）
        self.drive_t = 0.0      # 本轮考察期内"下令移动"的累计时长
        self.stuck_left = 0.0   # 强制脱困剩余时长
        self.stuck_dir = 0      # 强制脱困的转向方向
        # --- 探索状态（v4）---
        self._hist = []         # 最近 explore_win 秒的位置 (x, y)
        self._seen = {}         # 访问记忆：格 -> 次数
        self._yaw = None        # 最近一次里程计朝向
        self.esc_left = 0.0     # 探索模式剩余时长
        self.esc_cool = 0.0     # 探索冷却剩余时长
        self.esc_w = 0.0        # 探索目标朝向（世界系，弧度）
        self.n_escape = 0       # 累计进入探索模式次数（给日志看）

    def feed_odom(self, x, y, yaw=None):
        """喂一帧里程计（位置 + 可选朝向）。看门狗和探索都靠它。

        yaw 可以不传（老调用方式仍然能用），此时只按位移判卡死、探索模式退化。
        """
        if self._last_xy is not None:
            self.moved += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        if yaw is not None and self._last_yaw is not None:
            self.dyaw += abs(wrap_pi(yaw - self._last_yaw))
        self._last_xy = (x, y)
        if yaw is not None:
            self._last_yaw = yaw
            self._yaw = yaw
        self.odom_ok = True
        # 访问记忆：当前位置所在格 +1
        if self.explore:
            c = self.visit_cell
            k = (int(math.floor(x / c)), int(math.floor(y / c)))
            self._seen[k] = self._seen.get(k, 0) + 1

    def _judge_stall(self, lin, ang, dt):
        """下令移动（平移或旋转）却既没挪窝、又没转向 → 判定卡死，准备强制脱困。

        判据是【两个都没发生】才算卡死 ——
          - 正常避障原地转：moved≈0 但 dyaw 很大 → 不算卡死；
          - 正常直行：dyaw≈0 但 moved 够大 → 不算卡死；
          - 真卡死（顶在死角/幽灵障碍/打滑）：两者都为 0 → 触发。
        """
        if abs(lin) > 0.02 or abs(ang) > 0.05:
            self.drive_t += dt
        else:
            self.drive_t = 0.0
            self.moved = 0.0
            self.dyaw = 0.0
        if self.drive_t >= self.stall_sec:
            if self.moved < self.stall_dist and self.dyaw < self.stall_yaw:
                # 想走却走不动、想转也转不动：九成是被"看不见的东西"卡住了
                self.stuck_left = self.stall_escape_sec
                self.stuck_dir = -self.dir if self.dir != 0 else 1
                self.dir = self.stuck_dir
                self.hold = 0.0
            self.drive_t = 0.0
            self.moved = 0.0
            self.dyaw = 0.0

    # -- 探索：挑一个"没走过 + 走得通"的方向 --
    def _fresh_dir(self, depth, rel, x, y, yaw):
        """在 24 个候选朝向里挑最值得去的那个（世界系角度）。找不到返回 None。

        评分 = 开阔度奖励 - 走过次数惩罚：
          - 方向被挡（净空 < explore_clear）直接淘汰，避免"探索模式撞墙"；
          - 尽量选 explore_range 米内没走过的方向（这就是最简版 frontier 探索）。
        """
        best, best_score = None, -1e9
        n = len(self._seen)
        for k in range(24):
            mid = -math.pi + k * math.pi / 12.0
            rel_mid = wrap_pi(mid - yaw)          # 转成相对车头
            near = sector_min(depth, rel, rel_mid - 0.14, rel_mid + 0.14, self.max_range)
            if near < self.explore_clear:          # 这个方向走不通，跳过
                continue
            vis = 0
            if n:
                for (ix, iy), c in self._seen.items():
                    cx = (ix + 0.5) * self.visit_cell
                    cy = (iy + 0.5) * self.visit_cell
                    d = math.hypot(cx - x, cy - y)
                    if d < 1.0 or d > self.explore_range:
                        continue
                    a = math.atan2(cy - y, cx - x)
                    if abs(wrap_pi(a - mid)) < 0.26:   # 落在这个方向扇区里
                        vis += c
            score = -vis * 1.0 + min(near, 4.0) * 0.5
            if score > best_score:
                best_score, best = score, mid
        return best

    def _check_looping(self, depth, rel, x, y):
        """往复检测：最近 explore_win 秒都在 explore_bbox 米的小框里 → 触发探索。"""
        if not self.explore or self._yaw is None:
            return
        self._hist.append((x, y))
        need = max(2, int(self.explore_win / 0.1))
        if len(self._hist) > need:
            del self._hist[0]
        if self.esc_cool > 0.0 or len(self._hist) < need:
            return
        xs = [q[0] for q in self._hist]
        ys = [q[1] for q in self._hist]
        if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) >= self.explore_bbox:
            return
        d = self._fresh_dir(depth, rel, x, y, self._yaw)
        if d is None:
            return
        self.esc_w = d
        self.esc_left = self.explore_sec
        self.esc_cool = self.explore_cool
        self.n_escape += 1
        self._hist = []
        self.dir = 0
        self.hold = 0.0

    # -- 每帧更新 --
    def update(self, depth, fov, front, dt):
        """返回 (linear_x, angular_z, obstacle, direction, note)。

        note 给日志看：go/clear/turn/flip/back/stuck/explore/nodepth。
        """
        # ---------- 0) 卡死看门狗：优先级最高，强制"倒车 + 转向"挣脱 ----
        if self.stuck_left > 0.0:
            self.stuck_left -= dt
            return (-abs(self.escape_speed), self.stuck_dir * self.turn,
                    True, self.stuck_dir, 'stuck')

        w = len(depth)
        if w == 0:
            return 0.0, 0.0, False, 0, 'nodepth'
        rel = rel_angles(w, fov)
        min_front = sector_min(depth, rel, -front, front, self.max_range)

        # ---------- 0.5) 探索模式：朝"没走过又走得通"的方向开 ----------
        if self.esc_left > 0.0:
            self.esc_left -= dt
            err = wrap_pi(self.esc_w - (self._yaw if self._yaw is not None else 0.0))
            d = 1 if err > 0 else -1
            if abs(err) > 0.35:
                # v5：先【原地】把车头对准目标，对准了再直行。
                # 不要再"边走边转"——那会画出半径 v/ω 的圆（实测 0.175m），
                # 在窄缝里这个圆转不开，位置会被碰撞检测完全冻住。
                # 纯转有角速度变化，看门狗不会误判成"卡死"。
                if min_front < self.near:
                    lin = -abs(self.escape_speed) * 0.8
                else:
                    lin = abs(self.fwd) * self.turn_drift
                return lin, d * self.turn, True, d, 'explore'
            if min_front >= self.safe:
                return self.fwd, max(-self.turn, min(self.turn, err * 1.2)), \
                       False, 0, 'explore'
            # 【v6 实测修出来的死锁】目标方向被挡住时，旧代码返回
            # (lin=0, ang=clamp(err*1.2))。当 err≈0（已经对准目标）时 ang 也≈0，
            # 于是"前进被前方障碍置 0 + 角速度≈0" = 彻底不动。
            # 更糟的是探索分支不经过卡死看门狗，没人来救它 —— 实测真车上
            # 出现过连续 6.0s / 4.4s 的一动不动（日志里表现为"慢转"）。
            # 修法：目标方向被挡住了就干脆【放弃这次探索】，落到下面的常规避障去处理。
            self.esc_left = 0.0
            self.esc_cool = min(self.esc_cool, 2.0)

        if self.esc_cool > 0.0:
            self.esc_cool -= dt

        # ---------- 0.7) 往复检测（只在有里程计时才可能触发）----------
        if self.odom_ok and self._last_xy is not None:
            self._check_looping(depth, rel, self._last_xy[0], self._last_xy[1])

        # ---------- 1) 前方安全：直行 ----------
        if min_front >= self.safe:
            self.clear += dt
            if self.clear >= self.keep_sec:
                # 连续安全够久，说明确实绕出来了，清掉锁定方向与计数器
                self.dir = 0
                self.hold = 0.0
                self.escape = 0.0
                self.hard_flips = 0
            if self.odom_ok:
                self._judge_stall(self.fwd, 0.0, dt)     # 直行时也要盯着"走没走"
            return self.fwd, 0.0, False, self.dir, 'go' if self.dir == 0 else 'clear'

        # ---------- 2) 前方过近：避障转向 ----------
        self.clear = 0.0
        if self.dir == 0:
            # 首次进入避障：用【扇区平均深度】比左右哪边更开阔。
            # 不能用"最小深度"：窄缝口两侧都是 0.6m 分不出来（v4 修正）。
            left_open = sector_mean(depth, rel, front * 0.5, fov / 2.0)
            right_open = sector_mean(depth, rel, -fov / 2.0, -front * 0.5)
            self.dir = 1 if left_open >= right_open else -1
            self.hold = 0.0

        note = 'turn'
        self.hold += dt
        if self.hold >= self.hold_sec:
            # 锁着一个方向【转满整整一圈】还是没找到出路 → 换边 + 直线倒车一段。
            # hold_sec 必须 ≥ 2π/turn，否则会在"离豁口还差一点"时就掉头，
            # 来回摆 = 原地转圈（见文件顶部 v3 说明）。
            self.dir = -self.dir
            self.escape = self.escape_sec
            self.hold = 0.0
            self.hard_flips += 1
            note = 'flip'

        lin = 0.0
        if self.escape > 0.0:
            self.escape = max(0.0, self.escape - dt)
            lin = -abs(self.escape_speed)      # 脱困倒车
            note = 'back'
        elif min_front < self.near:
            lin = -abs(self.escape_speed)      # 贴太近，先退一点再转
            note = 'back'
        elif self.dir != 0 and min_front >= self.near:
            # v5：转向时【不再叠加前进速度】，就原地转。
            # 旧版这里写的是 lin = abs(fwd) * 0.5，等于以 17.5cm 半径绕圈 ——
            # 车看起来在动，其实每次都回到原地（"追自己的尾巴"）。
            # turn_drift 默认 0.0；想复现旧现象把它设成 0.5 即可。
            lin = abs(self.fwd) * self.turn_drift

        if self.odom_ok:
            self._judge_stall(lin, self.dir * self.turn, dt)   # 转着也要盯着"转没转"
        return lin, self.dir * self.turn, True, self.dir, note
