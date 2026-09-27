"""仿真世界（真实车间 3D 版）：一个 16×12×3.2 米的水产养殖车间。

这个文件只负责两件事：
  1) "世界长什么样" —— 用一张统一的 SOLIDS 实体表描述（墙、养殖桶、养殖池、控制柜、
     设备货架、立柱、物料箱），每个实体都带颜色和文字标签；
  2) "从相机发出一条光线，会撞到多远" —— 给 sim_robot 算深度相机用。

完全不碰 ROS，所以可以在本机用纯 Python 单独测试（见 _test_world.py）。

坐标系：x 向右、y 向前、z 向上，单位米；机器人初始站在 (0,0,0)，车头朝 +x。
"""

import math

import numpy as np

# ---------------- 车间外尺寸 ----------------
ROOM_MIN_X = -8.0
ROOM_MAX_X = 8.0
ROOM_MIN_Y = -6.0
ROOM_MAX_Y = 6.0
WALL = 0.2            # 墙厚度（米）
ROOM_HEIGHT = 3.2     # 墙高度（米）
LASER_MAX = 8.0       # 激光雷达最大测距（米，仅供 /scan 显示用）

# 统一实体表。每条记录字段：
#   kind   : 'box'（长方体）或 'cyl'（竖直圆柱）
#   cx,cy  : 水平中心位置（米）
#   sx,sy  : 长方体水平尺寸（米）        —— 仅 box
#   r      : 圆柱半径（米）              —— 仅 cyl
#   h      : 高度（米，从地面 z=0 起）
#   color  : (r,g,b) 0~1 显示颜色
#   label  : 文字标签（空字符串=不显示）
#   water  : True 表示顶部加一层"水面"装饰（半透明蓝），仅视觉、不是障碍
SOLIDS = []

# 几个常用颜色
WALL_COLOR = (0.82, 0.84, 0.88)   # 墙：浅灰白
TANK_COLOR = (0.20, 0.55, 0.85)   # 圆形养殖桶：蓝
POOL_COLOR = (0.15, 0.70, 0.75)   # 方形养殖池：青
CTRL_COLOR = (0.95, 0.55, 0.15)   # 中央控制柜：橙
RACK_COLOR = (0.55, 0.57, 0.62)   # 设备货架：灰
COL_COLOR  = (0.35, 0.37, 0.42)   # 立柱：深灰
CRATE_COLOR = (0.66, 0.46, 0.26)  # 物料箱：棕


def _box(cx, cy, sx, sy, h, color, label, water=False):
    SOLIDS.append(dict(kind='box', cx=cx, cy=cy, sx=sx, sy=sy, h=h,
                       color=color, label=label, water=water))


def _cyl(cx, cy, r, h, color, label, water=False):
    SOLIDS.append(dict(kind='cyl', cx=cx, cy=cy, r=r, h=h,
                       color=color, label=label, water=water))


# ---------------- 四面墙（带门洞，分段画）----------------
def _wall_x(x0, x1, y, label):
    """沿 y 方向的一段墙（水平范围 x0..x1，中心在 y 线上）。"""
    _box((x0 + x1) / 2.0, y, x1 - x0, WALL, ROOM_HEIGHT, WALL_COLOR, label)


def _wall_y(x, y0, y1, label):
    """沿 x 方向的一段墙（竖直范围 y0..y1，中心在 x 线上）。"""
    _box(x, (y0 + y1) / 2.0, WALL, y1 - y0, ROOM_HEIGHT, WALL_COLOR, label)


# 下墙 (y=-6)，中间留一道门洞 x∈[-1.2, 1.2]
_wall_x(-8.0, -1.2, -6.0, '墙(下)')
_wall_x(1.2, 8.0, -6.0, '墙(下)')
# 上墙 (y=+6)，同样留门洞
_wall_x(-8.0, -1.2, 6.0, '墙(上)')
_wall_x(1.2, 8.0, 6.0, '墙(上)')
# 左墙 / 右墙（整段，无门洞）
_wall_y(-8.0, -6.0, 6.0, '墙(左)')
_wall_y(8.0, -6.0, 6.0, '墙(右)')

# ---------------- 圆形养殖桶（左半区，4 个）----------------
for (tx, ty) in [(-5.2, 3.0), (-5.2, -3.0), (-2.8, 3.0), (-2.8, -3.0)]:
    _cyl(tx, ty, 0.9, 1.3, TANK_COLOR, '圆形养殖桶', water=True)

# ---------------- 方形养殖池（右半区，4 个）----------------
_cyl(5.2, 3.0, 1.2, 1.1, POOL_COLOR, '方形养殖池', water=True)   # 用圆柱当圆角池也行，这里统一用圆柱体
_cyl(5.2, -3.0, 1.2, 1.1, POOL_COLOR, '方形养殖池', water=True)
_cyl(2.8, 3.0, 0.9, 1.1, POOL_COLOR, '方形养殖池', water=True)
_cyl(2.8, -3.0, 0.9, 1.1, POOL_COLOR, '方形养殖池', water=True)

# ---------------- 中央控制柜 ----------------
_box(0.0, 4.6, 1.4, 0.8, 1.0, CTRL_COLOR, '中央控制柜')

# ---------------- 设备货架（贴上下墙，各 4 个）----------------
for rx in (-6.5, -4.5, 4.5, 6.5):
    _box(rx, 5.1, 0.6, 1.6, 2.4, RACK_COLOR, '设备货架')
    _box(rx, -5.1, 0.6, 1.6, 2.4, RACK_COLOR, '设备货架')

# ---------------- 立柱（4 根，角落附近）----------------
for (cx, cy) in [(7.3, 5.0), (7.3, -5.0), (-7.3, 5.0), (-7.3, -5.0)]:
    _cyl(cx, cy, 0.12, ROOM_HEIGHT, COL_COLOR, '立柱')

# ---------------- 物料箱（2 个，右后/右前角落）----------------
_box(6.4, 5.0, 0.9, 0.9, 0.6, CRATE_COLOR, '物料箱')
_box(6.4, -5.0, 0.9, 0.9, 0.6, CRATE_COLOR, '物料箱')

# ---------------- 由 SOLIDS 派生出"给程序算距离"的列表 ----------------
AABBS = []   # 所有长方体：[xmin,xmax,ymin,ymax,zmin,zmax]
CYLS = []    # 所有圆柱：(cx, cy, r, h)
FOOTPRINTS = []          # 长方体在地面的矩形占地：(xmin,xmax,ymin,ymax)
CIRCLE_FOOTPRINTS = []   # 圆柱在地面的圆形占地：(cx, cy, r)

for s in SOLIDS:
    if s['kind'] == 'box':
        xmin, xmax = s['cx'] - s['sx'] / 2.0, s['cx'] + s['sx'] / 2.0
        ymin, ymax = s['cy'] - s['sy'] / 2.0, s['cy'] + s['sy'] / 2.0
        AABBS.append((xmin, xmax, ymin, ymax, 0.0, s['h']))
        FOOTPRINTS.append((xmin, xmax, ymin, ymax))
    else:  # cyl
        CYLS.append((s['cx'], s['cy'], s['r'], s['h']))
        # 【重要】圆柱必须按"真圆"写入占用栅格，不能写成外接正方形！
        # 早期版本这里写的是 (cx-r, cx+r, cy-r, cy+r) 外接矩形，而深度相机的射线
        # 检测用的是真圆柱 —— 两套模型不一致，会在矩形"四角"造出一片
        # "相机看不见、底盘却过不去"的幽灵障碍：小车开进去后一直发前进指令却原地不动
        # （就是"小车走了一会儿不动了"）。这里改成真圆，两个模型就对齐了。
        CIRCLE_FOOTPRINTS.append((s['cx'], s['cy'], s['r']))


# ---------------- 3D 射线检测（深度相机的核心）----------------
def raycast_batch(O, D, max_t):
    """批量算一堆光线"最近撞到哪"。

    参数：
      O      : 相机位置，长度 3 的数组 [x,y,z]
      D      : 所有光线的方向，形状 (N,3)，每行一条，必须单位向量
      max_t  : 最大测距（米），超出的当成"什么都没看见"
    返回：
      tmin   : 长度 N 的数组，每条光线撞到障碍的距离；没撞到就是 max_t
    """
    N = D.shape[0]
    tmin = np.full(N, max_t, dtype=float)

    # ---- 长方体（墙 + 箱子）：用"纸板盒"算法，对每条光线逐轴算进出区间 ----
    for (x0, x1, y0, y1, z0, z1) in AABBS:
        bmin = np.array([x0, y0, z0], dtype=float)
        bmax = np.array([x1, y1, z1], dtype=float)
        Ds = np.where(D == 0.0, 1e-12, D)          # 防止除以 0（方向为 0 时给个极小量）
        t1 = (bmin - O) / Ds                       # 每个轴"进入"的参数
        t2 = (bmax - O) / Ds                       # 每个轴"离开"的参数
        tnear = np.minimum(t1, t2)
        tfar = np.maximum(t1, t2)
        t_enter = tnear.max(axis=1)               # 三个轴都进去了才算真正进入盒子
        t_exit = tfar.min(axis=1)
        hit = (t_enter <= t_exit) & (t_exit >= 0.0) & (t_enter < max_t)
        td = np.where(t_enter > 0.0, t_enter, 0.0)  # 相机若在盒子内部，距离算 0
        tmin = np.where(hit & (td < tmin), td, tmin)

    # ---- 圆柱（养殖桶/立柱）：解方程 (水平距离圆心)==半径，并检查高度在区间内 ----
    for (cx, cy, r, h) in CYLS:
        ox, oy, oz = O
        dx, dy, dz = D[:, 0], D[:, 1], D[:, 2]
        a = dx * dx + dy * dy
        b = 2.0 * (dx * (ox - cx) + dy * (oy - cy))
        c = (ox - cx) ** 2 + (oy - cy) ** 2 - r * r
        disc = b * b - 4.0 * a * c
        disc = np.where(disc < 0.0, -1.0, disc)    # 负数 = 不相交
        sq = np.sqrt(np.where(disc < 0.0, 0.0, disc))
        denom = np.where(a < 1e-12, 1e-12, 2.0 * a)
        t0 = (-b - sq) / denom
        t1 = (-b + sq) / denom
        tc = np.where((t0 > 0) & (t0 < t1), t0, t1)  # 取更近的那个交点
        zc = oz + tc * dz                           # 交点高度
        ok = (disc >= 0.0) & (tc > 1e-4) & (tc < max_t) & (zc >= 0.0) & (zc <= h)
        tmin = np.where(ok & (tc < tmin), tc, tmin)

    return tmin


# ---------------- 2D 占用栅格（只给"机器人会不会撞墙"用，投影到地面）----------------
def _fill(grid, x0, y0, res, xmin, xmax, ymin, ymax):
    """把一个矩形标记进栅格地图（占用=True）。"""
    ix0 = int(round((xmin - x0) / res))
    iy0 = int(round((ymin - y0) / res))
    ix1 = int(round((xmax - x0) / res))
    iy1 = int(round((ymax - y0) / res))
    ix0 = max(0, ix0); iy0 = max(0, iy0)
    ix1 = min(grid.shape[1] - 1, ix1); iy1 = min(grid.shape[0] - 1, iy1)
    if ix1 >= ix0 and iy1 >= iy0:
        grid[iy0:iy1 + 1, ix0:ix1 + 1] = True


def _fill_circle(grid, x0, y0, res, cx, cy, r):
    """把一个圆（圆心 cx,cy 半径 r）按"格子中心是否落在圆内"标记进栅格。"""
    ix0 = max(0, int(math.floor((cx - r - x0) / res)))
    ix1 = min(grid.shape[1] - 1, int(math.ceil((cx + r - x0) / res)))
    iy0 = max(0, int(math.floor((cy - r - y0) / res)))
    iy1 = min(grid.shape[0] - 1, int(math.ceil((cy + r - y0) / res)))
    for iy in range(iy0, iy1 + 1):
        dy = (y0 + iy * res) - cy
        for ix in range(ix0, ix1 + 1):
            dx = (x0 + ix * res) - cx
            if dx * dx + dy * dy <= r * r:
                grid[iy, ix] = True


def occupancy_grid(res=0.05):
    """生成一张地面占用栅格（bool 数组），用于机器人碰撞检测。

    返回 (grid, origin_x, origin_y, res, gw, gh)。

    注意：这里的几何必须和 raycast_batch（深度相机）用【同一套】形状，
    否则会出现"相机看不到、底盘过不去"的幽灵障碍。所以长方体填矩形、圆柱填真圆。
    """
    margin = 0.7
    x0 = ROOM_MIN_X - margin
    y0 = ROOM_MIN_Y - margin
    x1 = ROOM_MAX_X + margin
    y1 = ROOM_MAX_Y + margin
    gw = int(round((x1 - x0) / res)) + 1
    gh = int(round((y1 - y0) / res)) + 1
    grid = np.zeros((gh, gw), dtype=bool)

    for r in FOOTPRINTS:
        _fill(grid, x0, y0, res, *r)
    for (cx, cy, r) in CIRCLE_FOOTPRINTS:
        _fill_circle(grid, x0, y0, res, cx, cy, r)
    return grid, x0, y0, res, gw, gh


def sample_grid(grid, x0, y0, res, px, py):
    """给定一批世界坐标，返回是否占用(bool 数组)。边界外一律算占用。"""
    gx = np.round((px - x0) / res).astype(int)
    gy = np.round((py - y0) / res).astype(int)
    inside = (gx >= 0) & (gx < grid.shape[1]) & (gy >= 0) & (gy < grid.shape[0])
    occ = np.ones_like(gx, dtype=bool)
    occ[inside] = grid[gy[inside], gx[inside]]
    return occ


def circle_free(grid, x0, y0, res, x, y, r):
    """判断以(x,y)为圆心、半径 r 的圆是否完全空闲（不压到障碍/墙）。"""
    angs = np.linspace(0.0, 2.0 * np.pi, 14)
    px = x + r * np.cos(angs)
    py = y + r * np.sin(angs)
    return not bool(np.any(sample_grid(grid, x0, y0, res, px, py)))


def to_occupancy_grid_msg(grid, x0, y0, res):
    """把 bool 栅格转成 ROS OccupancyGrid 的 data（占用=100，空闲=0，未知=-1）。"""
    data = np.full(grid.size, -1, dtype=np.int8)
    data[grid.flatten() == False] = 0
    data[grid.flatten() == True] = 100
    return data.tolist()
