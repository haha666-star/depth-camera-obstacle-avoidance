#!/usr/bin/env python3
"""保存地图节点（save_map）。

slam_toolbox 建图时会在 /map 话题上持续发布 OccupancyGrid。
这个节点订阅 /map，收到第一帧就把它写成 nav2 map_server 能加载的格式：
  - sim_map.pgm  （灰度图：黑=障碍，白=空闲，灰=未知）
  - sim_map.yaml （元数据：分辨率、原点、阈值）

用法：先在建图 launch 跑着的时候，另开一个终端执行
  ros2 run sim_robot save_map
它会自动把地图存到本包 maps/ 目录下并退出。
"""

import sys
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory


class MapSaver(Node):
    def __init__(self, out_basename):
        super().__init__('save_map')
        self.out = out_basename
        self.sub = self.create_subscription(OccupancyGrid, 'map', self.cb, 10)
        self.done = False
        self.get_logger().info(f'等待 /map ... 将保存到 {out_basename}.pgm/.yaml')

    def cb(self, msg):
        if self.done:
            return
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # OccupancyGrid.data: -1 未知, 0 空闲, 100 占用
        pgm_data = bytearray()
        for v in msg.data:
            if v == -1:
                pgm_data.append(205)   # 灰：未知
            elif v >= 50:
                pgm_data.append(0)     # 黑：占用
            else:
                pgm_data.append(254)    # 白：空闲

        with open(self.out + '.pgm', 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            f.write(bytes(pgm_data))

        with open(self.out + '.yaml', 'w') as f:
            f.write(f'image: {self.out.split("/")[-1]}.pgm\n')
            f.write(f'resolution: {res}\n')
            f.write(f'origin: [{ox}, {oy}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')

        self.get_logger().info(f'地图已保存：{self.out}.pgm / .yaml（{w}x{h}, 分辨率 {res}）')
        self.done = True
        # 给一点时间让日志刷出来再退出
        self.destroy_subscription(self.sub)


def main(args=None):
    rclpy.init(args=args)
    # 输出路径：默认存到本包 maps 目录（导航 launch 就在这里找地图）；也可通过参数覆盖
    if len(sys.argv) > 1:
        out = sys.argv[1]
    else:
        try:
            out = os.path.join(get_package_share_directory('sim_robot'), 'maps', 'sim_map')
        except Exception:
            out = 'sim_map'
    node = MapSaver(out)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
