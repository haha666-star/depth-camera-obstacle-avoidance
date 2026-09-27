#!/usr/bin/env python3
"""自动漫游建图节点（explore）。

建图阶段不需要人一直手控：这个节点只看 /scan，自己做"扫地机式"避障——
前方有障碍就转向，否则往前走。这样小车会自己把房间跑遍，slam_toolbox 就能画出完整地图。

它只发 /cmd_vel，不发别的。想自己手控时别启动它，改用手柄/键盘即可。
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Explore(Node):
    def __init__(self):
        super().__init__('explore')
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('safe_dist', 0.6)     # 前方多近开始转向
        self.declare_parameter('avoid_angle', 1.2)    # 只看正前方 ±avoid_angle 弧度

        self.fwd = self.get_parameter('forward_speed').value
        self.turn = self.get_parameter('turn_speed').value
        self.safe = self.get_parameter('safe_dist').value
        self.av_ang = self.get_parameter('avoid_angle').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.create_timer(1.0 / 10.0, self.drive)
        self.min_front = 99.0
        self._running = True
        self.context.on_shutdown(self._on_shutdown)
        self.get_logger().info('explore 已启动：扫地机式自动漫游避障')

    def _on_shutdown(self):
        self._running = False

    def scan_cb(self, msg):
        if not self._running:
            return
        angs = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r = np.array(msg.ranges, dtype=float)
        r[~np.isfinite(r)] = msg.range_max
        # 正前方 ±avoid_angle 扇区
        mask = np.abs(angs) <= self.av_ang
        self.min_front = float(np.min(r[mask])) if np.any(mask) else float(msg.range_max)

    def drive(self):
        if not self._running:
            return
        cmd = Twist()
        if self.min_front < self.safe:
            # 前方太近：随机选个方向转（这里固定左转，简单可靠）
            cmd.angular.z = self.turn
            self.get_logger().warn(f'前方 {self.min_front:.2f}m 太近，转向', throttle_duration_sec=1.0)
        else:
            cmd.linear.x = self.fwd
        try:
            self.cmd_pub.publish(cmd)
        except RuntimeError:
            self._running = False


def main(args=None):
    rclpy.init(args=args)
    node = Explore()
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
