#!/usr/bin/env python3
"""横向里程计测试脚本。

只发布 cmd_vel.linear.y 让小车纯横移指定距离，全程记录 /odom，
停车后打印分析所需数据：起点/终点位姿、odom 全局位移、转回车体系的
纵向/横向位移、yaw 漂移、以及里程计认为走过的横向距离。

用法（先 source 工作区）：
    python3 test_odom_lateral.py --dir left  --dist 0.5
    python3 test_odom_lateral.py --dir right --dist 1.0 --speed 0.08

停车后用卷尺量实际横移距离，连同终端输出一起发我分析缩放/符号。

安全：默认低速 0.08 m/s；到达里程计目标或超时即停；Ctrl-C 立即停车。
"""

import argparse
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import tf2_ros


def yaw_from_quat(q):
    """从四元数取 yaw（绕 z）。"""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class LateralOdomTest(Node):
    def __init__(self, direction, dist, speed, rate_hz, delay, out_path=None,
                 with_tf=False, map_frame='map', odom_frame='odom',
                 base_frame='base_footprint'):
        super().__init__('test_odom_lateral')

        # 测试 B：横移时采样 map->odom（AMCL 修正）和 odom->base_footprint
        self.with_tf = with_tf
        self.map_frame = map_frame
        self.odom_frame = odom_frame
        self.base_frame = base_frame
        self.mo_start = None       # map->odom 起点 (x,y,yaw)
        self.mo_last = None        # map->odom 最新
        self.mo_max_xy = 0.0       # 过程中相对起点最大平移漂移
        self.mo_max_yaw = 0.0      # 过程中相对起点最大角度漂移
        self.tf_buffer = None
        self.tf_listener = None
        if self.with_tf:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sign = 1.0 if direction == 'left' else -1.0  # +y = 左
        self.direction = direction
        self.target = abs(dist)
        self.speed = abs(speed)
        self.dt = 1.0 / rate_hz

        # 超时保护：理论时间的 3 倍再加 5 秒
        self.timeout = (self.target / self.speed) * 3.0 + 5.0
        self.elapsed = 0.0

        self.start = None          # (x, y, yaw)
        self.last = None           # 最新 (x, y, yaw)
        self.max_yaw_dev = 0.0     # 过程中相对起点的最大 yaw 偏差
        self.done = False
        self.moving = False        # 倒计时结束、收到 odom 后才置 True

        # 启动倒计时：拿到第一帧 odom 后才开始数，数完才动
        self.delay = max(0.0, delay)
        self.countdown_left = self.delay
        self.last_announced = None

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f'横移测试: 方向={direction} 目标={self.target:.2f}m '
            f'速度={self.speed:.3f}m/s 超时={self.timeout:.1f}s')
        self.get_logger().info(f'等待 /odom ...（收到后倒计时 {self.delay:.0f}s 再开始横移）')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.last = (p.x, p.y, yaw)
        if self.start is None:
            self.start = (p.x, p.y, yaw)
            self.get_logger().info(
                f'起点  x={p.x:+.4f}  y={p.y:+.4f}  yaw={math.degrees(yaw):+7.2f} deg')

    def body_lateral(self):
        """把 odom 全局位移转回车体系，返回 (纵向 x_body, 横向 y_body)。"""
        sx, sy, syaw = self.start
        lx, ly, _ = self.last
        dx, dy = lx - sx, ly - sy
        c, s = math.cos(syaw), math.sin(syaw)
        x_body = c * dx + s * dy
        y_body = -s * dx + c * dy
        return x_body, y_body

    def on_timer(self):
        if self.start is None or self.done:
            return

        # 倒计时阶段：只数数、不发速度
        if not self.moving:
            sec = int(math.ceil(self.countdown_left))
            if sec != self.last_announced and sec > 0:
                self.get_logger().info(f'{sec} ... 准备横移（Ctrl-C 可取消）')
                self.last_announced = sec
            self.countdown_left -= self.dt
            if self.countdown_left <= 0.0:
                self.moving = True
                self.get_logger().info('开始横移！')
            return

        self.elapsed += self.dt

        # 记录 yaw 漂移
        yaw_dev = abs(self._wrap(self.last[2] - self.start[2]))
        self.max_yaw_dev = max(self.max_yaw_dev, yaw_dev)

        # 测试 B：采样 map->odom（AMCL 在横移时的修正量）
        if self.with_tf:
            self.sample_tf()

        x_body, y_body = self.body_lateral()
        odom_lateral = abs(y_body)  # 里程计认为走过的横向距离

        if odom_lateral >= self.target:
            self.finish('到达里程计目标距离')
            return
        if self.elapsed >= self.timeout:
            self.finish('!! 超时停车（未达目标，请检查）')
            return

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_footprint'
        cmd.twist.linear.y = self.sign * self.speed
        self.pub.publish(cmd)

    def sample_tf(self):
        """读取 map->odom，记录起点、最新值和过程中最大漂移。"""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time())
        except tf2_ros.TransformException:
            return
        x = t.transform.translation.x
        y = t.transform.translation.y
        yaw = yaw_from_quat(t.transform.rotation)
        self.mo_last = (x, y, yaw)
        if self.mo_start is None:
            self.mo_start = (x, y, yaw)
            self.get_logger().info(
                f'map->odom 起点 x={x:+.4f} y={y:+.4f} yaw={math.degrees(yaw):+.2f} deg')
            return
        dxy = math.hypot(x - self.mo_start[0], y - self.mo_start[1])
        dyaw = abs(wrap_pi(yaw - self.mo_start[2]))
        self.mo_max_xy = max(self.mo_max_xy, dxy)
        self.mo_max_yaw = max(self.mo_max_yaw, dyaw)

    def finish(self, reason):
        self.done = True
        # 多发几帧零速确保停住
        for _ in range(10):
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = 'base_footprint'
            self.pub.publish(stop)
        self.report(reason)
        self.timer.cancel()
        rclpy.shutdown()

    def report(self, reason):
        sx, sy, syaw = self.start
        lx, ly, lyaw = self.last
        dx, dy = lx - sx, ly - sy
        x_body, y_body = self.body_lateral()
        euclid = math.hypot(dx, dy)
        yaw_drift = math.degrees(self._wrap(lyaw - syaw))

        lines = [
            '================ 横移里程计测试结果 ================',
            f'时间       : {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            f'停车原因   : {reason}',
            f'方向/目标  : {self.direction}  目标 {self.target:.3f} m'
            f'  速度 {self.speed:.3f} m/s  用时 {self.elapsed:.2f} s',
            '--- odom 全局坐标系 ---',
            f'起点       : x={sx:+.4f}  y={sy:+.4f}  yaw={math.degrees(syaw):+7.2f} deg',
            f'终点       : x={lx:+.4f}  y={ly:+.4f}  yaw={math.degrees(lyaw):+7.2f} deg',
            f'位移 dx,dy : dx={dx:+.4f} m   dy={dy:+.4f} m',
            f'欧氏位移   : {euclid:.4f} m   （= sqrt(dx^2+dy^2)）',
            '--- 转回车体坐标系（去掉起点 yaw）---',
            f'纵向 x_body: {x_body:+.4f} m   （纯横移应 ≈ 0）',
            f'横向 y_body: {y_body:+.4f} m   （{self.direction} 应 {"为正" if self.sign>0 else "为负"}）',
            '--- yaw 稳定性 ---',
            f'yaw 净漂移 : {yaw_drift:+.3f} deg   过程最大偏差 {math.degrees(self.max_yaw_dev):.3f} deg',
        ]

        # 测试 B：map->odom（AMCL 修正）漂移
        if self.with_tf:
            lines.append('--- map->odom（AMCL 修正，横移时应基本不动）---')
            if self.mo_start is None:
                lines.append('!! 没拿到 map->odom：导航/AMCL 没起，或没设初始位姿')
            else:
                ms, me = self.mo_start, self.mo_last
                net_xy = math.hypot(me[0] - ms[0], me[1] - ms[1])
                net_yaw = math.degrees(wrap_pi(me[2] - ms[2]))
                lines += [
                    f'起点 m->o  : x={ms[0]:+.4f} y={ms[1]:+.4f} yaw={math.degrees(ms[2]):+7.2f} deg',
                    f'终点 m->o  : x={me[0]:+.4f} y={me[1]:+.4f} yaw={math.degrees(me[2]):+7.2f} deg',
                    f'净漂移     : 平移 {net_xy*1000:.1f} mm   角度 {net_yaw:+.3f} deg',
                    f'最大漂移   : 平移 {self.mo_max_xy*1000:.1f} mm   角度 {math.degrees(self.mo_max_yaw):.3f} deg',
                    '判读: 平移>20~30mm 或 角度>1~2deg → AMCL 在横移时被拉偏（元凶在 AMCL 层）',
                ]

        lines += [
            '===================================================',
            f'里程计认为横移了 {abs(y_body):.4f} m。',
        ]
        if not self.with_tf:
            lines.append('如需缩放：用卷尺量实际横移距离，缩放比 = 实际 / 该值。')

        log = self.get_logger()
        for ln in lines:
            log.info(ln)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))


def main():
    parser = argparse.ArgumentParser(description='横向里程计测试')
    parser.add_argument('--dir', dest='direction', choices=['left', 'right'],
                        required=True, help='横移方向：left=+y, right=-y')
    parser.add_argument('--dist', type=float, default=0.5,
                        help='目标横移距离（米），默认 0.5')
    parser.add_argument('--speed', type=float, default=0.08,
                        help='横移速度（m/s），默认 0.08')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='发布频率（Hz），默认 20')
    parser.add_argument('--delay', type=float, default=3.0,
                        help='收到 odom 后开始横移前的倒计时（秒），默认 3；设 0 立即开始')
    parser.add_argument('--with-tf', action='store_true',
                        help='测试 B：横移时采样 map->odom，量化 AMCL 修正（需先启动导航并设初始位姿）')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_footprint')
    args = parser.parse_args()

    rclpy.init()
    node = LateralOdomTest(args.direction, args.dist, args.speed, args.rate,
                           args.delay, out_path=None, with_tf=args.with_tf,
                           map_frame=args.map_frame, odom_frame=args.odom_frame,
                           base_frame=args.base_frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl-C：立即停车
        for _ in range(10):
            stop = TwistStamped()
            stop.header.frame_id = 'base_footprint'
            node.pub.publish(stop)
        node.get_logger().info('已手动停车 (Ctrl-C)')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
