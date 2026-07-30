# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import select
import time

import geometry_msgs.msg
import rclpy
from rclpy.node import Node


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


class TeleopKeyboard(Node):
    def __init__(self, name):
        super().__init__(name)

        self.stamped = self.declare_parameter('stamped', True).value
        self.frame_id = self.declare_parameter('frame_id', '').value
        self.speed_limit = self.declare_parameter('speed_limit', 1.6).value
        self.turn_limit = self.declare_parameter('turn_limit', 7.27).value
        self.speed_limit_min = self.declare_parameter('speed_limit_min', 0.1).value
        self.turn_limit_min = self.declare_parameter('turn_limit_min', 0.1).value
        self.speed = clamp(
            self.declare_parameter('speed', 0.25).value,
            self.speed_limit_min, self.speed_limit)
        self.turn = clamp(
            self.declare_parameter('turn', 1.0).value,
            self.turn_limit_min, self.turn_limit)
        self.key_poll_timeout = self.declare_parameter('key_poll_timeout', 0.05).value
        self.zero_publish_interval = self.declare_parameter('zero_publish_interval', 0.15).value

        if self.stamped:
            self.TwistMsg = geometry_msgs.msg.TwistStamped
        else:
            self.TwistMsg = geometry_msgs.msg.Twist

        self.pub = self.create_publisher(self.TwistMsg, 'cmd_vel', 10)
        self.settings = self.get_terminal_settings()

    def get_terminal_settings(self):
        if sys.platform == 'win32':
            return None
        import termios
        return termios.tcgetattr(sys.stdin)

    def restore_terminal_settings(self):
        if sys.platform == 'win32' or self.settings is None:
            return
        import termios
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def getKey(self, timeout):
        if sys.platform == 'win32':
            import msvcrt
            return msvcrt.getwch()
        import tty
        import termios
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self):
        return 'currently:\tspeed %s\tturn %s ' % (self.speed, self.turn)

    def create_twist_msg(self, x, y, z, th):
        twist_msg = self.TwistMsg()
        if self.stamped:
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.frame_id
            twist = twist_msg.twist
        else:
            twist = twist_msg
        twist.linear.x = x * self.speed
        twist.linear.y = y * self.speed
        twist.linear.z = z * self.speed
        twist.angular.z = th * self.turn
        return twist_msg


def main():
    rclpy.init()
    teleop = TeleopKeyboard('teleop_twist_keyboard')

    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0
    last_motion_key_time = 0.0
    motion_active = False

    try:
        print(msg)
        print(teleop.vels())

        while True:
            key = teleop.getKey(teleop.key_poll_timeout)
            now = time.monotonic()
            should_publish = False

            if key in moveBindings:
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                last_motion_key_time = now
                motion_active = True
                should_publish = True
            elif key in speedBindings:
                teleop.speed = clamp(
                    teleop.speed * speedBindings[key][0],
                    teleop.speed_limit_min, teleop.speed_limit)
                teleop.turn = clamp(
                    teleop.turn * speedBindings[key][1],
                    teleop.turn_limit_min, teleop.turn_limit)
                if teleop.speed == teleop.speed_limit:
                    print("Linear speed upper limit reached!")
                elif teleop.speed == teleop.speed_limit_min:
                    print("Linear speed lower limit reached!")
                if teleop.turn == teleop.turn_limit:
                    print("Angular speed upper limit reached!")
                elif teleop.turn == teleop.turn_limit_min:
                    print("Angular speed lower limit reached!")
                print(teleop.vels())
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
                should_publish = True
            else:
                if key == '\x03':
                    break
                if key != '':
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    th = 0.0
                    motion_active = False
                    should_publish = True

            if motion_active and now - last_motion_key_time >= teleop.zero_publish_interval:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                motion_active = False
                should_publish = True

            if should_publish:
                teleop.pub.publish(teleop.create_twist_msg(x, y, z, th))

    except Exception as e:
        print(e)

    finally:
        teleop.pub.publish(teleop.create_twist_msg(0, 0, 0, 0))
        teleop.restore_terminal_settings()
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
