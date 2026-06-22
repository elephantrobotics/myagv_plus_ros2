#!/usr/bin/env python3

import argparse
from pathlib import Path
import sys
import time

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from smart_logistics_kit.wit_usb2can import SerialCANParser, mode_name


GREEN = '\033[32m'
YELLOW = '\033[33m'
RED = '\033[31m'
RESET = '\033[0m'


def read_frame_fast(can_parser, timeout):
    """Read one 0x182 frame without adding a sleep after every byte."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not can_parser.shutdown:
        if can_parser.ser is not None and can_parser.ser.in_waiting > 0:
            frame = can_parser.read_serial_data_once()
            if frame is not None:
                return frame
            continue
        time.sleep(0.001)
    return None


def publish_cmd_vel(rclpy_module, node, cmd_pub, twist_msg, x_speed=0.0, z_speed=0.0):
    if cmd_pub is None:
        return

    msg = twist_msg()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'base_link'
    msg.twist.linear.x = float(x_speed)
    msg.twist.angular.z = float(z_speed)
    cmd_pub.publish(msg)
    rclpy_module.spin_once(node, timeout_sec=0.0)


def main():
    parser = argparse.ArgumentParser(description='Test autocharge USB-CAN serial connection.')
    parser.add_argument('--port', default='/dev/ttyCH341USB1')
    parser.add_argument('--baudrate', type=int, default=9600)
    parser.add_argument('--timeout', type=float, default=1.0)
    parser.add_argument('--read-seconds', type=float, default=9999.0)
    parser.add_argument('--frame-timeout', type=float, default=2.0)
    parser.add_argument('--publish-cmd-vel', dest='publish_cmd_vel', action='store_true', default=True)
    parser.add_argument('--no-publish-cmd-vel', dest='publish_cmd_vel', action='store_false')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()

    rclpy = None
    cmd_pub = None
    twist_msg = None
    if args.publish_cmd_vel:
        import rclpy
        from geometry_msgs.msg import TwistStamped

        rclpy.init()
        node = rclpy.create_node('test_autocharge_can')
        cmd_pub = node.create_publisher(TwistStamped, args.cmd_vel_topic, 10)
        twist_msg = TwistStamped
        print(f'{YELLOW}TwistStamped cmd_vel publishing enabled: {args.cmd_vel_topic}{RESET}')
    else:
        node = None

    can_parser = SerialCANParser(args.port, args.baudrate, args.timeout, args.debug)
    frame_count = 0
    try:
        can_parser.open_serial()
        can_parser.send_at_commands(['AT+CG', 'AT+AT'])
        print(f'{GREEN}USB-CAN handshake succeeded. Reading 0x182 frames...{RESET}')

        deadline = time.monotonic() + args.read_seconds
        while time.monotonic() < deadline:
            frame = read_frame_fast(can_parser, args.frame_timeout)
            if frame is None:
                print(f'{YELLOW}Waiting for valid 0x182 frame...{RESET}')
                continue
            frame_count += 1
            bits = frame['infrared_bits']
            mode = frame['mode']
            print(
                f"frame={frame_count}, mode={mode_name(mode)}, "
                f"x={frame['x_speed']:.3f} m/s, z={frame['z_speed']:.3f} rad/s, "
                f"L_A={bits[2]}, L_B={bits[3]}, R_B={bits[4]}, R_A={bits[5]}, "
                f"infrared_flag={bits[6]}, charging_flag={bits[7]}, "
                f"current={frame['actual_current']:.1f} mA")

            if bits[7] != 0:
                publish_cmd_vel(rclpy, node, cmd_pub, twist_msg)
                print(f'{RED}Charging flag / obstacle stop - stop movement and finish.{RESET}')
                return 0

            if mode == 0x01:
                publish_cmd_vel(
                    rclpy,
                    node,
                    cmd_pub,
                    twist_msg,
                    frame['x_speed'],
                    frame['z_speed'])
                continue

            if mode == 0xBB:
                publish_cmd_vel(rclpy, node, cmd_pub, twist_msg)
                print(f'{YELLOW}Pressure zone - stop movement and keep observing.{RESET}')
                continue

            if mode == 0xAA:
                publish_cmd_vel(rclpy, node, cmd_pub, twist_msg)
                print(f'{GREEN}Charging zone - stop movement and finish.{RESET}')
                return 0

            if mode == 0xCF:
                publish_cmd_vel(rclpy, node, cmd_pub, twist_msg)
                print(f'{RED}Emergency stop mode - stop movement and finish.{RESET}')
                return 3

            publish_cmd_vel(rclpy, node, cmd_pub, twist_msg)
            print(f'{YELLOW}Unknown mode 0x{mode:02X} - publish stop and keep observing.{RESET}')

        if frame_count == 0:
            print(f'{RED}No valid 0x182 frame received. Check charger CAN output and serial wiring.{RESET}')
            return 2
        print(f'{GREEN}Autocharge CAN test finished: {frame_count} valid frames received.{RESET}')
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f'{RED}Autocharge CAN test failed: {error}{RESET}')
        return 1
    finally:
        if cmd_pub is not None:
            publish_cmd_vel(rclpy, node, cmd_pub, twist_msg)
            node.destroy_node()
            rclpy.shutdown()
        can_parser.close_serial()


if __name__ == '__main__':
    sys.exit(main())
