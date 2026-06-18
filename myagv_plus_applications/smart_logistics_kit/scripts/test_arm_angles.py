#!/usr/bin/env python3

import argparse
import atexit
from pathlib import Path
import readline
import shlex
import time


ANGLE_TABLE = {
    "zero_position": [0, 0, 0, 0, 0, 0],
    "move_init": [90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
    "pick_init": [5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
    "pick_watch": [94.13, 20.2, -21.88, 0.96, 90.79, 4.57],
    "pick_point2": [-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],
    "place_init": [-93.6, 0.93, 6.24, -0.17, 80, -6.24],
    "place_point2": [-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
    "place_point3": [90.0, 22.5, -12.48, 2.54, 50.27, -0.35],
    "place_point4": [-95.36, 7.03, -22.85, -3.07, 87.89, 1.46],
}


def setup_readline_history():
    history_file = Path.home() / '.test_arm_angles_history'
    try:
        readline.read_history_file(history_file)
    except FileNotFoundError:
        pass
    atexit.register(readline.write_history_file, history_file)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Interactive MechArm270 angle-table test tool.')
    parser.add_argument('--port', default='/dev/ttyACM1')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--speed', type=int, default=50)
    parser.add_argument('--no-wait', action='store_true')
    return parser.parse_args()


def print_help(speed):
    print()
    print('Commands:')
    print('  list                         Show named angle poses')
    print('  <pose_name>                  Send named pose, for example: move_init')
    print('  angles a1 a2 a3 a4 a5 a6     Send custom joint angles')
    print('  a1 a2 a3 a4 a5 a6            Send custom joint angles directly')
    print('  speed <value>                Set speed, current:', speed)
    print('  read                         Read current angles and coords')
    print('  pump_on | on                 Turn suction pump on')
    print('  pump_off | off               Turn suction pump off')
    print('  help                         Show this help')
    print('  quit                         Exit')
    print()


def print_angle_table():
    for name, angles in ANGLE_TABLE.items():
        print(f'{name:14s} {angles}')


def parse_angle_values(values):
    if len(values) != 6:
        raise ValueError('need exactly 6 joint angles')
    return [float(value) for value in values]


def wait_until_stopped(arm):
    time.sleep(0.3)
    while True:
        state = arm.is_moving()
        if state == 0:
            return
        time.sleep(0.1)


def send_angles(arm, angles, speed, wait):
    print(f'Send angles: {angles}, speed={speed}')
    arm.send_angles(angles, speed)
    if wait:
        wait_until_stopped(arm)
        print('Motion finished.')


def pump_on(arm):
    arm.set_basic_output(2, 0)
    arm.set_basic_output(5, 0)
    print('Pump on.')


def pump_off(arm):
    arm.set_basic_output(2, 0)
    arm.set_basic_output(5, 1)
    time.sleep(0.05)
    arm.set_basic_output(2, 1)
    print('Pump off.')


def main():
    args = parse_args()
    setup_readline_history()

    try:
        from pymycobot import MechArm270
    except ImportError:
        print('Missing Python dependency: pymycobot')
        return 1

    arm = MechArm270(args.port, args.baudrate)
    arm.set_fresh_mode(0)
    speed = args.speed

    print(f'Connected MechArm270: port={args.port}, baudrate={args.baudrate}')
    print_help(speed)
    print_angle_table()

    while True:
        try:
            line = input('arm> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0

        if not line:
            continue

        try:
            tokens = shlex.split(line)
            command = tokens[0].lower()

            if command in {'q', 'quit', 'exit'}:
                return 0
            if command in {'h', 'help', '?'}:
                print_help(speed)
                continue
            if command in {'l', 'list'}:
                print_angle_table()
                continue
            if command == 'speed':
                if len(tokens) != 2:
                    print('Usage: speed <value>')
                    continue
                speed = int(tokens[1])
                print(f'Speed set to {speed}')
                continue
            if command == 'read':
                print('angles:', arm.get_angles())
                print('coords:', arm.get_coords())
                continue
            if command in {'pump_on', 'on'}:
                pump_on(arm)
                continue
            if command in {'pump_off', 'off'}:
                pump_off(arm)
                continue
            if command == 'angles':
                angles = parse_angle_values(tokens[1:])
                send_angles(arm, angles, speed, not args.no_wait)
                continue
            if command in ANGLE_TABLE and len(tokens) == 1:
                send_angles(arm, ANGLE_TABLE[command], speed, not args.no_wait)
                continue

            angles = parse_angle_values(tokens)
            send_angles(arm, angles, speed, not args.no_wait)
        except Exception as error:
            print(f'Error: {error}')


if __name__ == '__main__':
    raise SystemExit(main())
