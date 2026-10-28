#!/usr/bin/env python3

import time

import numpy as np
from pymycobot import MechArm270

from smart_logistics_kit_lite.QRCodeScanner import (
    QRCodeScanner, PICK_HEIGHT_HIGH, PICK_HEIGHT_LOW)


class MechArm270Control:
    def __init__(
            self,
            port='/dev/ttyACM1',
            baudrate=115200,
            qr_camera='/dev/video1',
            qr_timeout=60.0,
            qr_show_window=True,
            io_client=None):
        self.mc = MechArm270(port, baudrate)
        self.mc.set_fresh_mode(0)
        self.io = io_client
        self.qr_camera = qr_camera
        self.qr_timeout = qr_timeout
        self.qr_show_window = qr_show_window
        self.pick_z_tolerance = 8.0  # 取货吸泵前 Z 轴到位容差，单位 mm；Z tolerance before pump-on, in mm.
        self.pick_reach_timeout = 3.0  # 取货坐标到位重试总时间，单位秒；Total retry window for pickup target, in seconds.
        self.scanner = None

        self.angle_table = {
            "zero_position": [0, 0, 0, 0, 0, 0],
            "move_init": [90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
            "pick_init": [5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
            "pick_watch": [94.13, 30.2, -25.88, 0.96, 85.79, 4.57],
            "pick_point2": [-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],
            "place_init": [-93.6, 0.93, 6.24, -0.17, 80, -6.24],
            "place_point2": [-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
            "place_point3": [90.0, 22.5, -12.48, 2.54, 50.27, -0.35],
            "place_point4": [-95.36, 7.03, -22.85, -3.07, 87.89, 1.46],
        }

    def pump_on(self):
        self.io.set_pump_state(1)

    def pump_off(self):
        self.io.set_pump_state(0)

    def move_angles(self, angles, speed=50):
        self.mc.send_angles(angles, speed)
        self.wait()

    def move_angles_repeat(self, angles, speed=50, count=3):
        for _ in range(count):
            self.mc.send_angles(angles, speed)
            time.sleep(0.05)
        self.wait()

    def move_coords(self, coords, speed=40, mode=1):
        self.mc.send_coords(coords, speed, mode)
        self.wait()

    def move_coords_settle(self, coords, speed=40):
        self.mc.send_coords(coords, speed, 1)
        deadline = time.monotonic() + self.pick_reach_timeout
        last_z = None
        while time.monotonic() < deadline:
            time.sleep(0.3)
            cur = self.mc.get_coords()
            if not isinstance(cur, (list, tuple)) or len(cur) < 3:
                continue
            z = float(cur[2])
            if last_z is not None and abs(z - last_z) < 1.0:
                return
            last_z = z

    def wait(self):
        time.sleep(0.3)
        state = self.mc.is_moving()
        while state != 0:
            state = self.mc.is_moving()
            time.sleep(0.1)

    def get_coords_safe(self):
        coords = self.mc.get_coords()
        while not isinstance(coords, (list, tuple)) or len(coords) < 6:
            time.sleep(0.2)
            coords = self.mc.get_coords()
        return coords

    def get_scanner(self):
        if self.scanner is None:
            self.scanner = QRCodeScanner(
                video_device=self.qr_camera,
                timeout=self.qr_timeout,
                show_window=self.qr_show_window)
        return self.scanner

    def reset_scanner(self):
        if self.scanner is not None:
            try:
                self.scanner.release_resources()
            except Exception as error:
                print(f"[arm] QR scanner release failed: {error}")
            finally:
                self.scanner = None

    def probe_arm(self):
        try:
            angles = self.mc.get_angles()
        except Exception as error:
            return False, f'arm no response: {error}'
        if not angles:
            return False, ('device reports readiness to read but returned no data '
                           '(disconnected or multiple access on port?)')
        return True, 'ok'

    def move_pick_target(self, coords, target_z):
        deadline = None
        actual_coords = None
        attempt = 0
        while deadline is None or time.monotonic() < deadline:
            attempt += 1
            self.move_coords(coords, 20)
            if deadline is None:
                deadline = time.monotonic() + self.pick_reach_timeout
            observe_until = min(deadline, time.monotonic() + 1.0)
            while time.monotonic() < observe_until:
                actual_coords = self.mc.get_coords()
                if isinstance(actual_coords, (list, tuple)) and len(actual_coords) >= 3 and abs(float(actual_coords[2]) - float(target_z)) <= self.pick_z_tolerance:
                    return True, actual_coords
                time.sleep(0.1)
            if time.monotonic() >= deadline:
                break
            print(
                f"[arm] pick target not reached, retry within {self.pick_reach_timeout:.1f}s "
                f"(attempt={attempt}): target_z={target_z:.1f}, actual_coords={actual_coords}")
        return False, actual_coords

    def move_until_z_reached(self, coords, target_z, speed=40, label="move"):
        deadline = None
        actual_coords = None
        attempt = 0
        while deadline is None or time.monotonic() < deadline:
            attempt += 1
            self.move_coords(coords, speed)
            if deadline is None:
                deadline = time.monotonic() + self.pick_reach_timeout
            observe_until = min(deadline, time.monotonic() + 1.0)
            while time.monotonic() < observe_until:
                actual_coords = self.mc.get_coords()
                if (
                        isinstance(actual_coords, (list, tuple)) and len(actual_coords) >= 3
                        and abs(float(actual_coords[2]) - float(target_z)) <= self.pick_z_tolerance):
                    return True, actual_coords
                time.sleep(0.1)
            if time.monotonic() >= deadline:
                break
            print(
                f"[arm] {label} z not reached, retry within {self.pick_reach_timeout:.1f}s "
                f"(attempt={attempt}): target_z={target_z:.1f}, actual_coords={actual_coords}")
        return False, actual_coords

    def Rx(self, theta):
        return np.array([[1, 0, 0],
                         [0, np.cos(theta), -np.sin(theta)],
                         [0, np.sin(theta), np.cos(theta)]])

    def Ry(self, theta):
        return np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])

    def Rz(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])

    def degree2radian(self, degree):
        return (degree / 180) * np.pi

    def rotation_matrix(self, rx, ry, rz, order="ZYX"):
        order = order.upper()
        if len(order) != 3 or set(order) != set("XYZ"):
            raise Exception("Order must be string of component of XYZ or xyz")
        mat = np.identity(3)
        rx = self.degree2radian(rx)
        ry = self.degree2radian(ry)
        rz = self.degree2radian(rz)
        for c in order:
            if c == "X":
                mat = mat @ self.Rx(rx)
            elif c == "Y":
                mat = mat @ self.Ry(ry)
            elif c == "Z":
                mat = mat @ self.Rz(rz)
        return mat

    def homo_transform_matrix(self, x, y, z, rx, ry, rz, order="ZYX"):
        rot_mat = self.rotation_matrix(rx, ry, rz, order=order)
        trans_vec = np.array([[x, y, z, 1]]).T
        mat = np.vstack([rot_mat, np.zeros((1, 3))])
        mat = np.hstack([mat, trans_vec])
        return mat

    def qr_base_point(self, tvec, curr_coords=None):
        if curr_coords is None:
            curr_coords = self.get_coords_safe()
        mat = self.homo_transform_matrix(*curr_coords) @ \
              self.homo_transform_matrix(-10, -35, 10, 0, 0, 0)
        p_end = np.vstack([np.reshape(tvec, (3, 1)), 1])
        return np.squeeze((mat @ p_end)[:-1])

    def pick(self, box_height):
        self.move_angles(self.angle_table["pick_watch"], 50)

        while True:
            try:
                scanner = self.get_scanner()
                result = scanner.start_capture()
            except RuntimeError as error:
                print(f"[arm] QR scanner open failed: {error}")
                self.reset_scanner()
                time.sleep(0.5)
                continue

            if result == -1:
                continue
            if not result:
                self.reset_scanner()
                time.sleep(0.5)
                continue

            qr_texts, tvecs = result
            if qr_texts is None or tvecs is None:
                continue

            visual_height = getattr(scanner, "last_pick_height", None)
            edge_med = getattr(scanner, "last_visual_edge_median", None)
            tvec_z = getattr(scanner, "last_visual_tvec_z", None)
            box_height = (float(visual_height)
                          if visual_height in (PICK_HEIGHT_LOW, PICK_HEIGHT_HIGH)
                          else PICK_HEIGHT_HIGH)

            curr_coords = self.get_coords_safe()
            p_base = self.qr_base_point(tvecs[0], curr_coords).astype(int)

            print(f"[arm] pick height={box_height:.1f} | edge_med={edge_med} tvec_z={tvec_z}")

            p_base[0] -= 30 #y ->
            p_base[1] += 20 #x
            p_base[2] = box_height

            new_coords = [
                float(p_base[0]),
                float(p_base[1]),
                float(box_height),
                *[float(value) for value in curr_coords[3:]],
            ]
            print(f"[arm] pick target height={box_height:.1f}, coords={new_coords}")

            reached, actual_coords = self.move_pick_target(new_coords, box_height)
            if not reached:
                print(
                    f"[arm] pick target not reached, skip pump: "
                    f"target_z={box_height:.1f}, actual_coords={actual_coords}")
                self.move_angles(self.angle_table["pick_watch"], 50)
                continue

            self.pump_on()
            time.sleep(2)

            lift_coords = self.get_coords_safe()
            lift_coords = [float(value) for value in lift_coords]
            lift_coords[2] += 40.0
            print(f"[arm] post-pick lift target_z={lift_coords[2]:.1f}, coords={lift_coords}")
            self.move_coords_settle(lift_coords, 40)

            self.move_angles(self.angle_table["pick_point2"], 50)
            self.move_angles(self.angle_table["place_init"], 80)

            coords = self.get_coords_safe()
            coords[2] -= 45
            self.move_coords(coords, 40)

            self.pump_off()
            time.sleep(2)

            coords[2] += 45
            self.move_coords(coords, 40)

            self.move_angles(self.angle_table["place_point4"], 50)

            return qr_texts

    def place(self):
        self.move_angles_repeat(self.angle_table["place_init"], 50)

        coords = self.get_coords_safe()
        coords[2] -= 80
        reached, actual_coords = self.move_until_z_reached(coords, coords[2], 40, "place pickup")
        if not reached:
            print(
                f"[arm] place pickup target not reached, skip pump: "
                f"target_z={coords[2]:.1f}, actual_coords={actual_coords}")
            return False

        self.pump_on()
        time.sleep(1)

        coords = self.get_coords_safe()
        coords[2] += 80
        self.move_coords(coords, 40)

        self.move_angles(self.angle_table["place_point4"], 50)
        self.move_angles(self.angle_table["place_point2"], 50)
        self.move_angles(self.angle_table["place_point3"], 50)

        self.pump_off()
        time.sleep(2)

        self.move_angles(self.angle_table["move_init"], 50)
        return True
