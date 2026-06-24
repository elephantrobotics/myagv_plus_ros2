#! /usr/bin/env python3
from rclpy.node import Node
import numpy as np

# import Jetson.GPIO as GPIO
import time
from pymycobot import MechArm270
from smart_logistics_kit_lite.QRCodeScanner import QRCodeScanner
from smart_logistics_kit_lite.bottom_io import PumpClient

class MechArm270Control(Node):
    def __init__(self,port='/dev/ttyACM1',baudrate=115200,io_client=None):
        self.mc = MechArm270(port, baudrate)
        self.mc.set_fresh_mode(0)
        self.io = io_client if io_client is not None else PumpClient()

        self.scanner = QRCodeScanner("/dev/video1")

        self.angle_table = {
            "zero_position":[0,0,0,0,0,0],
            "move_init":[90.06, -30.41, 22.14, -1.05, 87.45, 0.39],
            "pick_init":[5.44, 6.5, -13.09, -2.54, 81.82, -4.3],
            "pick_watch":[94.13, 15.2, -21.88, 0.96, 90.79, 4.57],
            "pick_point2":[-57.91, 0.61, -8.34, 6.32, 19.24, -2.19],
            "place_init":[-93.6, 1.93, 6.24, -0.17, 68.81, -6.24],
            "place_point2":[-7.11, -5.62, -14.85, 0.87, 77.95, -10.37],
            "place_point3":[90.0, 22.5, -12.48, 2.54, 50.27, -0.35],
            "place_point4":[-95.36, 7.03, -22.85, -3.07, 87.89, 1.46]
        }

    def pump_on(self):
        self.io.set_pump_state(1)

    def pump_off(self):
        self.io.set_pump_state(0)

    def move_angles(self, angles, speed=50):
        self.mc.send_angles(angles, speed)
        self.wait()

    def move_coords(self, coords, speed=40, mode=1):
        self.mc.send_coords(coords, speed, mode)
        self.wait()

    def wait(self):
        time.sleep(0.3)
        state = self.mc.is_moving()
        while state != 0:
            state = self.mc.is_moving()
            time.sleep(0.1)
            
    def get_coords_safe(self):
        coords = self.mc.get_coords()
        while coords is None:
            time.sleep(0.2)
            coords = self.mc.get_coords()
        return coords

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


    def degree2radian(self,degree):
        return (degree / 180) * np.pi


    def rotation_matrix(self,rx, ry, rz, order="ZYX"):
        """
        :param rx: rx in degree
        :param ry: ry in degree
        :param rz: rz in degree
        :param order:
        :return:
        """
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


    def homo_transform_matrix(self,x, y, z, rx, ry, rz, order="ZYX"):
        rot_mat = self.rotation_matrix(rx, ry, rz, order=order)
        trans_vec = np.array([[x, y, z, 1]]).T
        mat = np.vstack([rot_mat, np.zeros((1, 3))])
        mat = np.hstack([mat, trans_vec])
        return mat


    def pick(self, box_height):
        self.move_angles(self.angle_table["pick_watch"], 60)

        while True:
            qr_texts, tvecs = self.scanner.start_capture()
            time.sleep(1)

            if qr_texts is None:
                continue

            curr_coords = self.get_coords_safe()

            mat = self.homo_transform_matrix(*curr_coords) @ \
                  self.homo_transform_matrix(-10, -35, 10, 0, 0, 0)

            p_end = np.vstack([np.reshape(tvecs[0], (3, 1)), 1])
            p_base = np.squeeze((mat @ p_end)[:-1]).astype(int)

            p_base[0] -= 10
            p_base[1] += 40
            p_base[2] = box_height

            new_coords = np.concatenate([p_base, curr_coords[3:]])

            self.move_coords(list(new_coords), 20)

            self.pump_on()
            time.sleep(2)

            curr_coords = self.get_coords_safe()
            curr_coords[2] += 40
            self.move_coords(curr_coords, 40)

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

            try:
                if self.scanner is not None:
                    self.scanner.release_resources()
                    del self.scanner
            except Exception as e:
                print(f"release scanner error: {e}")

            self.scanner = QRCodeScanner("/dev/video1")

            return qr_texts

    def place(self):
        self.move_angles([0,0,0,0,0,0], 60)
        self.move_angles(self.angle_table["place_init"], 50)

        coords = self.get_coords_safe()
        coords[2] -= 70
        self.move_coords(coords, 40)

        self.pump_on()
        time.sleep(1)

        coords = self.get_coords_safe()
        coords[2] += 70
        self.move_coords(coords, 40)

        self.move_angles(self.angle_table["place_point4"], 50)
        self.move_angles(self.angle_table["place_point2"], 50)
        self.move_angles(self.angle_table["place_point3"], 50)

        self.pump_off()
        time.sleep(2)

        self.move_angles(self.angle_table["move_init"], 50)