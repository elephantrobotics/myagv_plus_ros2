#!/usr/bin/env python3

import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from pyzbar.pyzbar import decode


PACKAGE_DIR = Path(__file__).resolve().parents[1] / "smart_logistics_kit_lite"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))


# ===== 一、常调参数；Frequently tuned =====

# 基座系 XY 修正，mm；Base-frame XY correction, in mm.
# 对应 arm_controller.py pick() 的 p_base[0] / p_base[1]。
PICK_BASE_OFFSET_X = -30.0  # y -> 车体左右；lateral
PICK_BASE_OFFSET_Y = 20.0   # x -> 车体前后；longitudinal

# 低档判定：edge 小且码面远，其余按高档；Low level: small edge and far QR plane, else high.
# 对应 QRCodeScanner.py 的 VISUAL_LOW_EDGE_MAX / VISUAL_LOW_TVEC_Z_MIN。
LOW_EDGE_MAX = 52.03
LOW_TVEC_Z_MIN = 0.3579

# 两档下抓高度，mm；Two pick heights, in mm.
# 对应 QRCodeScanner.py 的 PICK_HEIGHT_LOW / PICK_HEIGHT_HIGH。
PICK_Z_LOW = 3.0
PICK_Z_HIGH = 45.0

# 托盘二次取件的下降量，mm；Drop distance for re-picking from the tray, in mm.
PLACE_PICK_DROP_Z = 80.0

# 本脚本专用位姿副本，改这里不影响 arm_controller.py；Test-only pose copy, does not touch arm_controller.py.
ANGLE_TABLE = {
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


# ===== 二、装配参数，换硬件或重标才改；Rig constants, change only on hardware or re-calibration =====

# 手眼偏移，相机相对法兰；Hand-eye offset, camera relative to the flange.
# 对应 arm_controller.py 的 homo_transform_matrix(-10, -35, 10, 0, 0, 0)。
PICK_CAMERA_OFFSET = [-10.0, -35.0, 10.0, 0.0, 0.0, 0.0]

# 二维码黑框实际边长，米；Printed QR black border edge length, in meters.
DEFAULT_QR_SIZE_M = 0.0225

DEFAULT_CAMERA_MATRIX = np.array([
    [827.29511682, 0.0, 368.87666292],
    [0.0, 824.88958537, 262.03016541],
    [0.0, 0.0, 1.0],
], dtype=np.float64)

DEFAULT_DIST_COEFFS = np.array(
    [[0.21780081, -0.56324781, 0.01165061, 0.01845253, -1.0631406]],
    dtype=np.float64,
)

ARM_PORT = "/dev/ttyACM0"
ARM_BAUDRATE = 115200
CAMERA_DEVICE = "/dev/video1"
CAMERA_BACKEND = "auto"


# ===== 三、行为参数，一般不动；Behaviour knobs, rarely changed =====

ARM_SPEED = 60
ARM_TIMEOUT = 0.0
PICK_REACH_TIMEOUT = 3.0
PICK_Z_TOLERANCE = 18.0  # 生产 arm_controller.py 为 8.0；Production uses 8.0.
INIT_PICK_WATCH = True
ORIGINAL_PICK_QR_SHOW_WINDOW = True

FLUSH_FRAMES = 8
STABLE_TIMEOUT = 8.0
STABLE_COUNT = 5
STABLE_EDGE_TOLERANCE = 1.0
STABLE_Z_TOLERANCE = 0.005


@dataclass
class QRMeasurement:
    timestamp: float
    text: str
    known_height: str
    tvec_x: float
    tvec_y: float
    tvec_z: float
    distance: float
    edge_mean: float
    edge_median: float
    edge_min: float
    edge_max: float
    sqrt_area: float
    area: float
    center_x: float
    center_y: float
    reprojection_error: float
    height_guess: str = ""
    confidence: str = ""


def make_object_points(qr_size_m):
    half = qr_size_m / 2.0
    return np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)


def edge_lengths(quad):
    return [
        float(np.linalg.norm(quad[(index + 1) % 4] - quad[index]))
        for index in range(4)
    ]


def solve_qr_pose_original_style(quad, object_points):
    image_points = np.float32(quad)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        DEFAULT_CAMERA_MATRIX,
        DEFAULT_DIST_COEFFS,
    )
    if not ok:
        return None

    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, DEFAULT_CAMERA_MATRIX, DEFAULT_DIST_COEFFS
    )
    projected = projected.reshape(-1, 2)
    reprojection_error = float(np.mean(np.linalg.norm(projected - image_points, axis=1)))
    return rvec, tvec.reshape(3), reprojection_error


def build_measurement(obj, quad, pose, known_height=""):
    _rvec, tvec, reprojection_error = pose
    quad_float = quad.astype(np.float32)
    lengths = edge_lengths(quad_float)
    area = abs(float(cv2.contourArea(quad_float)))
    center = quad_float.mean(axis=0)
    text = obj.data.decode("utf-8", errors="ignore").strip()

    return QRMeasurement(
        timestamp=time.time(),
        text=text,
        known_height=known_height,
        tvec_x=float(tvec[0]),
        tvec_y=float(tvec[1]),
        tvec_z=float(tvec[2]),
        distance=float(np.linalg.norm(tvec)),
        edge_mean=float(statistics.mean(lengths)),
        edge_median=float(statistics.median(lengths)),
        edge_min=float(min(lengths)),
        edge_max=float(max(lengths)),
        sqrt_area=(area ** 0.5) if area > 0.0 else 0.0,
        area=area,
        center_x=float(center[0]),
        center_y=float(center[1]),
        reprojection_error=reprojection_error,
    )


def detect_measurement_from_frame(frame, object_points, known_height=""):
    for obj in decode(frame):
        if len(obj.polygon) != 4:
            continue
        quad = np.array(obj.polygon, dtype=np.int32)
        pose = solve_qr_pose_original_style(quad, object_points)
        if pose is None:
            continue
        return build_measurement(obj, quad, pose, known_height), quad
    return None, None


def measurement_line(index, measurement):
    return (
        f"sample={index:03d} known={measurement.known_height or '-'} "
        f"guess={measurement.height_guess or '-'} conf={measurement.confidence or '-'} "
        f"tvec=({measurement.tvec_x:.4f},{measurement.tvec_y:.4f},{measurement.tvec_z:.4f}) "
        f"dist={measurement.distance:.4f} edge_med={measurement.edge_median:.1f} "
        f"sqrt_area={measurement.sqrt_area:.1f} err={measurement.reprojection_error:.2f} "
        f"text={measurement.text}"
    )


def is_stable_measurement(previous, current, edge_tolerance, z_tolerance):
    if previous is None or current is None:
        return False
    return (
        abs(previous.edge_median - current.edge_median) <= edge_tolerance
        and abs(previous.tvec_z - current.tvec_z) <= z_tolerance
    )


def capture_stable_measurement(
    cap,
    object_points,
    timeout,
    stable_count,
    edge_tolerance,
    z_tolerance,
    known_height="",
):
    start_time = time.monotonic()
    last_measurement = None
    stable_seen = 0
    detected_count = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("Cannot read camera frame.")
            return None

        measurement, _quad = detect_measurement_from_frame(
            frame, object_points, known_height=known_height
        )
        if measurement is not None:
            detected_count += 1
            if is_stable_measurement(last_measurement, measurement, edge_tolerance, z_tolerance):
                stable_seen += 1
            else:
                stable_seen = 1
            last_measurement = measurement

            print(
                f"stable={stable_seen}/{stable_count} "
                f"{measurement_line(detected_count, measurement)}"
            )
            if stable_seen >= stable_count:
                return measurement

        if time.monotonic() - start_time >= timeout:
            if last_measurement is not None:
                print("Stable timeout; using last detected QR measurement.")
                return last_measurement
            print(f"QR stable capture timeout after {timeout:.1f} s.")
            return None


def open_capture(device, backend):
    if backend == "v4l2":
        return cv2.VideoCapture(device, cv2.CAP_V4L2)
    if backend == "default":
        return cv2.VideoCapture(device)

    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if cap.isOpened():
        return cap
    cap.release()
    return cv2.VideoCapture(device)


def pick_z_for_class(height_class):
    if height_class == "low":
        return float(PICK_Z_LOW)
    if height_class == "high":
        return float(PICK_Z_HIGH)
    return None


def classify_pick_height(measurement):
    low_band = measurement.edge_median <= LOW_EDGE_MAX and measurement.tvec_z >= LOW_TVEC_Z_MIN
    if low_band:
        measurement.height_guess = "low"
        measurement.confidence = "high"
        return "low", "high", "small_edge_and_high_z"

    measurement.height_guess = "high"
    measurement.confidence = "default"
    return "high", "default", "not_low_default_high"


def resolve_height_for_pick(measurement):
    height_class, confidence, reason = classify_pick_height(measurement)
    return pick_z_for_class(height_class), height_class, confidence, reason


class ArmPickTester:
    def __init__(
        self,
        port=ARM_PORT,
        baudrate=ARM_BAUDRATE,
        speed=ARM_SPEED,
        timeout=ARM_TIMEOUT,
        pick_z_tolerance=PICK_Z_TOLERANCE,
        pick_reach_timeout=PICK_REACH_TIMEOUT,
        qr_show_window=ORIGINAL_PICK_QR_SHOW_WINDOW,
        io_client=None,
    ):
        try:
            from arm_controller import MechArm270Control
        except ImportError as error:
            try:
                from smart_logistics_kit.arm_controller import MechArm270Control
            except ImportError as fallback_error:
                raise RuntimeError(
                    f"arm_controller import failed: {error}; fallback failed: {fallback_error}"
                ) from fallback_error

        self.arm = MechArm270Control(
            port=port,
            baudrate=baudrate,
            qr_camera=CAMERA_DEVICE,
            qr_timeout=STABLE_TIMEOUT,
            qr_show_window=qr_show_window,
            io_client=io_client,
        )
        self.arm.angle_table.update(
            {name: list(angles) for name, angles in ANGLE_TABLE.items()}
        )
        self.arm.pick_z_tolerance = float(pick_z_tolerance)
        self.arm.pick_reach_timeout = float(pick_reach_timeout)

        module = sys.modules.get(MechArm270Control.__module__)
        print(f"[diag] arm_controller={getattr(module, '__file__', '?')}")
        print(
            f"[diag] v2 pick_watch={self.arm.angle_table['pick_watch']}, "
            f"pick_point2={self.arm.angle_table['pick_point2']}, "
            f"place_init={self.arm.angle_table['place_init']}, "
            f"place_point4={self.arm.angle_table['place_point4']}, "
            f"camera_offset={PICK_CAMERA_OFFSET}, "
            f"base_offset=({PICK_BASE_OFFSET_X:.1f}, {PICK_BASE_OFFSET_Y:.1f})"
        )

    def move_pick_watch(self):
        print(f"[arm] restore pick_watch: angles={self.arm.angle_table['pick_watch']}")
        self.arm.move_angles(self.arm.angle_table["pick_watch"], 60)
        return True

    def reset_scanner(self):
        self.arm.reset_scanner()

    def place_main_flow(self):
        print("[arm] run test_qr_pick_place original-style unload/place flow.")
        print(f"[arm] rear place_init repeat=3, angles={self.arm.angle_table['place_init']}")
        self.arm.move_angles_repeat(self.arm.angle_table["place_init"], 50, count=3)

        coords = self.arm.get_coords_safe()
        coords = [float(value) for value in coords]
        coords[2] -= PLACE_PICK_DROP_Z
        print(f"[arm] rear pick target drop_z={PLACE_PICK_DROP_Z:.1f}, coords={coords}")
        self.arm.move_coords(coords, 40)

        self.arm.pump_on()
        time.sleep(1.0)

        coords = self.arm.get_coords_safe()
        coords = [float(value) for value in coords]
        coords[2] += PLACE_PICK_DROP_Z
        print(f"[arm] rear pick lift target_z={coords[2]:.1f}, coords={coords}")
        self.arm.move_coords(coords, 40)

        self.arm.move_angles(self.arm.angle_table["place_point4"], 50)
        self.arm.move_angles(self.arm.angle_table["place_point2"], 50)
        self.arm.move_angles(self.arm.angle_table["place_point3"], 50)

        self.arm.pump_off()
        time.sleep(2.0)

        self.arm.move_angles(self.arm.angle_table["move_init"], 50)

    def make_pick_coords_from_tvec(self, tvecs, box_height):
        curr_coords = self.arm.get_coords_safe()
        mat = self.arm.homo_transform_matrix(*curr_coords) @ self.arm.homo_transform_matrix(
            *PICK_CAMERA_OFFSET
        )
        p_end = np.vstack([np.reshape(tvecs[0], (3, 1)), 1])
        p_base = np.squeeze((mat @ p_end)[:-1]).astype(int)
        p_base[0] += PICK_BASE_OFFSET_X
        p_base[1] += PICK_BASE_OFFSET_Y
        p_base[2] = box_height

        return [
            float(p_base[0]),
            float(p_base[1]),
            float(box_height),
            *[float(value) for value in curr_coords[3:]],
        ]

    def pick_v2_original_style_flow(self, box_height):
        self.move_pick_watch()

        while True:
            try:
                scanner = self.arm.get_scanner()
                result = scanner.start_capture()
            except RuntimeError as error:
                print(f"[arm] QR scanner open failed: {error}")
                self.arm.reset_scanner()
                time.sleep(0.5)
                continue

            if result == -1:
                continue
            if not result:
                self.arm.reset_scanner()
                time.sleep(0.5)
                continue

            qr_texts, tvecs = result
            if qr_texts is None or tvecs is None:
                continue

            coords = self.make_pick_coords_from_tvec(tvecs, box_height)
            print(f"[arm] pick target height={box_height:.1f}, coords={coords}")

            reached, actual_coords = self.arm.move_pick_target(coords, box_height)
            if not reached:
                print(
                    f"[arm] pick target not reached, skip pump: "
                    f"target_z={box_height:.1f}, actual_coords={actual_coords}"
                )
                self.move_pick_watch()
                continue

            self.arm.pump_on()
            time.sleep(2.0)

            lift_coords = [float(value) for value in self.arm.get_coords_safe()]
            lift_coords[2] += 40.0
            print(f"[arm] post-pick lift target_z={lift_coords[2]:.1f}, coords={lift_coords}")
            self.arm.move_coords_settle(lift_coords, 40)

            self.arm.move_angles(self.arm.angle_table["pick_point2"], 50)
            self.arm.move_angles(self.arm.angle_table["place_init"], 80)

            place_coords = self.arm.get_coords_safe()
            place_coords[2] -= 45
            self.arm.move_coords(place_coords, 40)

            self.arm.pump_off()
            time.sleep(2.0)

            place_coords[2] += 45
            self.arm.move_coords(place_coords, 40)
            self.arm.move_angles(self.arm.angle_table["place_point4"], 50)
            return qr_texts


def run_live_pick_test(io_client):
    arm = ArmPickTester(
        port=ARM_PORT,
        baudrate=ARM_BAUDRATE,
        speed=ARM_SPEED,
        timeout=ARM_TIMEOUT,
        pick_z_tolerance=PICK_Z_TOLERANCE,
        pick_reach_timeout=PICK_REACH_TIMEOUT,
        qr_show_window=ORIGINAL_PICK_QR_SHOW_WINDOW,
        io_client=io_client,
    )
    object_points = make_object_points(DEFAULT_QR_SIZE_M)

    print("QR low-package classifier pick/place test started.")
    print("Ready Enter: classify height and pick package to rear area.")
    print("Place Enter: run original unload/place flow. Type q then Enter to quit.")
    print("Threshold source: production QRCodeScanner.py VISUAL_LOW_* constants.")
    print(
        f"Low classifier: LOW/Z={PICK_Z_LOW:.1f} if edge_med<={LOW_EDGE_MAX:.2f} "
        f"and tvec_z>={LOW_TVEC_Z_MIN:.4f}; otherwise HIGH/Z={PICK_Z_HIGH:.1f}."
    )

    while True:
        command = input("\nReady> ").strip().lower()
        if command in {"q", "quit", "exit"}:
            break

        if INIT_PICK_WATCH:
            arm.move_pick_watch()

        cap = open_capture(CAMERA_DEVICE, CAMERA_BACKEND)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {CAMERA_DEVICE}")
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        try:
            for _ in range(max(FLUSH_FRAMES, 0)):
                cap.grab()
                time.sleep(0.01)
            measurement = capture_stable_measurement(
                cap,
                object_points,
                timeout=STABLE_TIMEOUT,
                stable_count=STABLE_COUNT,
                edge_tolerance=STABLE_EDGE_TOLERANCE,
                z_tolerance=STABLE_Z_TOLERANCE,
            )
        finally:
            cap.release()

        if measurement is None:
            print("No QR measurement; skip this cycle.")
            continue

        pick_z, height_class, confidence, reason = resolve_height_for_pick(measurement)
        if pick_z is None:
            print(f"Skip pick: confidence={confidence}, reason={reason}")
            continue

        print(measurement_line(1, measurement))
        print(
            f"Confirmed pick: class={height_class}, pick_z={pick_z:.1f}, "
            f"confidence={confidence}, reason={reason}"
        )
        print("[arm] run test_qr_pick_place independent original-style pick flow.")
        try:
            arm.pick_v2_original_style_flow(pick_z)
        finally:
            arm.reset_scanner()

        command = input("\nPlace> press Enter to run unload/place flow, s(skip), or q(quit): ").strip().lower()
        if command in {"q", "quit", "exit"}:
            break
        if command in {"s", "skip"}:
            continue

        arm.place_main_flow()


def main():
    import rclpy
    from ros_client import AGVIOClient

    rclpy.init()
    io_client = AGVIOClient()
    try:
        run_live_pick_test(io_client)
    finally:
        io_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
