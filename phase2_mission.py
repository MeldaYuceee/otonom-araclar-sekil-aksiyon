#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math
import threading
from dataclasses import dataclass

from dronekit import connect, VehicleMode
from pymavlink import mavutil

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

import cv2
import numpy as np


# -------------------- Sabitler (senaryon ile aynı) --------------------
MODEL_DRONE = "iris_demo"
MODEL_A = "direk"
MODEL_B = "direk_0"
MODEL_HEX = "mavi_altigen"
MODEL_TRI = "kirmizi_ucgen"

# Sende KESİN doğru kamera topic'i:
IMAGE_TOPIC = "/iris_demo/gimbal_camera/image_raw"

TAKEOFF_ALT_M = 10.0
SCAN_ALT_M = 6.0
HEX_ALT_M = 3.0

WPT_REACH_THRESH_M = 1.0
ALT_REACH_THRESH_M = 0.5

TRIGGER_RADIUS_M = 3.0
STABLE_FRAMES = 3

HEX_COOLDOWN_S = 12.0
HEX_REARM_EXTRA_M = 2.0  # yeniden tetiklenmesi için uzaklaşma payı

RED_TRIGGER_PX = 450
RED_STABLE_FRAMES = 3

SETPOINT_HZ = 5.0

# HSV eşikleri (sahnen için uygun)
HSV_RED1_LOW = (0, 70, 60)
HSV_RED1_HIGH = (10, 255, 255)
HSV_RED2_LOW = (170, 70, 60)
HSV_RED2_HIGH = (180, 255, 255)

HSV_BLUE_LOW = (100, 120, 60)
HSV_BLUE_HIGH = (130, 255, 255)

MIN_AREA_PX = 300
EPS_FRACTION = 0.02
HEX_VERT_RANGE = (5, 7)     # approx köşe sayısı
TRI_VERT_SET = {3, 4}       # üçgen (ve bazen 4 köşeye indirgenebilir)


# -------------------- Yardımcılar --------------------
def normalize_conn(conn_str: str) -> str:
    """
    Kullanıcı '127.0.0.1:14550' yazsa bile, bağlantıyı doğru protokolle kur.
    MAVProxy default'u UDP:14550 olduğu için prefix yoksa 'udp:' ekleriz.
    """
    if not conn_str:
        return "udp:127.0.0.1:14550"
    lower = conn_str.lower()
    if lower.startswith("udp:") or lower.startswith("tcp:") or lower.startswith("com") or lower.startswith("serial:"):
        return conn_str
    # prefix yoksa UDP kabul et
    return f"udp:{conn_str}"


@dataclass
class TargetPoint:
    name: str
    gazebo_x: float
    gazebo_y: float


class GazeboClient:
    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        self._get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def get_xy(self, model_name: str):
        req = GetModelStateRequest()
        req.model_name = model_name
        req.relative_entity_name = "world"
        res = self._get_state(req)
        if not res.success:
            raise RuntimeError(f"get_model_state başarısız: {model_name}")
        return res.pose.position.x, res.pose.position.y


class NEDController:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def wait_for_local_origin(self, timeout_s=30):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            lf = self.vehicle.location.local_frame
            if lf and lf.north is not None and lf.east is not None and lf.down is not None:
                return True
            time.sleep(0.1)
        return False

    def set_mode_guided(self):
        self.vehicle.mode = VehicleMode("GUIDED")
        while self.vehicle.mode.name != "GUIDED":
            time.sleep(0.1)

    def current_alt_agl(self):
        lf = self.vehicle.location.local_frame
        if not lf or lf.down is None:
            return 0.0
        return -float(lf.down)

    def current_ned(self):
        lf = self.vehicle.location.local_frame
        if not lf:
            return 0.0, 0.0, 0.0
        n = 0.0 if lf.north is None else float(lf.north)
        e = 0.0 if lf.east is None else float(lf.east)
        d = 0.0 if lf.down is None else float(lf.down)
        return n, e, d

    def arm_and_takeoff(self, alt_m):
        self.set_mode_guided()
        self.vehicle.armed = True
        while not self.vehicle.armed:
            rospy.loginfo("Armlanıyor...")
            time.sleep(0.5)

        rospy.loginfo("STATE=TAKEOFF_10M")
        self.vehicle.simple_takeoff(alt_m)

        while not rospy.is_shutdown():
            alt = self.current_alt_agl()
            rospy.loginfo_throttle(1.0, f"[ALT] {alt:.1f} / hedef {alt_m:.1f}")
            if alt >= alt_m - ALT_REACH_THRESH_M:
                break
            time.sleep(0.2)

    def goto_ned_pos(self, north, east, alt_m):
        down = -alt_m
        type_mask = 0b0000111111111000  # pos aktif, diğerleri ignore
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north, east, down,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def reached(self, north, east, thresh=WPT_REACH_THRESH_M):
        cn, ce, _ = self.current_ned()
        dist = math.hypot(north - cn, east - ce)
        return dist <= thresh, dist

    def change_alt(self, alt_m):
        cn, ce, _ = self.current_ned()
        self.goto_ned_pos(cn, ce, alt_m)

    def land(self):
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.mode.name != "LAND":
            time.sleep(0.1)


class ShapeDetector:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.red_px = 0
        self.blue_px = 0

        self._tri_poly_cnt = 0
        self._hex_cnt = 0
        self._red_cnt = 0

        self.tri_poly_stable = False
        self.hex_stable = False
        self.red_stable = False

        self._dbg = 0
        self.sub = rospy.Subscriber(topic, Image, self._cb, queue_size=1)

    def _has_poly(self, mask, tri=False):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA_PX:
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, EPS_FRACTION * peri, True)
            v = len(approx)
            if tri:
                if v in TRI_VERT_SET:
                    return True
            else:
                lo, hi = HEX_VERT_RANGE
                if lo <= v <= hi:
                    return True
        return False

    def _cb(self, msg):
        try:
            if msg.encoding.lower() == "rgb8":
                rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask_r1 = cv2.inRange(hsv, HSV_RED1_LOW, HSV_RED1_HIGH)
        mask_r2 = cv2.inRange(hsv, HSV_RED2_LOW, HSV_RED2_HIGH)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)

        mask_blue = cv2.inRange(hsv, HSV_BLUE_LOW, HSV_BLUE_HIGH)

        k = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, k, iterations=1)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, k, iterations=1)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, k, iterations=1)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, k, iterations=1)

        red_px = int(cv2.countNonZero(mask_red))
        blue_px = int(cv2.countNonZero(mask_blue))

        tri_poly = self._has_poly(mask_red, tri=True)
        hexx = self._has_poly(mask_blue, tri=False)

        self._tri_poly_cnt = self._tri_poly_cnt + 1 if tri_poly else 0
        self._hex_cnt = self._hex_cnt + 1 if hexx else 0
        self._red_cnt = self._red_cnt + 1 if red_px >= RED_TRIGGER_PX else 0

        tri_poly_stable = self._tri_poly_cnt >= STABLE_FRAMES
        hex_stable = self._hex_cnt >= STABLE_FRAMES
        red_stable = self._red_cnt >= RED_STABLE_FRAMES

        with self.lock:
            self.red_px = red_px
            self.blue_px = blue_px
            self.tri_poly_stable = tri_poly_stable
            self.hex_stable = hex_stable
            self.red_stable = red_stable

        self._dbg += 1
        if self._dbg % 20 == 0:
            rospy.loginfo(
                f"[VISION] red_px={red_px} blue_px={blue_px} "
                f"TRI_POLY={tri_poly_stable} HEX={hex_stable} RED_STABLE={red_stable}"
            )

    def get(self):
        with self.lock:
            return self.tri_poly_stable, self.hex_stable, self.red_stable, self.red_px, self.blue_px


class Phase2Mission:
    def __init__(self, vehicle, gz, ned, vision):
        self.vehicle = vehicle
        self.gz = gz
        self.ned = ned
        self.vision = vision

        self.home_gx = None
        self.home_gy = None

        self.a_ne = (0.0, 0.0)
        self.b_ne = (0.0, 0.0)
        self.hex_ne = (0.0, 0.0)
        self.tri_ne = (0.0, 0.0)

        self.ignore_hex_until = 0.0
        self.hex_armed = True

    def init_home(self):
        gx, gy = self.gz.get_xy(MODEL_DRONE)
        self.home_gx, self.home_gy = gx, gy
        rospy.loginfo(f"[HOME@Gazebo] x={gx:.3f} y={gy:.3f}")

    def gz_to_ne(self, gx, gy):
        dx = gx - self.home_gx
        dy = gy - self.home_gy
        return dx, -dy  # North≈Δx, East≈-Δy

    def load_points(self):
        xa, ya = self.gz.get_xy(MODEL_A)
        xb, yb = self.gz.get_xy(MODEL_B)
        xh, yh = self.gz.get_xy(MODEL_HEX)
        xt, yt = self.gz.get_xy(MODEL_TRI)

        self.a_ne = self.gz_to_ne(xa, ya)
        self.b_ne = self.gz_to_ne(xb, yb)
        self.hex_ne = self.gz_to_ne(xh, yh)
        self.tri_ne = self.gz_to_ne(xt, yt)

        rospy.loginfo(f"[A@NE]   N={self.a_ne[0]:.2f} E={self.a_ne[1]:.2f}")
        rospy.loginfo(f"[B@NE]   N={self.b_ne[0]:.2f} E={self.b_ne[1]:.2f}")
        rospy.loginfo(f"[HEX@NE] N={self.hex_ne[0]:.2f} E={self.hex_ne[1]:.2f}")
        rospy.loginfo(f"[TRI@NE] N={self.tri_ne[0]:.2f} E={self.tri_ne[1]:.2f}")

    def wait_alt(self, alt):
        while not rospy.is_shutdown():
            cur = self.ned.current_alt_agl()
            if abs(cur - alt) <= ALT_REACH_THRESH_M:
                return
            rospy.loginfo_throttle(1.0, f"[ALT] {cur:.2f} -> hedef {alt:.2f}")
            time.sleep(0.1)

    def dist_to(self, ne):
        cn, ce, _ = self.ned.current_ned()
        return math.hypot(ne[0] - cn, ne[1] - ce)

    def stream_to(self, n, e, alt, thresh=1.0, timeout=180):
        period = 1.0 / SETPOINT_HZ
        t0 = time.time()
        while not rospy.is_shutdown():
            self.ned.goto_ned_pos(n, e, alt)
            ok, dist = self.ned.reached(n, e, thresh)
            if ok:
                return True
            if time.time() - t0 > timeout:
                return False
            time.sleep(period)

    def do_hex_action(self):
        rospy.loginfo("STATE=HEXAGON_DETECTED")
        n, e = self.hex_ne

        self.stream_to(n, e, SCAN_ALT_M, thresh=1.0, timeout=180)

        rospy.loginfo("STATE=DESCEND_3M")
        self.ned.change_alt(HEX_ALT_M)
        self.wait_alt(HEX_ALT_M)

        rospy.loginfo("STATE=WAIT_5S")
        time.sleep(5.0)

        rospy.loginfo("STATE=ASCEND_10M")
        self.ned.change_alt(TAKEOFF_ALT_M)
        self.wait_alt(TAKEOFF_ALT_M)

        rospy.loginfo("STATE=RETURN_TO_SCAN_6M")
        self.ned.change_alt(SCAN_ALT_M)
        self.wait_alt(SCAN_ALT_M)

        self.ignore_hex_until = time.time() + HEX_COOLDOWN_S
        self.hex_armed = False
        rospy.loginfo("STATE=RESUME_SCANNING")

    def do_triangle_action(self):
        rospy.loginfo("STATE=TRIANGLE_DETECTED")
        n, e = self.tri_ne

        cur_alt = self.ned.current_alt_agl()
        self.stream_to(n, e, cur_alt, thresh=1.0, timeout=180)

        rospy.loginfo("STATE=LAND")
        self.ned.land()
        rospy.loginfo("STATE=DONE")

    def pole_pass(self):
        rospy.loginfo("STATE=POLE_PASS")
        self.stream_to(self.a_ne[0], self.a_ne[1], TAKEOFF_ALT_M, thresh=1.0, timeout=180)
        self.stream_to(self.b_ne[0], self.b_ne[1], TAKEOFF_ALT_M, thresh=1.0, timeout=180)
        self.stream_to(self.a_ne[0], self.a_ne[1], TAKEOFF_ALT_M, thresh=1.0, timeout=180)

    def run(self):
        self.ned.arm_and_takeoff(TAKEOFF_ALT_M)
        self.init_home()
        self.load_points()

        self.pole_pass()

        rospy.loginfo("STATE=DESCEND_TO_SCAN_6M")
        self.ned.change_alt(SCAN_ALT_M)
        self.wait_alt(SCAN_ALT_M)

        rospy.loginfo("STATE=SCAN_ROUTE")
        scan_route = [
            ("A(direk)", self.a_ne[0], self.a_ne[1]),
            ("HEX", self.hex_ne[0], self.hex_ne[1]),
            ("TRI", self.tri_ne[0], self.tri_ne[1]),
            ("B(direk_0)", self.b_ne[0], self.b_ne[1]),
        ]

        period = 1.0 / SETPOINT_HZ
        idx = 0

        while not rospy.is_shutdown():
            name, tn, te = scan_route[idx]
            rospy.loginfo(f"[WPT] {name} -> N={tn:.2f} E={te:.2f} ALT={SCAN_ALT_M:.1f}")

            while not rospy.is_shutdown():
                self.ned.goto_ned_pos(tn, te, SCAN_ALT_M)
                reached, dist = self.ned.reached(tn, te, WPT_REACH_THRESH_M)

                tri_poly, hex_ok, red_ok, rpx, bpx = self.vision.get()
                d_hex = self.dist_to(self.hex_ne)
                d_tri = self.dist_to(self.tri_ne)

                rospy.loginfo_throttle(
                    1.0,
                    f"[PROGRESS] d={dist:.2f} red_px={rpx} blue_px={bpx} "
                    f"TRI_POLY={tri_poly} RED_STABLE={red_ok} HEX={hex_ok} "
                    f"d_hex={d_hex:.2f} d_tri={d_tri:.2f}"
                )

                tri_trigger = (tri_poly or red_ok) and (d_tri <= TRIGGER_RADIUS_M)
                if tri_trigger:
                    self.do_triangle_action()
                    return

                if not self.hex_armed:
                    if d_hex > (TRIGGER_RADIUS_M + HEX_REARM_EXTRA_M):
                        self.hex_armed = True

                if self.hex_armed and time.time() >= self.ignore_hex_until:
                    if hex_ok and d_hex <= TRIGGER_RADIUS_M:
                        self.do_hex_action()
                        break

                if reached:
                    rospy.loginfo(f"[WPT] {name} ulaşıldı")
                    break

                time.sleep(period)

            idx = (idx + 1) % len(scan_route)


# -------------------- main --------------------
def main():
    import argparse
    parser = argparse.ArgumentParser()
    # Varsayılanı UDP yapıyoruz (MAVProxy default)
    parser.add_argument("--connect", default="udp:127.0.0.1:14550")
    args = parser.parse_args()

    conn = normalize_conn(args.connect)
    rospy.init_node("phase2_mission_node", anonymous=True)
    rospy.loginfo(f"DroneKit bağlanıyor: {conn}")

    # DroneKit bağlantısı
    vehicle = connect(conn, wait_ready=True, timeout=60)

    ned = NEDController(vehicle)
    if not ned.wait_for_local_origin(30):
        rospy.logwarn("Local NED henüz hazır değil, devam ediyorum...")

    gz = GazeboClient()
    vision = ShapeDetector(IMAGE_TOPIC)

    mission = Phase2Mission(vehicle, gz, ned, vision)

    try:
        mission.run()
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C")
    finally:
        try:
            vehicle.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
