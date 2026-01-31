#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math

from dronekit import connect, VehicleMode
from pymavlink import mavutil

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


MODEL_DRONE = "iris_demo"
MODEL_A = "direk"
MODEL_B = "direk_0"

TAKEOFF_ALT_M = 10.0
SETPOINT_HZ = 5.0
WPT_THRESH = 1.0


class GazeboClient:
    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        self._get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def get_xy(self, name):
        req = GetModelStateRequest()
        req.model_name = name
        req.relative_entity_name = "world"
        res = self._get_state(req)
        if not res.success:
            raise RuntimeError(f"get_model_state başarısız: {name}")
        return res.pose.position.x, res.pose.position.y


class NED:
    def __init__(self, vehicle):
        self.v = vehicle

    def mode_guided(self):
        self.v.mode = VehicleMode("GUIDED")
        while self.v.mode.name != "GUIDED":
            time.sleep(0.1)

    def arm_takeoff(self, alt):
        self.mode_guided()
        self.v.armed = True
        while not self.v.armed:
            time.sleep(0.2)
        self.v.simple_takeoff(alt)
        while True:
            lf = self.v.location.local_frame
            alt_now = 0.0 if not lf or lf.down is None else -float(lf.down)
            if alt_now >= alt - 0.5:
                break
            time.sleep(0.2)

    def cur_ne(self):
        lf = self.v.location.local_frame
        if not lf:
            return 0.0, 0.0
        n = 0.0 if lf.north is None else float(lf.north)
        e = 0.0 if lf.east is None else float(lf.east)
        return n, e

    def goto_ne(self, n, e, alt):
        d = -alt
        mask = 0b0000111111111000
        msg = self.v.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,
            n, e, d,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.v.send_mavlink(msg)
        self.v.flush()

    def reached(self, n, e, thr=WPT_THRESH):
        cn, ce = self.cur_ne()
        dist = math.hypot(n - cn, e - ce)
        return dist <= thr, dist

    def stream_to(self, n, e, alt, timeout=180):
        period = 1.0 / SETPOINT_HZ
        t0 = time.time()
        while True:
            self.goto_ne(n, e, alt)
            ok, _ = self.reached(n, e)
            if ok:
                return True
            if time.time() - t0 > timeout:
                return False
            time.sleep(period)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="127.0.0.1:14550")
    args = parser.parse_args()

    rospy.init_node("pole_nav_v2", anonymous=True)

    vehicle = connect(args.connect, wait_ready=True, timeout=60)
    gz = GazeboClient()
    ned = NED(vehicle)

    try:
        ned.arm_takeoff(TAKEOFF_ALT_M)

        hx, hy = gz.get_xy(MODEL_DRONE)
        ax, ay = gz.get_xy(MODEL_A)
        bx, by = gz.get_xy(MODEL_B)

        def gz_to_ne(x, y):
            dx = x - hx
            dy = y - hy
            return dx, -dy

        a_n, a_e = gz_to_ne(ax, ay)
        b_n, b_e = gz_to_ne(bx, by)

        print("STATE=GO_A")
        ned.stream_to(a_n, a_e, TAKEOFF_ALT_M)

        print("STATE=GO_B")
        ned.stream_to(b_n, b_e, TAKEOFF_ALT_M)

        print("STATE=RETURN_A")
        ned.stream_to(a_n, a_e, TAKEOFF_ALT_M)

        print("STATE=DONE")

    finally:
        try:
            vehicle.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
