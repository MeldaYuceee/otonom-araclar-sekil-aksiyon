#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math

from dronekit import connect, VehicleMode
from pymavlink import mavutil

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


MODEL_DRONE = "iris_demo"
ALT = 6.0
STEP = 1.0
HZ = 5.0


class GazeboClient:
    def __init__(self):
        rospy.wait_for_service("/gazebo/get_model_state")
        self._get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    def get_xyz(self, name):
        req = GetModelStateRequest()
        req.model_name = name
        req.relative_entity_name = "world"
        res = self._get_state(req)
        if not res.success:
            raise RuntimeError("get_model_state başarısız")
        p = res.pose.position
        return p.x, p.y, p.z


def goto_ne(vehicle, n, e, alt):
    d = -alt
    mask = 0b0000111111111000
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        n, e, d,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--connect", default="127.0.0.1:14550")
    args = parser.parse_args()

    rospy.init_node("axis_probe_air3", anonymous=True)

    vehicle = connect(args.connect, wait_ready=True, timeout=60)
    gz = GazeboClient()

    try:
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            time.sleep(0.1)

        vehicle.armed = True
        while not vehicle.armed:
            time.sleep(0.2)

        vehicle.simple_takeoff(ALT)
        time.sleep(6)

        x0, y0, _ = gz.get_xyz(MODEL_DRONE)
        lf = vehicle.location.local_frame
        n0 = float(lf.north) if lf and lf.north is not None else 0.0
        e0 = float(lf.east) if lf and lf.east is not None else 0.0

        def stream_to(n, e, t=6):
            period = 1.0 / HZ
            t0 = time.time()
            while time.time() - t0 < t:
                goto_ne(vehicle, n, e, ALT)
                time.sleep(period)

        print("TEST: North +1m")
        stream_to(n0 + STEP, e0, t=6)
        x1, y1, _ = gz.get_xyz(MODEL_DRONE)
        print(f"Gazebo Δx={x1 - x0:.2f}, Δy={y1 - y0:.2f}")

        print("TEST: East +1m")
        stream_to(n0, e0 + STEP, t=6)
        x2, y2, _ = gz.get_xyz(MODEL_DRONE)
        print(f"Gazebo Δx={x2 - x1:.2f}, Δy={y2 - y1:.2f}")

        print("Beklenen eşleşme: North≈Δx, East≈-Δy")

    finally:
        try:
            vehicle.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
