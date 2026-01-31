#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np

from dronekit import connect, VehicleMode
from pymavlink import mavutil

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState, GetModelStateRequest


MODEL_DRONE = "iris_demo"
MODEL_TARGET = "mavi_altigen"  # istersen "kirmizi_ucgen" yap
IMAGE_TOPIC = "/iris_demo/gimbal_camera/image_raw"

ALT = 6.0
HZ = 5.0

HSV_RED1_LOW = (0, 70, 60)
HSV_RED1_HIGH = (10, 255, 255)
HSV_RED2_LOW = (170, 70, 60)
HSV_RED2_HIGH = (180, 255, 255)

HSV_BLUE_LOW = (100, 120, 60)
HSV_BLUE_HIGH = (130, 255, 255)


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


class Vision:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_bgr = None
        self.red_px = 0
        self.blue_px = 0
        rospy.Subscriber(IMAGE_TOPIC, Image, self.cb, queue_size=1)

    def cb(self, msg):
        if msg.encoding.lower() == "rgb8":
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        else:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        r1 = cv2.inRange(hsv, HSV_RED1_LOW, HSV_RED1_HIGH)
        r2 = cv2.inRange(hsv, HSV_RED2_LOW, HSV_RED2_HIGH)
        mask_red = cv2.bitwise_or(r1, r2)
        mask_blue = cv2.inRange(hsv, HSV_BLUE_LOW, HSV_BLUE_HIGH)

        self.red_px = int(cv2.countNonZero(mask_red))
        self.blue_px = int(cv2.countNonZero(mask_blue))
        self.last_bgr = bgr


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
    parser.add_argument("--target", default=MODEL_TARGET)
    args = parser.parse_args()

    rospy.init_node("hover_debug_shape", anonymous=True)

    vehicle = connect(args.connect, wait_ready=True, timeout=60)
    gz = GazeboClient()
    vis = Vision()

    try:
        vehicle.mode = VehicleMode("GUIDED")
        while vehicle.mode.name != "GUIDED":
            time.sleep(0.1)

        vehicle.armed = True
        while not vehicle.armed:
            time.sleep(0.2)

        vehicle.simple_takeoff(ALT)
        time.sleep(6)

        hx, hy = gz.get_xy(MODEL_DRONE)
        tx, ty = gz.get_xy(args.target)

        n = (tx - hx)
        e = -(ty - hy)

        period = 1.0 / HZ
        t0 = time.time()
        while time.time() - t0 < 10:
            goto_ne(vehicle, n, e, ALT)
            rospy.loginfo_throttle(1.0, f"[VISION] red_px={vis.red_px} blue_px={vis.blue_px}")
            time.sleep(period)

        if vis.last_bgr is not None:
            out = f"snapshot_{args.target}.png"
            cv2.imwrite(out, vis.last_bgr)
            print("KAYDEDİLDİ:", out)

    finally:
        try:
            vehicle.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
