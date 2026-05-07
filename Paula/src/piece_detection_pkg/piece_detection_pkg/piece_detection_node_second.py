#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from construccion_interfaces.srv import (
    PixelToRobot,
    DetectLoosePieces
)


class LoosePieceDetectionService(Node):

    def __init__(self):
        super().__init__('loose_piece_detection_service')

        self.bridge = CvBridge()
        self.declare_parameter('image_topic', '/image_raw')
        self.callback_groups()
        self.bg_threshold = 0.08
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10
        )
        self.srv = self.create_service(
            DetectLoosePieces,
            '/detect_loose_pieces',
            self.handle_detect, callback_group=self.srv_callback
        )
        self.pixel_to_robot = self.create_client(
            PixelToRobot,
            '/pixel_to_robot_plane', callback_group=self.pixel_to_robot_callback
        )
        while not self.pixel_to_robot.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calibration service...')
        self.latest_frame = None
        self.background = None
        self.get_logger().info("Detection service READY")

    def callback_groups(self):
        self.srv_callback = MutuallyExclusiveCallbackGroup()
        self.pixel_to_robot_callback = MutuallyExclusiveCallbackGroup()

    # ----------------------------------------
    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e))

    # ----------------------------------------
    async def handle_detect(self, request, response):

        if self.latest_frame is None:
            response.success = False
            response.message = "No image received yet"
            return response

        frame = self.latest_frame.copy()

        # ---- 1. Capture background ----
        if request.capture_background:
            self.background = self.rgb_to_rgi(frame)
            response.success = True
            response.message = "Background captured"
            return response

        # ---- 2. Detect ----
        if request.detect:
            if self.background is None:
                response.success = False
                response.message = "Background not set"
                return response
            self.get_logger().info("I am here detecting")
            fg = self.subtract_background(frame)
            fg = self.clean_mask(fg)
            detections = self.extract_blobs(fg, frame)
            results = []
            self.get_logger().info("I will iterate")
            for det in detections:
                req = PixelToRobot.Request()
                req.u = float(det['cx'])
                req.v = float(det['cy'])
                self.get_logger().info("I will iterate first")
                future = self.pixel_to_robot.call_async(req)
                resultFuture = await future
                if not resultFuture.success:
                    continue
                results.append({
                    "id": det["id"],
                    "pixel": [det["cx"], det["cy"]],
                    "robot": [resultFuture.x, resultFuture.y],
                    "angle": det["angle_deg"],
                    "color": det["color"],
                    "size": det["size_class"]
                })
            self.get_logger().info("I got all the results")
            response.success = True
            response.message = f"{len(results)} pieces detected"
            response.json_result = json.dumps(results)

            return response

        response.success = False
        response.message = "Invalid request"
        return response

    # ----------------------------------------
    def rgb_to_rgi(self, img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)
        R, G, B = rgb[:,:,0], rgb[:,:,1], rgb[:,:,2]
        s = R + G + B + 1e-6
        return np.dstack((R/s, G/s, s/3))

    def subtract_background(self, frame):
        current = self.rgb_to_rgi(frame)
        diff = np.abs(current - self.background)
        mask = (diff[:,:,0] > self.bg_threshold) | (diff[:,:,1] > self.bg_threshold)
        return (mask.astype(np.uint8) * 255)

    def clean_mask(self, mask):
        kernel = np.ones((5,5), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    def extract_blobs(self, mask, frame):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            detections.append({
                "id": i,
                "cx": cx,
                "cy": cy,
                "angle_deg": 0.0,
                "color": "unknown",
                "size_class": "unknown"
            })

        return detections


def main():
    rclpy.init()
    node = LoosePieceDetectionService()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()