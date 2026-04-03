#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import os
from ament_index_python.packages import get_package_share_directory
from custom_interfaces.srv import CustomColorDetector
from . import analysis_threshold as analysis
import yaml
from enum import Enum
from detection_node.helper import BGR2RGI

class Colors(Enum):
    ORANGE = "orange"
    BLUE = "blue"
    WHITE = "white"
    GREEN = "green"
    RED = "red"
    YELLOW = "yellow"

def is_valid_color(color_str: str) -> bool:
    try:
        Colors(color_str.lower())
        return True
    except ValueError:
        return False

class CalibrationNode(Node):
    def __init__(self):
        """
        Initializes the node for handling an image
        """

        super().__init__('calibration_node')
        self.opencv_bridge_ = CvBridge()
        self.call_back_groups_setup()
        self.subscribers_setup()
        self.services_setup()
        self.current_image = None
        self.get_logger().info("calibration node fully setup")
    
    def call_back_groups_setup(self):
        self.taking_picture_group = MutuallyExclusiveCallbackGroup()
    
    def subscribers_setup(self):
        self.unprocessed_image_subscription = self.create_subscription(Image, "/image_raw", self.get_image_from_camera, 10, callback_group=self.taking_picture_group)
    
    def services_setup(self):
        self.capture_image = self.create_service(Trigger, "/calibration/picture", self.take_picture, callback_group = self.taking_picture_group)
        self.select_roi = self.create_service(CustomColorDetector, "/calibration/SelectCustomColor", self.get_color_thresholds, callback_group = self.taking_picture_group)

    def load_calibration_image(self):
        pkg_share = get_package_share_directory("calibration_package")
        file_path = os.path.join(pkg_share, "config", "sample_colors2.jpg")
        image = cv2.imread(file_path)
        if image is None:
            raise ValueError(f"Image not found at {file_path}")
        return image
    
    def save_yaml(self, yaml_path: str, new_data: dict):
        if os.path.exists(yaml_path):
            with open(yaml_path, "r") as f:
                existing_data = yaml.safe_load(f) or {}
        else:
            existing_data = {}
        existing_data.update(new_data)
        with open(yaml_path, "w") as f:
            yaml.dump(existing_data, f, sort_keys = False)
    
    def color_entry(self, color: str, roi: tuple, thresholds: dict):
        return {
            color : {
                "x": int(roi[0]),
                "y": int(roi[1]),
                "w": int(roi[2]),
                "h": int(roi[3]),
            } | thresholds
        }

    def get_image_from_camera(self, msg: Image):
        try:
            self.current_bgr = self.opencv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_bgr = cv2.cvtColor(self.current_bgr, cv2.COLOR_YUV2BGR_YUY2)
        except CvBridgeError as e:
            self.get_logger().error('Image sensor msg to Opencv conversion is not possible : {}'.format(e))
            return
        self.current_image = self.current_bgr
    
    def take_picture(self, request, response):
        pkg_share = get_package_share_directory("calibration_package")
        config_path = os.path.join(pkg_share, "config")
        file_path = os.path.join(config_path, "picture_taken.jpg")
        while (self.current_image is None):
            self.get_logger().info("No image received yet!")
        response.success = cv2.imwrite(file_path, self.current_image)
        if response.success == True:
            response.message = "Image properly taken"
        else:
            response.message = "Error while taking picture"
        return response
    
    def get_color_thresholds(self, request, response):
        if not is_valid_color(request.color):
            response.success = False
            response.message = "Please insert a valid color"
            return response
        try:
            calibration_image = self.load_calibration_image()
            roi_coordinate = analysis.select_roi(calibration_image)
            rgi_scene, _ = BGR2RGI(calibration_image)
            thresholds = analysis.compute_thresholds(rgi_scene, roi_coordinate, 5, 95)
            yaml_data = self.color_entry(request.color, roi_coordinate, thresholds)
            yaml_path = os.path.expanduser("~/.ros/color_calibration.yaml")
            #with open(yaml_path, "w") as f:
            #    yaml.safe_dump(yaml_data, f, sort_keys=False)
            self.save_yaml(yaml_path, yaml_data)
            self.get_logger().info(f"ROI saved to {yaml_path}")
            response.x, response.y, response.w, response.h = roi_coordinate
            response.success = True
            response.message = f"Coordinates gotten: x={response.x}, y={response.y}, w={response.w}, h={response.h}"
        except Exception as e:
            response.message = f"Error during roi selection: {e}"
            response.success = False
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






        
