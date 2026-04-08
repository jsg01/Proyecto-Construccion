#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from . import helper as hp
from custom_interfaces.srv import DetectSpecificColor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from calibration_package.calibration_node import is_valid_color
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PieceDetectionNode(Node):
    def __init__(self):
        """
        Initializes the node for handling an image
        """

        super().__init__('detection_node')
        self.opencv_bridge_ = CvBridge()
        self.callbackgroup_setup()
        self.subscriber_setup()
        self.publisher_setup()
        self.service_setup()
        self.background_threshold = 12 #*This value changes manually
        self.get_logger().info("Detection node fully setup")
        self.yaml_config_file = "/home/ricardo/.ros/color_calibration.yaml"
        self.background_image = "/home/ricardo/Desktop/UPM/LABrobotica/Proyecto-Construccion/LabRob/src/calibration_package/config/background_table.jpg"
        self.current_detection = "blue"

    def callbackgroup_setup(self):
        """
        Initializes all the callback groups in the node
        """

        self.color_detection_group = MutuallyExclusiveCallbackGroup()

    def publisher_setup(self):
        """
        Initializes all the publishers in the node
        """

        self.image_processed_publisher = self.create_publisher(Image, "/image_raw_processed", 10)
    
    def subscriber_setup(self):
        """
        Initializes all the subscribers in the node
        """
        #self.unprocessed_image_subscription = self.create_subscription(Image, "/image_raw", self.image_processor_callback, 10)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.unprocessed_image_subscriber = Subscriber(self, Image, '/image_raw', qos_profile=qos_profile)
        self.foreground_subscriber = Subscriber(self, Image, '/foreground', qos_profile=qos_profile)
        self.subscriber_sync = ApproximateTimeSynchronizer(
            [self.unprocessed_image_subscriber, self.foreground_subscriber],
            queue_size=50,
            slop=1
        )
        self.subscriber_sync.registerCallback(self.image_processor_callback)



    def service_setup(self):
        """
        Initializes all the services in the node
        """

        self.change_current_detection = self.create_service(DetectSpecificColor, "/detection/color", self.change_detection_color, callback_group=self.color_detection_group)
    
    def image_processor_callback(self, rgb_msg: Image, binary_msg: Image):
        """
        Processes the image gotten as an Image message and publishes the processed image back

        ARGS:
            msg (Image): The message received.
        """

        try:
            self.current_bgr = self.opencv_bridge_.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
            self.current_bgr = cv2.cvtColor(self.current_bgr, cv2.COLOR_YUV2BGR_YUY2)
            self.current_background = self.opencv_bridge_.imgmsg_to_cv2(binary_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error('Combination of image messages was not possible : {}'.format(e))
            return
        mask = (self.current_bgr == 255).any(axis = 2)
        self.current_bgr[mask] = 0
        mask2 = ~(self.current_background > 0)
        self.current_bgr[mask2] = 0 
        self.current_bgr, _ = hp.BGR2RGI(self.current_bgr)
        #self.current_bgr, _ = hp.background_removal(self.current_bgr, _, 12, self.background_image)
        foreground = hp.detect_color(self.current_bgr, _, self.yaml_config_file, self.current_detection)
        if not foreground:
            self.get_logger("Foreground is empty")
            return
        [self.current_bgr, processed_image, center_angle] = foreground
        for i in center_angle:
            processed_image = hp.image_angle_editor(processed_image, i)
        try:
            msg_out = self.opencv_bridge_.cv2_to_imgmsg(processed_image, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Publish conversion failed: {e}')
            return
        self.image_processed_publisher.publish(msg_out)
    
    def change_detection_color(self, request, response):
        if not is_valid_color(request.color):
            response.success = False
            response.message = f"Not a valid color, received: {request.color}"
            return response
        self.current_detection = request.color
        response.success = True
        response.message =f"Color successfully changed to: {self.current_detection}"
        return response


        

def main(args=None):
    rclpy.init(args=args)
    node = PieceDetectionNode()
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


