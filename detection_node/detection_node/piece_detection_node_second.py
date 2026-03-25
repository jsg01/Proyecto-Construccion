#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from . import helper as hp


class PieceDetectionNode(Node):
    def __init__(self):
        """
        Initializes the node for handling an image
        """

        super().__init__('detection_node')
        self.opencv_bridge_ = CvBridge()
        self.subscriber_setup()
        self.publisher_setup()
        self.background_threshold = 12 #*This value changes manually
        self.get_logger().info("Detection node fully setup")

    def publisher_setup(self):
        """
        Initializes all the publishers in the node
        """

        self.image_processed_publisher = self.create_publisher(Image, "/image_raw_processed", 10)
    
    def subscriber_setup(self):
         """
         Initializes all the subscribers in the node
         """

         self.unprocessed_image_subscription = self.create_subscription(Image, "/image_raw", self.image_processor_callback, 10)
    
    def image_processor_callback(self, msg: Image):
        """
        Processes the image gotten as an Image message and publishes the processed image back

        ARGS:
            msg (Image): The message received.
        """

        try:
            self.current_bgr = self.opencv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_bgr = cv2.cvtColor(self.current_bgr, cv2.COLOR_YUV2BGR_YUY2)
        except CvBridgeError as e:
            self.get_logger().error('Image sensor msg to Opencv conversion is not possible : {}'.format(e))
            return
        mask = (self.current_bgr == 255).any(axis = 2)
        self.current_bgr[mask] = 0
        self.current_bgr, _ = hp.BGR2RGI(self.current_bgr)
        self.current_bgr, _ = hp.background_removal(self.current_bgr, _, 12, "/home/ricardo/Desktop/UPM/LABrobotica/Proyecto-Construccion/LabRob/src/calibration_package/config/background_table.jpg")
        #self.current_bgr, _ = hp.foreground_from_rgi(self.current_bgr, _)
        try:
            msg_out = self.opencv_bridge_.cv2_to_imgmsg(_, encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Publish conversion failed: {e}')
            return
        self.image_processed_publisher.publish(msg_out)
        

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


