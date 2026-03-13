# **************************************************************************** #
#                                                                              #
#                                                         :::      ::::::::    #
#    image_capture.py                                   :+:      :+:    :+:    #
#                                                     +:+ +:+         +:+      #
#    By: rortiz <rortiz@student.42madrid.com>       +#+  +:+       +#+         #
#                                                 +#+#+#+#+#+   +#+            #
#    Created: 2026/03/13 03:49:10 by rortiz            #+#    #+#              #
#    Updated: 2026/03/13 04:18:34 by rortiz           ###   ########.fr        #
#                                                                              #
# **************************************************************************** #

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy
import cv2
import os
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ImageHandler(Node):
	def __init__(self):
		super().__init__("image_handler_node")
		self.setup_subscriptions()
	
	def setup_subscriptions(self):
		self.image_processor = self.create_subscription(Image, "/image_raw", self.image_processor_callback, 10)
	
	def image_processor_callback(self, image_msg: Image):
		try:
			logic_image = self.opencv_bridge_.imgmsg_to_cv2(image_msg, desired_encoding="brg8")
		except CvBridgeError as e:
			self.get_logger().error(f"Image msg to Opencv conversion not possible: {e}")

def main():
	rclpy.init()
	image_processor_node = ImageHandler()
	executor = MultiThreadedExecutor()
	rclpy.spin(image_processor_node, executor=executor)
	image_processor_node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()