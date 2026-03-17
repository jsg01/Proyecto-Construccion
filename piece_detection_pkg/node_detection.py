#!/usr/bin/env python3
import json
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class PieceDetectionNode(Node):
    def __init__(self):
        super().__init__('piece_detection_node')
        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/piece_detections')
        self.declare_parameter('min_area', 800.0)
        self.declare_parameter('binary_inverted', True)
        self.declare_parameter('show_debug', True)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.min_area = float(self.get_parameter('min_area').value)
        self.binary_inverted = bool(self.get_parameter('binary_inverted').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(String, output_topic, 10)

        self.get_logger().info(f'Escuchando imágenes en: {image_topic}')
        self.get_logger().info(f'Publicando detecciones en: {output_topic}')

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        annotated, mask, detections = self.detect_pieces(frame)

        payload = {
            'image_width': int(frame.shape[1]),
            'image_height': int(frame.shape[0]),
            'detections': detections
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.publisher.publish(out_msg)

        if len(detections) > 0:
         self.get_logger().info(f'{len(detections)} piezas')

        if self.show_debug:
            cv2.imshow('Deteccion de piezas', annotated)
            cv2.imshow('Mascara', mask)
            cv2.waitKey(1)

    def detect_pieces(self, frame: np.ndarray):
        vis = frame.copy()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        thresh_type = cv2.THRESH_BINARY_INV if self.binary_inverted else cv2.THRESH_BINARY
        _, mask = cv2.threshold(blur, 0, 255, thresh_type + cv2.THRESH_OTSU)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2

            rect = cv2.minAreaRect(cnt)
            (rx, ry), (rw, rh), angle = rect

            angle_deg = float(angle)
            if rw < rh:
                angle_deg = angle_deg + 90.0

            size_class = self.classify_piece(w, h, area)

            box = cv2.boxPoints(rect)
            box = np.int32(box)

            cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)

            text = f'{size_class} ({cx},{cy}) a={int(area)} ang={angle_deg:.1f}'
            cv2.putText(
                vis, text, (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
            )

            detections.append({
                'x': int(x),
                'y': int(y),
                'w': int(w),
                'h': int(h),
                'cx': int(cx),
                'cy': int(cy),
                'area': float(area),
                'angle_deg': float(angle_deg),
                'size_class': size_class,
            })

        return vis, mask, detections

    def classify_piece(self, w: int, h: int, area: float) -> str:
        longest = max(w, h)
        shortest = min(w, h)
        aspect_ratio = longest / max(shortest, 1)

        if area < 1800:
            return 'pequena'
        if aspect_ratio > 1.6 and area < 5000:
            return 'alargada_mediana'
        if area < 5000:
            return 'mediana'
        if aspect_ratio > 1.8:
            return 'alargada_grande'
        return 'grande'


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