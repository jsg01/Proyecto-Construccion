#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class PieceDetectionNode(Node):
    def __init__(self):
        super().__init__('piece_detection_node')
        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/piece_detections')

        self.declare_parameter('bg_threshold', 0.08)

        self.declare_parameter('min_area', 500.0)
        self.declare_parameter('max_area_ratio', 0.20)
        self.declare_parameter('border_margin', 12)
        self.declare_parameter('min_fill_ratio', 0.45)

        self.declare_parameter('closing_kernel', 7)
        self.declare_parameter('opening_kernel', 3)
        self.declare_parameter('median_kernel', 5)

        self.declare_parameter('distance_threshold', 0.40)
        self.declare_parameter('remove_bottom_ratio', 0.85)

        self.declare_parameter('show_debug', True)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.bg_threshold = float(self.get_parameter('bg_threshold').value)

        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area_ratio = float(self.get_parameter('max_area_ratio').value)
        self.border_margin = int(self.get_parameter('border_margin').value)
        self.min_fill_ratio = float(self.get_parameter('min_fill_ratio').value)

        self.closing_kernel = int(self.get_parameter('closing_kernel').value)
        self.opening_kernel = int(self.get_parameter('opening_kernel').value)
        self.median_kernel = int(self.get_parameter('median_kernel').value)

        self.distance_threshold = float(self.get_parameter('distance_threshold').value)
        self.remove_bottom_ratio = float(self.get_parameter('remove_bottom_ratio').value)

        self.show_debug = bool(self.get_parameter('show_debug').value)

        self.background_rgi = None
        self.frame_count = 0
        self.background_capture_frame = 30
        self.background_ready = False

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.publisher = self.create_publisher(String, output_topic, 10)
        self.foreground_publisher = self.create_publisher(Image, "/foreground", 10)

        self.get_logger().info(f'Escuchando imágenes en: {image_topic}')
        self.get_logger().info(f'Publicando detecciones en: {output_topic}')
        self.get_logger().info(
            'Arranca con la escena vacía. El fondo se capturará automáticamente.'
        )
        self.get_logger().info(
            'Pulsa la tecla b en la ventana de OpenCV para recalibrar el fondo manualmente.'
        )

    def rgb_to_rgi(self, bgr_img):
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB).astype(np.float32)

        saturated = np.any(rgb >= 255.0, axis=2)
        rgb[saturated] = 0.0

        R = rgb[:, :, 0]
        G = rgb[:, :, 1]
        B = rgb[:, :, 2]

        s = R + G + B
        s_safe = np.where(s == 0, 1.0, s)

        r = R / s_safe
        g = G / s_safe
        I = s / 3.0

        return np.dstack((r, g, I))

    def capture_background(self, frame_bgr):
        self.background_rgi = self.rgb_to_rgi(frame_bgr)
        self.background_ready = True
        self.get_logger().info('Fondo capturado automáticamente desde la cámara')

    def subtract_background(self, frame_bgr):
        frame_rgi = self.rgb_to_rgi(frame_bgr)

        if self.background_rgi is None:
            h, w = frame_bgr.shape[:2]
            return np.zeros((h, w), dtype=np.uint8)

        if frame_rgi.shape != self.background_rgi.shape:
            self.get_logger().error('La imagen actual y el fondo tienen distinto tamaño')
            h, w = frame_bgr.shape[:2]
            return np.zeros((h, w), dtype=np.uint8)

        diff = np.abs(frame_rgi - self.background_rgi)

        dr = diff[:, :, 0]
        dg = diff[:, :, 1]
        dI = diff[:, :, 2] / 255.0

        fg_mask = (
            (dr > self.bg_threshold) |
            (dg > self.bg_threshold) |
            (dI > self.bg_threshold)
        ).astype(np.uint8) * 255

        return fg_mask

    def clean_mask(self, mask):
        if self.median_kernel >= 3 and self.median_kernel % 2 == 1:
            mask = cv2.medianBlur(mask, self.median_kernel)

        closing_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.closing_kernel, self.closing_kernel)
        )
        opening_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.opening_kernel, self.opening_kernel)
        )

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, closing_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel)

        return mask

    def remove_border_noise(self, mask):
        cleaned = mask.copy()
        h, w = cleaned.shape[:2]

        cut_row = int(h * self.remove_bottom_ratio)
        cleaned[cut_row:, :] = 0

        side_margin = 4
        cleaned[:, :side_margin] = 0
        cleaned[:, w - side_margin:] = 0

        return cleaned

    def split_touching_pieces(self, fg_mask):
        if np.count_nonzero(fg_mask) == 0:
            return fg_mask.copy()

        dist = cv2.distanceTransform(fg_mask, cv2.DIST_L2, 5)
        dist = cv2.normalize(dist, None, 0, 1.0, cv2.NORM_MINMAX)

        _, dist_thresh = cv2.threshold(
            dist, self.distance_threshold, 1.0, cv2.THRESH_BINARY
        )
        dist_thresh = (dist_thresh * 255).astype(np.uint8)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dist_thresh = cv2.morphologyEx(dist_thresh, cv2.MORPH_CLOSE, kernel)

        return dist_thresh

    def classify_size(self, w, h, area):
        longest = max(w, h)
        shortest = min(w, h)
        aspect_ratio = longest / max(shortest, 1)

        if aspect_ratio < 1.4:
            return 'square'
        return 'rectangular'

    def extract_blobs(self, mask):
        detections = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        h_img, w_img = mask.shape[:2]
        img_area = h_img * w_img
        det_id = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            if area > self.max_area_ratio * img_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if (
                x <= self.border_margin or
                y <= self.border_margin or
                (x + w) >= (w_img - self.border_margin) or
                (y + h) >= (h_img - self.border_margin)
            ):
                continue

            aspect_ratio = w / float(max(h, 1))
            if aspect_ratio > 3.5 or aspect_ratio < 0.25:
                continue

            rect_fill = area / float(max(w * h, 1))
            if rect_fill < self.min_fill_ratio:
                continue

            epsilon = 0.02 * cv2.arcLength(cnt, True)
            cnt_smooth = cv2.approxPolyDP(cnt, epsilon, True)

            M = cv2.moments(cnt_smooth)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            rect = cv2.minAreaRect(cnt_smooth)
            (_, _), (rw, rh), angle = rect

            angle_deg = float(angle)
            if rw < rh:
                angle_deg += 90.0

            size_class = self.classify_size(w, h, area)

            detections.append({
                'id': int(det_id),
                'x': int(x),
                'y': int(y),
                'w': int(w),
                'h': int(h),
                'cx': int(cx),
                'cy': int(cy),
                'area': float(area),
                'angle_deg': float(angle_deg),
                'size_class': size_class,
                'contour': cnt_smooth,
            })

            det_id += 1

        return detections

    def draw_detections(self, frame_bgr, detections):
        vis = frame_bgr.copy()

        for det in detections:
            x = det['x']
            y = det['y']
            w = det['w']
            h = det['h']
            cx = det['cx']
            cy = det['cy']
            area = det['area']
            angle_deg = det['angle_deg']
            size_class = det['size_class']
            cnt = det['contour']

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int32(box)

            cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)

            text = f'id={det["id"]} ({cx},{cy}) a={int(area)} ang={angle_deg:.1f} {size_class}'
            cv2.putText(
                vis,
                text,
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        return vis

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        if self.background_rgi is None:
            self.frame_count += 1

            if self.frame_count < self.background_capture_frame:
                if self.frame_count % 10 == 0:
                    self.get_logger().info(
                        f'Esperando para capturar fondo... frame {self.frame_count}/{self.background_capture_frame}'
                    )
                if self.show_debug:
                    cv2.imshow('Detections', frame)
                    cv2.waitKey(1)
                return

            self.capture_background(frame)
            return

        fg_mask = self.subtract_background(frame)
        fg_mask = self.clean_mask(fg_mask)
        fg_mask = self.remove_border_noise(fg_mask)
        
        try:
            msg_out = self.bridge.cv2_to_imgmsg(fg_mask, encoding='mono8')
        except CvBridgeError as e:
            self.get_logger().error(f'Publish conversion failed: {e}')
            return
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = msg.header.frame_id
        self.foreground_publisher.publish(msg_out)

        separated_mask = self.split_touching_pieces(fg_mask)

        detections = self.extract_blobs(separated_mask)
        annotated = self.draw_detections(frame, detections)

        payload = {
            'image_width': int(frame.shape[1]),
            'image_height': int(frame.shape[0]),
            'detections': [
                {
                    'id': det['id'],
                    'x': det['x'],
                    'y': det['y'],
                    'w': det['w'],
                    'h': det['h'],
                    'cx': det['cx'],
                    'cy': det['cy'],
                    'area': det['area'],
                    'angle_deg': det['angle_deg'],
                    'size_class': det['size_class'],
                }
                for det in detections
            ]
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.publisher.publish(out_msg)

        if self.show_debug:
            cv2.imshow('Foreground mask', fg_mask)
            cv2.imshow('Separated mask', separated_mask)
            cv2.imshow('Detections', annotated)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('b'):
                self.capture_background(frame)
                self.get_logger().info('Fondo recalibrado manualmente con la tecla b')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PieceDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
