#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from construccion_interfaces.srv import PixelToRobot, RobotToPixel


class LoosePieceDetectionNode(Node):
    def __init__(self):
        super().__init__('loose_piece_detection_node')
        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/loose_pieces')
        self.declare_parameter('feedback_topic', '/robot_phase_feedback')
        self.declare_parameter('command_topic', '/vision_phase_command')

        self.declare_parameter('detect_pose_name', 'DetectaPiezasSueltas')

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
        self.declare_parameter('save_snapshot', False)
        self.declare_parameter('snapshot_path', '/tmp/loose_pieces_snapshot.png')
        self.declare_parameter('save_annotated', False)
        self.declare_parameter('annotated_path', '/tmp/loose_pieces_annotated.png')

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        command_topic = self.get_parameter('command_topic').value

        self.detect_pose_name = str(self.get_parameter('detect_pose_name').value)

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
        self.save_snapshot = bool(self.get_parameter('save_snapshot').value)
        self.snapshot_path = str(self.get_parameter('snapshot_path').value)
        self.save_annotated = bool(self.get_parameter('save_annotated').value)
        self.annotated_path = str(self.get_parameter('annotated_path').value)

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.feedback_subscription = self.create_subscription(
            String, feedback_topic, self.robot_feedback_callback, 10
        )

        self.publisher = self.create_publisher(String, output_topic, 10)
        self.foreground_publisher = self.create_publisher(Image, '/foreground_loose', 10)
        self.command_publisher = self.create_publisher(String, command_topic, 10)

        # Cliente del servicio pixel -> robot
        self.pixel_to_robot_client = self.create_client(
            PixelToRobot,
            '/pixel_to_robot_plane'
        )

        # Cliente del servicio robot -> pixel
        self.robot_to_pixel_client = self.create_client(
            RobotToPixel,
            '/robot_to_pixel_plane'
        )

        while not self.pixel_to_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /pixel_to_robot_plane...')

        while not self.robot_to_pixel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /robot_to_pixel_plane...')

        self.latest_frame = None
        self.background_rgi = None
        self.background_ready = False
        self.robot_at_detect_pose = False
        self.snapshot_requested = False
        self.snapshot_taken = False
        self.processed_once = False
        self.pose_request_sent = False

        self.publish_go_to_pose(self.detect_pose_name)

        self.get_logger().info(f'Escuchando imágenes en: {image_topic}')
        self.get_logger().info(f'Escuchando feedback robot en: {feedback_topic}')
        self.get_logger().info(f'Publicando piezas sueltas en: {output_topic}')
        self.get_logger().info(f'Solicitada pose inicial: {self.detect_pose_name}')
        self.get_logger().info('Flujo correcto:')
        self.get_logger().info(f'1) El robot va a {self.detect_pose_name}')
        self.get_logger().info('2) Cuando confirme, con la mesa vacía pulsa b')
        self.get_logger().info('3) Eso guarda el fondo vacío')
        self.get_logger().info('4) Coloca las piezas manualmente')
        self.get_logger().info('5) Pulsa p para hacer una foto fija y procesarla una sola vez')

    def publish_command(self, payload_dict):
        msg = String()
        msg.data = json.dumps(payload_dict)
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Comando publicado: {msg.data}')

    def publish_go_to_pose(self, pose_name):
        self.pose_request_sent = True
        self.publish_command({'command': 'go_to_pose', 'pose': pose_name})

    def robot_feedback_callback(self, msg):
        raw = msg.data.strip()

        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            data = {'feedback': raw}

        pose = data.get('pose', '')
        status = data.get('status', '')
        feedback = data.get('feedback', '')

        at_detect_pose = (
            (pose == self.detect_pose_name and status in ('reached', 'ready', 'done')) or
            feedback == f'pose_reached:{self.detect_pose_name}' or
            feedback == f'{self.detect_pose_name}_ready' or
            raw == f'pose_reached:{self.detect_pose_name}' or
            raw == f'{self.detect_pose_name}_ready'
        )

        if at_detect_pose and not self.robot_at_detect_pose:
            self.robot_at_detect_pose = True
            self.get_logger().info(
                f'Robot confirmado en pose {self.detect_pose_name}.'
            )
            self.get_logger().info(
                'Ahora deja la mesa vacía y pulsa b para guardar el fondo.'
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
        self.snapshot_taken = False
        self.processed_once = False
        self.get_logger().info('Fondo vacío capturado correctamente en la pose DetectaPiezasSueltas.')

    def subtract_background(self, frame_bgr):
        frame_rgi = self.rgb_to_rgi(frame_bgr)

        if self.background_rgi is None:
            h, w = frame_bgr.shape[:2]
            return np.zeros((h, w), dtype=np.uint8)

        if frame_rgi.shape != self.background_rgi.shape:
            self.get_logger().error('La imagen actual y el fondo tienen distinto tamaño.')
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

    def classify_color(self, frame_bgr, contour_mask):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        mask = contour_mask > 0
        if not np.any(mask):
            return 'unknown'

        h_vals = hsv[:, :, 0][mask]
        s_vals = hsv[:, :, 1][mask]
        v_vals = hsv[:, :, 2][mask]

        if len(h_vals) == 0:
            return 'unknown'

        med_h = float(np.median(h_vals))
        med_s = float(np.median(s_vals))
        med_v = float(np.median(v_vals))

        if med_v < 40:
            return 'unknown'

        if med_s < 35:
            return 'neutral'

        if med_h < 8 or med_h >= 170:
            return 'red'
        if 8 <= med_h < 22:
            return 'orange'
        if 22 <= med_h < 40:
            return 'yellow'
        if 40 <= med_h < 90:
            return 'green'
        if 90 <= med_h < 135:
            return 'blue'
        if 135 <= med_h < 170:
            return 'pink'

        return 'unknown'

    def extract_blobs(self, mask, frame_bgr):
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

            contour_mask = np.zeros(mask.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [cnt_smooth], -1, 255, thickness=-1)
            color = self.classify_color(frame_bgr, contour_mask)

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
                'color': color,
                'contour': cnt_smooth,
            })

            det_id += 1

        return detections

    def pixel_to_robot(self, u, v):
        req = PixelToRobot.Request()
        req.u = float(u)
        req.v = float(v)

        future = self.pixel_to_robot_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error('El servicio /pixel_to_robot_plane no devolvió respuesta.')
            return None

        if not result.success:
            self.get_logger().error(f'Error en calibración directa: {result.message}')
            return None

        return {
            'x': float(result.x),
            'y': float(result.y),
            'z': float(result.z),
        }

    def robot_to_pixel(self, x, y, z):
        req = RobotToPixel.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)

        future = self.robot_to_pixel_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error('El servicio /robot_to_pixel_plane no devolvió respuesta.')
            return None

        if not result.success:
            self.get_logger().error(f'Error en calibración inversa: {result.message}')
            return None

        return {
            'u': float(result.u),
            'v': float(result.v),
        }

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
            color = det['color']
            cnt = det['contour']

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int32(box)

            cv2.drawContours(vis, [box], 0, (0, 255, 0), 2)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)

            if 'reproj_u' in det and det['reproj_u'] is not None:
                ru = int(round(det['reproj_u']))
                rv = int(round(det['reproj_v']))
                cv2.circle(vis, (ru, rv), 5, (255, 0, 255), 2)
                cv2.line(vis, (cx, cy), (ru, rv), (255, 255, 255), 1)

            text = f'id={det["id"]} {color} {size_class} ({cx},{cy}) a={int(area)} ang={angle_deg:.1f}'
            cv2.putText(
                vis,
                text,
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

            if 'robot_x' in det and det['robot_x'] is not None:
                robot_text = f'({det["robot_x"]:.3f}, {det["robot_y"]:.3f})'
                cv2.putText(
                    vis,
                    robot_text,
                    (x, min(frame_bgr.shape[0] - 10, y + h + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    2
                )

            if 'reproj_error_px' in det and det['reproj_error_px'] is not None:
                error_text = f'err={det["reproj_error_px"]:.1f}px'
                cv2.putText(
                    vis,
                    error_text,
                    (x, min(frame_bgr.shape[0] - 30, y + h + 40)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 255),
                    2
                )

        return vis

    def publish_foreground(self, fg_mask, header):
        try:
            msg_out = self.bridge.cv2_to_imgmsg(fg_mask, encoding='mono8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error convirtiendo foreground: {e}')
            return

        msg_out.header.stamp = header.stamp
        msg_out.header.frame_id = header.frame_id
        self.foreground_publisher.publish(msg_out)

    def process_snapshot(self, frame_bgr, header):
        fg_mask = self.subtract_background(frame_bgr)
        fg_mask = self.clean_mask(fg_mask)
        fg_mask = self.remove_border_noise(fg_mask)
        self.publish_foreground(fg_mask, header)

        separated_mask = self.split_touching_pieces(fg_mask)
        detections = self.extract_blobs(separated_mask, frame_bgr)

        pieces_out = []
        detections_with_robot = []

        for det in detections:
            robot_point = self.pixel_to_robot(det['cx'], det['cy'])

            piece_data = {
                'id': det['id'],
                'x': det['x'],
                'y': det['y'],
                'w': det['w'],
                'h': det['h'],
                'pick_cx': det['cx'],
                'pick_cy': det['cy'],
                'cx': det['cx'],
                'cy': det['cy'],
                'area': det['area'],
                'angle_deg': det['angle_deg'],
                'size_class': det['size_class'],
                'color': det['color'],
            }

            det_with_robot = det.copy()

            if robot_point is not None:
                piece_data['robot_x'] = robot_point['x']
                piece_data['robot_y'] = robot_point['y']
                piece_data['robot_z'] = robot_point['z']

                det_with_robot['robot_x'] = robot_point['x']
                det_with_robot['robot_y'] = robot_point['y']
                det_with_robot['robot_z'] = robot_point['z']

                reprojected_point = self.robot_to_pixel(
                    robot_point['x'],
                    robot_point['y'],
                    robot_point['z']
                )

                if reprojected_point is not None:
                    piece_data['reproj_u'] = reprojected_point['u']
                    piece_data['reproj_v'] = reprojected_point['v']

                    det_with_robot['reproj_u'] = reprojected_point['u']
                    det_with_robot['reproj_v'] = reprojected_point['v']

                    reproj_error = np.hypot(
                        reprojected_point['u'] - det['cx'],
                        reprojected_point['v'] - det['cy']
                    )

                    piece_data['reproj_error_px'] = float(reproj_error)
                    det_with_robot['reproj_error_px'] = float(reproj_error)
                else:
                    piece_data['reproj_u'] = None
                    piece_data['reproj_v'] = None
                    piece_data['reproj_error_px'] = None

                    det_with_robot['reproj_u'] = None
                    det_with_robot['reproj_v'] = None
                    det_with_robot['reproj_error_px'] = None

            else:
                piece_data['robot_x'] = None
                piece_data['robot_y'] = None
                piece_data['robot_z'] = None
                piece_data['reproj_u'] = None
                piece_data['reproj_v'] = None
                piece_data['reproj_error_px'] = None

                det_with_robot['robot_x'] = None
                det_with_robot['robot_y'] = None
                det_with_robot['robot_z'] = None
                det_with_robot['reproj_u'] = None
                det_with_robot['reproj_v'] = None
                det_with_robot['reproj_error_px'] = None

            pieces_out.append(piece_data)
            detections_with_robot.append(det_with_robot)

        annotated = self.draw_detections(frame_bgr, detections_with_robot)

        payload = {
            'phase': 'loose_pieces_ready',
            'image_width': int(frame_bgr.shape[1]),
            'image_height': int(frame_bgr.shape[0]),
            'pieces': pieces_out
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.publisher.publish(out_msg)

        self.get_logger().info(f'Foto procesada. Publicadas {len(pieces_out)} piezas sueltas.')

        if self.save_snapshot:
            cv2.imwrite(self.snapshot_path, frame_bgr)
            self.get_logger().info(f'Foto guardada en: {self.snapshot_path}')

        if self.save_annotated:
            cv2.imwrite(self.annotated_path, annotated)
            self.get_logger().info(f'Imagen anotada guardada en: {self.annotated_path}')

        if self.show_debug:
            cv2.imshow('Loose pieces snapshot', frame_bgr)
            cv2.imshow('Loose pieces foreground', fg_mask)
            cv2.imshow('Loose pieces separated', separated_mask)
            cv2.imshow('Loose pieces detections', annotated)
            cv2.waitKey(1)

        self.snapshot_taken = True
        self.processed_once = True

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        self.latest_frame = frame.copy()

        if self.show_debug:
            preview = frame.copy()

            info_lines = []
            if not self.robot_at_detect_pose:
                info_lines.append(f'Esperando robot en {self.detect_pose_name}')
            elif not self.background_ready:
                info_lines.append('Robot ya esta en pose. Deja mesa vacia y pulsa b')
            elif not self.snapshot_taken:
                info_lines.append('Ahora coloca piezas y pulsa p')
            else:
                info_lines.append('Foto ya procesada. Pulsa r para repetir')

            y0 = 25
            for line in info_lines:
                cv2.putText(
                    preview, line, (10, y0),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )
                y0 += 30

            cv2.imshow('Loose pieces live preview', preview)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('b'):
                if not self.robot_at_detect_pose:
                    self.get_logger().warn(
                        f'No captures el fondo todavia. Primero el robot debe estar en {self.detect_pose_name}.'
                    )
                else:
                    self.capture_background(frame)

            elif key == ord('r'):
                self.snapshot_taken = False
                self.processed_once = False
                self.get_logger().info('Reset hecho. Manteniendo el fondo actual.')

            elif key == ord('p'):
                if not self.robot_at_detect_pose:
                    self.get_logger().warn(
                        f'Todavia no hay confirmacion de que el robot este en {self.detect_pose_name}.'
                    )
                elif not self.background_ready:
                    self.get_logger().warn(
                        'Primero captura el fondo vacio con la tecla b.'
                    )
                else:
                    self.snapshot_requested = True
                    self.get_logger().info('Foto solicitada con tecla p. Procesando frame congelado...')

        if self.snapshot_requested and not self.processed_once:
            self.snapshot_requested = False
            self.process_snapshot(frame.copy(), msg.header)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoosePieceDetectionNode()
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