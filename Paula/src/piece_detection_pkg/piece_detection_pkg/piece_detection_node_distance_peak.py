#!/usr/bin/env python3
import json
import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import yaml
from construccion_interfaces.srv import PixelToRobot, RobotToPixel


class LoosePieceDetectionNode(Node):
    def __init__(self):
        super().__init__('loose_piece_detection_node')
        self.bridge = CvBridge()
        cv2.namedWindow('Loose pieces live preview', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Loose pieces live preview', 1920, 1080)
        cv2.namedWindow('Loose pieces snapshot', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Loose pieces snapshot', 1920, 1080)
        cv2.namedWindow('Loose pieces foreground', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Loose pieces foreground', 1920, 1080)
        cv2.namedWindow('Loose pieces separated', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Loose pieces separated', 1920, 1080)
        cv2.namedWindow('Loose pieces detections', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Loose pieces detections', 1920, 1080)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/loose_pieces')
        self.declare_parameter('feedback_topic', '/robot_phase_feedback')
        self.declare_parameter('command_topic', '/vision_phase_command')
        self.declare_parameter('detect_pose_name', 'DetectaPiezasSueltas')

        self.declare_parameter('bg_threshold', 0.08)
        self.declare_parameter('color_threshold', 0.045)
        self.declare_parameter('min_saturation', 45)
        self.declare_parameter('min_value', 35)
        self.declare_parameter('shadow_rejection', True)
        self.declare_parameter('shadow_chroma_threshold', 0.035)
        self.declare_parameter('shadow_darkening_threshold', 0.025)

        self.declare_parameter('min_area', 3000.0)
        self.declare_parameter('max_area_ratio', 0.20)
        self.declare_parameter('border_margin', 100)
        self.declare_parameter('min_fill_ratio', 0.45)

        self.declare_parameter('closing_kernel', 7)
        self.declare_parameter('opening_kernel', 3)
        self.declare_parameter('median_kernel', 5)

        self.declare_parameter('distance_threshold', 0.40)
        self.declare_parameter('center_method', 'distance_peak')
        self.declare_parameter('distance_peak_ratio', 0.82)
        self.declare_parameter('stud_min_radius', 12)
        self.declare_parameter('stud_max_radius', 24)
        self.declare_parameter('stud_min_distance', 30)
        self.declare_parameter('stud_hough_param1', 80)
        self.declare_parameter('stud_hough_param2', 10)
        self.declare_parameter('stud_valid_counts', [4, 8])
        self.declare_parameter('stud_roi_padding', 8)
        self.declare_parameter('stud_radius_tolerance_ratio', 0.35)
        self.declare_parameter('top_face_value_percentile', 58.0)
        self.declare_parameter('top_face_min_area_ratio', 0.15)
        self.declare_parameter('top_face_erode_kernel', 7)
        self.declare_parameter('camera_calibration_path', '/home/ricardo/Desktop/LabRobot/Paula/src/config/camera_intrinsics.yaml')
        self.declare_parameter('use_undistort', False)
        self.declare_parameter('remove_bottom_ratio', 0.85)

        self.declare_parameter('show_debug', True)
        self.declare_parameter('save_snapshot', False)
        self.declare_parameter('snapshot_path', '/tmp/loose_pieces_snapshot.png')
        self.declare_parameter('save_annotated', True)
        self.declare_parameter('annotated_path', '/tmp/distance_peak_082.png')

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value
        command_topic = self.get_parameter('command_topic').value

        self.detect_pose_name = str(self.get_parameter('detect_pose_name').value)

        self.bg_threshold = float(self.get_parameter('bg_threshold').value)
        self.color_threshold = float(self.get_parameter('color_threshold').value)
        self.min_saturation = int(self.get_parameter('min_saturation').value)
        self.min_value = int(self.get_parameter('min_value').value)
        self.shadow_rejection = bool(self.get_parameter('shadow_rejection').value)
        self.shadow_chroma_threshold = float(self.get_parameter('shadow_chroma_threshold').value)
        self.shadow_darkening_threshold = float(self.get_parameter('shadow_darkening_threshold').value)

        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area_ratio = float(self.get_parameter('max_area_ratio').value)
        self.border_margin = int(self.get_parameter('border_margin').value)
        self.min_fill_ratio = float(self.get_parameter('min_fill_ratio').value)

        self.closing_kernel = int(self.get_parameter('closing_kernel').value)
        self.opening_kernel = int(self.get_parameter('opening_kernel').value)
        self.median_kernel = int(self.get_parameter('median_kernel').value)

        self.distance_threshold = float(self.get_parameter('distance_threshold').value)
        self.center_method = str(self.get_parameter('center_method').value)
        self.distance_peak_ratio = float(self.get_parameter('distance_peak_ratio').value)
        if self.center_method not in ('studs', 'min_area_rect', 'moments', 'bbox', 'top_face_moments', 'distance_peak'):
            self.get_logger().warn(
                f'center_method={self.center_method} no valido. Usando distance_peak.'
            )
            self.center_method = 'distance_peak'
        self.stud_min_radius = int(self.get_parameter('stud_min_radius').value)
        self.stud_max_radius = int(self.get_parameter('stud_max_radius').value)
        self.stud_min_distance = int(self.get_parameter('stud_min_distance').value)
        self.stud_hough_param1 = float(self.get_parameter('stud_hough_param1').value)
        self.stud_hough_param2 = float(self.get_parameter('stud_hough_param2').value)
        raw_valid_counts = self.get_parameter('stud_valid_counts').value
        self.stud_valid_counts = set(int(v) for v in raw_valid_counts)
        if not self.stud_valid_counts:
            self.stud_valid_counts = {4, 8}
        self.stud_roi_padding = int(self.get_parameter('stud_roi_padding').value)
        self.stud_radius_tolerance_ratio = float(self.get_parameter('stud_radius_tolerance_ratio').value)
        self.top_face_value_percentile = float(self.get_parameter('top_face_value_percentile').value)
        self.top_face_min_area_ratio = float(self.get_parameter('top_face_min_area_ratio').value)
        self.top_face_erode_kernel = int(self.get_parameter('top_face_erode_kernel').value)
        self.camera_calibration_path = str(self.get_parameter('camera_calibration_path').value)
        self.use_undistort = bool(self.get_parameter('use_undistort').value)
        self.remove_bottom_ratio = float(self.get_parameter('remove_bottom_ratio').value)

        self.show_debug = bool(self.get_parameter('show_debug').value)
        self.save_snapshot = bool(self.get_parameter('save_snapshot').value)
        self.snapshot_path = str(self.get_parameter('snapshot_path').value)
        self.save_annotated = bool(self.get_parameter('save_annotated').value)
        self.annotated_path = str(self.get_parameter('annotated_path').value)

        self.load_camera_calibration()

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.feedback_subscription = self.create_subscription(
            String, feedback_topic, self.robot_feedback_callback, 10
        )
       

        self.publisher = self.create_publisher(String, output_topic, 10)
        self.foreground_publisher = self.create_publisher(Image, '/foreground_loose', 10)
        self.command_publisher = self.create_publisher(String, command_topic, 10)

        self.pixel_to_robot_client = self.create_client(
            PixelToRobot,
            '/pixel_to_robot_plane'
        )
        self.robot_to_pixel_client = self.create_client(
            RobotToPixel,
            '/robot_to_pixel_plane'
        )

        while not self.pixel_to_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /pixel_to_robot_plane...')

        while not self.robot_to_pixel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /robot_to_pixel_plane...')

        self.latest_frame = None
        self.latest_header = None

        self.background_rgi = None
        self.background_ready = False
        self.robot_at_detect_pose = False
        self.snapshot_requested = False
        self.snapshot_taken = False
        self.processed_once = False
        self.pose_request_sent = False
        self.processing_snapshot = False

        self.pending_frame_bgr = None
        self.pending_header = None
        self.pending_fg_mask = None
        self.pending_separated_mask = None
        self.pending_detections = []
        self.pending_index = 0
        self.pending_pieces_out = []
        self.pending_detections_with_robot = []

        self.current_piece_data = None
        self.current_det_with_robot = None

        self.timer = self.create_timer(0.1, self.timer_callback)

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
            self.get_logger().info(f'Robot confirmado en pose {self.detect_pose_name}.')
            self.get_logger().info('Ahora deja la mesa vacía y pulsa b para guardar el fondo.')

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
        self.get_logger().info(
            'Fondo vacío capturado correctamente en la pose DetectaPiezasSueltas.'
        )

    def subtract_background(self, frame_bgr):
        """
        Segmenta las piezas evitando que las sombras entren como objeto.

        En la versión anterior se aceptaba cualquier cambio fuerte de intensidad (dI).
        Una sombra cambia mucho la intensidad, pero cambia poco la cromaticidad y suele
        tener saturación baja. Por eso aquí se prioriza:
        - cambio de color normalizado R/G;
        - saturación HSV alta, típica de las piezas;
        - y se rechazan cambios que son solo oscurecimiento.
        """
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
        chroma_diff = np.maximum(dr, dg)

        frame_I = frame_rgi[:, :, 2]
        bg_I = self.background_rgi[:, :, 2]
        intensity_delta = (frame_I - bg_I) / 255.0
        dI = np.abs(intensity_delta)

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        # Las piezas son de colores vivos; el papel y la sombra tienen poca saturación.
        color_mask = (sat >= self.min_saturation) & (val >= self.min_value)

        # Cambio real de color respecto al fondo.
        chroma_mask = chroma_diff > self.color_threshold

        # Solo dejamos cambios de intensidad si no parecen una sombra pura.
        intensity_mask = dI > self.bg_threshold

        shadow_like = (
            (intensity_delta < -self.shadow_darkening_threshold) &
            (chroma_diff < self.shadow_chroma_threshold) &
            (sat < self.min_saturation)
        )

        fg = (color_mask | chroma_mask | intensity_mask)
        if self.shadow_rejection:
            fg = fg & (~shadow_like)

        fg_mask = fg.astype(np.uint8) * 255
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

    def split_touching_pieces(self, fg_mask, frame_bgr=None):
        """
        Crea una máscara de etiquetas por pieza conservando la silueta completa.

        En la versión anterior se devolvía solo el núcleo de cada pieza obtenido con
        distanceTransform. Eso ayudaba a separar, pero hacía que el centro se calculase
        sobre una pieza erosionada, no sobre la pieza real.

        Ahora:
        1) se calculan semillas con distanceTransform;
        2) se expanden con watershed sobre la imagen original;
        3) se devuelve labels, donde 0=fondo y 1..N=piezas.
        """
        binary = (fg_mask > 0).astype(np.uint8)
        if np.count_nonzero(binary) == 0:
            labels = np.zeros_like(binary, dtype=np.int32)
            debug = np.zeros_like(binary, dtype=np.uint8)
            return labels, debug

        dist = cv2.distanceTransform((binary * 255).astype(np.uint8), cv2.DIST_L2, 5)
        max_dist = float(dist.max())
        if max_dist <= 0.0:
            labels = np.zeros_like(binary, dtype=np.int32)
            debug = np.zeros_like(binary, dtype=np.uint8)
            return labels, debug

        dist_norm = dist / max_dist
        sure_fg = (dist_norm > self.distance_threshold).astype(np.uint8) * 255

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        sure_fg = cv2.morphologyEx(sure_fg, cv2.MORPH_CLOSE, kernel)
        sure_fg = cv2.morphologyEx(sure_fg, cv2.MORPH_OPEN, kernel)

        num_seed_labels, markers = cv2.connectedComponents(sure_fg)

        # Si no hay semillas válidas, caemos a componentes conectados de la máscara limpia.
        if num_seed_labels <= 1:
            _, cc_labels = cv2.connectedComponents(binary)
            debug = np.zeros_like(binary, dtype=np.uint8)
            if cc_labels.max() > 0:
                debug = ((cc_labels.astype(np.float32) / cc_labels.max()) * 255).astype(np.uint8)
            return cc_labels.astype(np.int32), debug

        # Watershed necesita background=0, objetos marcados con valores positivos.
        markers = markers.astype(np.int32) + 1
        markers[binary == 0] = 0

        if frame_bgr is None:
            ws_input = cv2.cvtColor(fg_mask, cv2.COLOR_GRAY2BGR)
        else:
            ws_input = frame_bgr.copy()

        ws_markers = cv2.watershed(ws_input, markers)

        labels = np.zeros_like(ws_markers, dtype=np.int32)
        labels[ws_markers >= 2] = ws_markers[ws_markers >= 2] - 1

        # El borde -1 del watershed queda vacío. Lo cerramos suavemente para no perder silueta.
        labels_closed = np.zeros_like(labels, dtype=np.int32)
        next_label = 1
        for lab in sorted(int(v) for v in np.unique(labels) if v > 0):
            part = (labels == lab).astype(np.uint8) * 255
            part = cv2.morphologyEx(part, cv2.MORPH_CLOSE, kernel)
            labels_closed[part > 0] = next_label
            next_label += 1

        labels = labels_closed
        debug = np.zeros_like(binary, dtype=np.uint8)
        if labels.max() > 0:
            debug = ((labels.astype(np.float32) / labels.max()) * 255).astype(np.uint8)

        return labels, debug

    def choose_piece_center(self, cnt, x, y, w, h):
        """Devuelve el punto de agarre en píxeles usando el método configurado."""
        rect = cv2.minAreaRect(cnt)
        (rect_cx, rect_cy), _, _ = rect

        M = cv2.moments(cnt)
        if M['m00'] != 0:
            moment_cx = M['m10'] / M['m00']
            moment_cy = M['m01'] / M['m00']
        else:
            moment_cx = rect_cx
            moment_cy = rect_cy

        bbox_cx = x + w / 2.0
        bbox_cy = y + h / 2.0

        if self.center_method == 'distance_peak':
            # Centro robusto por transformada de distancia: se queda con la zona más interior
            # de la cara superior. Los laterales/sombras pegados al borde casi no influyen,
            # porque tienen poca distancia al contorno.
            local = np.zeros((max(1, h), max(1, w)), dtype=np.uint8)
            cnt_shifted = cnt.copy().astype(np.int32)
            cnt_shifted[:, 0, 0] -= int(x)
            cnt_shifted[:, 0, 1] -= int(y)
            cv2.drawContours(local, [cnt_shifted], -1, 255, thickness=-1)

            dist = cv2.distanceTransform(local, cv2.DIST_L2, 5)
            max_dist = float(dist.max())
            if max_dist > 0.0:
                ratio = float(np.clip(self.distance_peak_ratio, 0.55, 0.98))
                peak = dist >= (ratio * max_dist)
                ys, xs = np.where(peak)
                if xs.size > 0:
                    weights = dist[ys, xs].astype(np.float64)
                    sw = float(weights.sum())
                    if sw > 0.0:
                        cx = float(x) + float((xs * weights).sum() / sw)
                        cy = float(y) + float((ys * weights).sum() / sw)
                    else:
                        cx = float(x) + float(xs.mean())
                        cy = float(y) + float(ys.mean())
                else:
                    cx, cy = moment_cx, moment_cy
            else:
                cx, cy = moment_cx, moment_cy
        elif self.center_method in ('moments', 'top_face_moments'):
            cx, cy = moment_cx, moment_cy
        elif self.center_method == 'bbox':
            cx, cy = bbox_cx, bbox_cy
        else:
            # Para piezas cuadradas/rectangulares rígidas, este suele ser el centro geométrico
            # más estable porque usa la caja rotada de la silueta completa.
            cx, cy = rect_cx, rect_cy

        return cx, cy, rect_cx, rect_cy, moment_cx, moment_cy, bbox_cx, bbox_cy

    def detect_studs(self, frame_bgr, piece_mask, x, y, w, h):
        """
        Detecta los tetones/círculos superiores de una pieza tipo LEGO dentro de
        la región de la pieza. El objetivo no es segmentar toda la pieza, sino
        usar esos círculos como referencia geométrica estable para el centro.
        """
        h_img, w_img = frame_bgr.shape[:2]
        pad = max(0, int(self.stud_roi_padding))
        x0 = max(0, int(x) - pad)
        y0 = max(0, int(y) - pad)
        x1 = min(w_img, int(x + w) + pad)
        y1 = min(h_img, int(y + h) + pad)

        if x1 <= x0 or y1 <= y0:
            return []

        roi = frame_bgr[y0:y1, x0:x1]
        mask_roi = piece_mask[y0:y1, x0:x1]
        if roi.size == 0 or np.count_nonzero(mask_roi) == 0:
            return []

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_CLOSE, kernel)
        mask_roi = cv2.dilate(mask_roi, kernel, iterations=1)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # Evita que el borde de la hoja/mesa participe en HoughCircles.
        inside_vals = gray[mask_roi > 0]
        fill_value = int(np.median(inside_vals)) if len(inside_vals) else 0
        gray_masked = gray.copy()
        gray_masked[mask_roi == 0] = fill_value

        min_radius = max(3, int(self.stud_min_radius))
        max_radius = max(min_radius + 1, int(self.stud_max_radius))
        min_dist = max(5, int(self.stud_min_distance))

        circles = cv2.HoughCircles(
            gray_masked,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=min_dist,
            param1=float(self.stud_hough_param1),
            param2=float(self.stud_hough_param2),
            minRadius=min_radius,
            maxRadius=max_radius,
        )

        if circles is None:
            return []

        circles = np.round(circles[0, :]).astype(int)
        dist_to_border = cv2.distanceTransform((mask_roi > 0).astype(np.uint8), cv2.DIST_L2, 5)

        candidates = []
        for cx_l, cy_l, r in circles:
            if cx_l < 0 or cy_l < 0 or cx_l >= mask_roi.shape[1] or cy_l >= mask_roi.shape[0]:
                continue
            if mask_roi[cy_l, cx_l] == 0:
                continue

            # Un tetón está dentro de la cara superior, no pegado al contorno externo.
            if dist_to_border[cy_l, cx_l] < max(2.0, 0.30 * float(r)):
                continue

            cx_g = int(cx_l + x0)
            cy_g = int(cy_l + y0)
            candidates.append({'cx': cx_g, 'cy': cy_g, 'r': int(r)})

        if not candidates:
            return []

        # Supresión de duplicados: Hough puede devolver varios círculos casi iguales.
        candidates = sorted(candidates, key=lambda c: (c['cy'], c['cx'], c['r']))
        kept = []
        duplicate_dist = max(6.0, 0.55 * float(min_dist))
        for c in candidates:
            duplicate = False
            for k in kept:
                if np.hypot(c['cx'] - k['cx'], c['cy'] - k['cy']) < duplicate_dist:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(c)

        # Si Hough ha encontrado demasiados círculos, nos quedamos con los más centrales
        # dentro de la pieza para evitar bordes/reflejos falsos.
        if len(kept) > 10:
            rect_center = np.array([x + w / 2.0, y + h / 2.0], dtype=np.float32)
            kept = sorted(
                kept,
                key=lambda c: np.hypot(c['cx'] - rect_center[0], c['cy'] - rect_center[1])
            )[:10]

        return kept

    def center_from_studs(self, studs):
        """Calcula el centro de la pieza a partir de los tetones detectados."""
        pts = np.array([[s['cx'], s['cy']] for s in studs], dtype=np.float32)
        if len(pts) >= 3:
            rect = cv2.minAreaRect(pts.reshape(-1, 1, 2))
            (cx, cy), _, angle = rect
            return float(cx), float(cy), float(angle)

        # Con menos de 3 puntos no es suficientemente fiable para una pieza LEGO.
        mean = pts.mean(axis=0)
        return float(mean[0]), float(mean[1]), None


    def estimate_top_face_mask(self, frame_bgr, piece_mask):
        """
        Intenta quedarse solo con la cara superior de la pieza.
        La máscara de foreground puede incluir laterales y sombra; para el centro
        de una pieza LEGO interesa la cara de arriba, que suele ser la zona más
        iluminada y saturada dentro de la propia pieza.
        """
        if piece_mask is None or np.count_nonzero(piece_mask) == 0:
            return piece_mask, None

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        s = hsv[:, :, 1]
        v = hsv[:, :, 2]

        full = piece_mask > 0
        full_area = int(np.count_nonzero(full))
        if full_area < 10:
            return piece_mask, None

        sat_min = max(25, int(self.min_saturation) - 10)
        valid_color = full & (s >= sat_min) & (v >= int(self.min_value))
        if np.count_nonzero(valid_color) < 0.20 * full_area:
            valid_color = full

        vals_v = v[valid_color]
        if vals_v.size == 0:
            return piece_mask, None

        pct = float(np.clip(self.top_face_value_percentile, 35.0, 85.0))
        v_thr = float(np.percentile(vals_v, pct))

        # Cara superior: pixels con color de pieza y suficientemente claros.
        # Esto suele eliminar el lateral oscuro y mucha sombra.
        top = full & (s >= sat_min) & (v >= v_thr)

        # Si se queda demasiado pequeña, relajamos el umbral de brillo.
        if np.count_nonzero(top) < self.top_face_min_area_ratio * full_area:
            v_thr = float(np.percentile(vals_v, 40.0))
            top = full & (s >= max(20, sat_min - 10)) & (v >= v_thr)

        top_u8 = top.astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        top_u8 = cv2.morphologyEx(top_u8, cv2.MORPH_CLOSE, kernel, iterations=2)
        top_u8 = cv2.morphologyEx(top_u8, cv2.MORPH_OPEN, kernel, iterations=1)

        # Paso clave: quitamos una franja exterior de la máscara de color.
        # Los laterales y las sombras suelen estar pegados al borde de la silueta;
        # al erosionar antes de calcular el centro, dejan de tirar del punto central.
        erode_k = int(self.top_face_erode_kernel)
        if erode_k >= 3:
            if erode_k % 2 == 0:
                erode_k += 1
            erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_k, erode_k))
            eroded = cv2.erode(top_u8, erode_kernel, iterations=1)
            min_eroded_area = max(35, int(0.08 * full_area))
            if np.count_nonzero(eroded) >= min_eroded_area:
                top_u8 = eroded

        contours, _ = cv2.findContours(top_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return piece_mask, None

        cnt = max(contours, key=cv2.contourArea)
        top_area = float(cv2.contourArea(cnt))
        if top_area < self.top_face_min_area_ratio * float(full_area):
            return piece_mask, None

        clean = np.zeros_like(piece_mask, dtype=np.uint8)
        cv2.drawContours(clean, [cnt], -1, 255, thickness=-1)
        return clean, cnt

    def choose_valid_studs(self, studs, top_cnt=None):
        """
        HoughCircles puede devolver círculos falsos: brillos, bordes interiores o
        círculos repetidos. Aquí filtramos por radio y por cantidad esperada.
        Después de este filtrado, solo se acepta exactamente 4 u 8.
        """
        if not studs:
            return []

        studs = list(studs)
        radii = np.array([float(s.get('r', 0)) for s in studs], dtype=np.float32)
        if radii.size == 0:
            return []

        # Quita círculos pequeños/grandes respecto al radio dominante.
        # Usamos la mediana para que 1 o 2 falsos no manden.
        med_r = float(np.median(radii))
        tol = max(2.5, self.stud_radius_tolerance_ratio * med_r)
        filtered = [s for s in studs if abs(float(s.get('r', 0)) - med_r) <= tol]

        # Si hay demasiados pequeños y la mediana cae mal, probamos quedarnos con
        # radios grandes, que suelen corresponder al borde completo del tetón.
        if len(filtered) not in self.stud_valid_counts and len(studs) > 0:
            sorted_by_r = sorted(studs, key=lambda s: float(s.get('r', 0)), reverse=True)
            for n in sorted(self.stud_valid_counts, reverse=True):
                if len(sorted_by_r) >= n:
                    candidate = sorted_by_r[:n]
                    rr = np.array([float(s.get('r', 0)) for s in candidate], dtype=np.float32)
                    if rr.max() - rr.min() <= max(5.0, 0.45 * float(np.median(rr))):
                        filtered = candidate
                        break

        # Decide si esperamos pieza 2x2 o 2x4 usando la caja rotada de la cara superior.
        expected = None
        if top_cnt is not None and len(top_cnt) >= 3:
            (_, _), (rw, rh), _ = cv2.minAreaRect(top_cnt)
            longest = max(float(rw), float(rh))
            shortest = max(1.0, min(float(rw), float(rh)))
            aspect = longest / shortest
            expected = 8 if aspect > 1.55 else 4

        if expected in self.stud_valid_counts and len(filtered) >= expected:
            # Si hay más de los esperados, nos quedamos con los de mayor radio.
            filtered = sorted(filtered, key=lambda s: float(s.get('r', 0)), reverse=True)[:expected]

        if len(filtered) in self.stud_valid_counts:
            # Orden estable para dibujar y guardar.
            return sorted(filtered, key=lambda s: (s['cy'], s['cx']))

        return []

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

    def extract_blobs(self, labels, frame_bgr):
        detections = []

        h_img, w_img = labels.shape[:2]
        img_area = h_img * w_img
        det_id = 0

        label_values = [int(v) for v in np.unique(labels) if int(v) > 0]

        for lab in label_values:
            piece_mask = np.zeros(labels.shape[:2], dtype=np.uint8)
            piece_mask[labels == lab] = 255

            contours, _ = cv2.findContours(
                piece_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            # Si una etiqueta queda partida por ruido, nos quedamos con la silueta principal.
            cnt = max(contours, key=cv2.contourArea)
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

            epsilon = 0.01 * cv2.arcLength(cnt, True)
            cnt_smooth = cv2.approxPolyDP(cnt, epsilon, True)

            rect = cv2.minAreaRect(cnt)
            (_, _), (rw, rh), angle = rect

            angle_deg = float(angle)
            if rw < rh:
                angle_deg += 90.0

            contour_mask = np.zeros(labels.shape[:2], dtype=np.uint8)
            cv2.drawContours(contour_mask, [cnt], -1, 255, thickness=-1)

            top_mask, top_cnt = self.estimate_top_face_mask(frame_bgr, contour_mask)
            center_cnt = top_cnt if top_cnt is not None else cnt
            tx, ty, tw, th = cv2.boundingRect(center_cnt)

            studs_raw = self.detect_studs(frame_bgr, top_mask, tx, ty, tw, th)
            studs = self.choose_valid_studs(studs_raw, top_cnt=center_cnt)

            (
                fallback_cx, fallback_cy,
                rect_cx, rect_cy,
                moment_cx, moment_cy,
                bbox_cx, bbox_cy,
            ) = self.choose_piece_center(center_cnt, tx, ty, tw, th)

            center_method_used = self.center_method
            stud_angle = None
            studs_count = len(studs)
            studs_raw_count = len(studs_raw)
            if self.center_method == 'studs':
                if studs_count in self.stud_valid_counts:
                    cx_f, cy_f, stud_angle = self.center_from_studs(studs)
                    center_method_used = f'studs_{studs_count}'
                else:
                    cx_f, cy_f = fallback_cx, fallback_cy
                    center_method_used = f'top_face_fallback_raw{studs_raw_count}' if top_cnt is not None else f'fallback_raw{studs_raw_count}'
            else:
                cx_f, cy_f = fallback_cx, fallback_cy

            cx = int(round(cx_f))
            cy = int(round(cy_f))

            size_class = self.classify_size(w, h, area)
            if studs_count == 8:
                size_class = 'rectangular'
            elif studs_count == 4:
                size_class = 'square'

            color = self.classify_color(frame_bgr, contour_mask)

            detections.append({
                'id': int(det_id),
                'label': int(lab),
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
                'center_method': center_method_used,
                'studs_count': int(len(studs)),
                'studs_raw_count': int(len(studs_raw)),
                'top_face_used': bool(top_cnt is not None),
                'top_contour': None if top_cnt is None else top_cnt,
                'stud_angle_deg': None if stud_angle is None else float(stud_angle),
                'studs': studs,
                'rect_cx': float(rect_cx),
                'rect_cy': float(rect_cy),
                'moment_cx': float(moment_cx),
                'moment_cy': float(moment_cy),
                'bbox_cx': float(bbox_cx),
                'bbox_cy': float(bbox_cy),
                'contour': cnt_smooth,
            })
            det_id += 1

        return detections

    def on_pixel_to_robot_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Error llamando a /pixel_to_robot_plane: {e}')
            self.store_current_detection_and_continue()
            return

        if result is None:
            self.get_logger().error('El servicio /pixel_to_robot_plane no devolvió respuesta.')
            self.store_current_detection_and_continue()
            return

        if not result.success:
            self.get_logger().error(f'Error en calibración: {result.message}')
            self.store_current_detection_and_continue()
            return

        self.current_piece_data['robot_x'] = float(result.x)
        self.current_piece_data['robot_y'] = float(result.y)

        self.current_det_with_robot['robot_x'] = float(result.x)
        self.current_det_with_robot['robot_y'] = float(result.y)

        self.get_logger().info(
            f'PixelToRobot: pixel=({self.current_piece_data["cx"]},{self.current_piece_data["cy"]}) '
            f'-> robot=({result.x:.6f}, {result.y:.6f})'
        )

        req = RobotToPixel.Request()
        req.x = float(result.x)
        req.y = float(result.y)
        
        future2 = self.robot_to_pixel_client.call_async(req)
        future2.add_done_callback(self.on_robot_to_pixel_done)

    def on_robot_to_pixel_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Error llamando a /robot_to_pixel_plane: {e}')
            self.store_current_detection_and_continue()
            return

        if result is None:
            self.get_logger().error('El servicio /robot_to_pixel_plane no devolvió respuesta.')
            self.store_current_detection_and_continue()
            return

        if not result.success:
            self.get_logger().error(f'Error en proyección inversa: {result.message}')
            self.store_current_detection_and_continue()
            return

        u = float(result.u)
        v = float(result.v)

        self.current_piece_data['reproj_u'] = u
        self.current_piece_data['reproj_v'] = v

        self.current_det_with_robot['reproj_u'] = u
        self.current_det_with_robot['reproj_v'] = v

        reproj_error = np.hypot(
            u - self.current_piece_data['cx'],
            v - self.current_piece_data['cy']
        )

        self.get_logger().info(
            f'RobotToPixel: robot=({self.current_piece_data["robot_x"]:.6f}, '
            f'{self.current_piece_data["robot_y"]:.6f}) -> reproy=({u:.2f}, {v:.2f}) '
            f'error={reproj_error:.2f}px'
        )

        self.current_piece_data['reproj_error_px'] = float(reproj_error)
        self.current_det_with_robot['reproj_error_px'] = float(reproj_error)

        self.store_current_detection_and_continue()

    def draw_detections(self, frame_bgr, detections):
        vis = frame_bgr.copy()
        self.save_detections_yaml(detections=detections)
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
            if det.get('top_contour') is not None:
                top_rect = cv2.minAreaRect(det['top_contour'])
                top_box = np.int32(cv2.boxPoints(top_rect))
                cv2.drawContours(vis, [top_box], 0, (0, 165, 255), 2)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 0, 0), 2)
            for stud in det.get('studs', []):
                scx = int(stud['cx'])
                scy = int(stud['cy'])
                sr = int(stud.get('r', 0))
                cv2.circle(vis, (scx, scy), max(2, sr), (255, 255, 0), 2)
                cv2.circle(vis, (scx, scy), 2, (255, 255, 0), -1)
            cv2.circle(vis, (cx, cy), 4, (0, 0, 255), -1)
            if 'reproj_u' in det and det['reproj_u'] is not None:
                ru = int(round(det['reproj_u']))
                rv = int(round(det['reproj_v']))
                cv2.circle(vis, (ru, rv), 5, (255, 0, 255), 2)
                cv2.line(vis, (cx, cy), (ru, rv), (255, 255, 255), 1)
            method = det.get('center_method', 'center')
            text = f'id={det["id"]} {color} {size_class} ({cx},{cy}) {method} studs={det.get("studs_count", 0)}/{det.get("studs_raw_count", det.get("studs_count", 0))} a={int(area)} ang={angle_deg:.1f}'
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

        if header is not None:
            msg_out.header.stamp = header.stamp
            msg_out.header.frame_id = header.frame_id

        self.foreground_publisher.publish(msg_out)

    def process_snapshot(self, frame_bgr, header):
        self.get_logger().info(
            f'Procesando snapshot con resolucion={frame_bgr.shape[1]}x{frame_bgr.shape[0]}'
        )

        fg_mask = self.subtract_background(frame_bgr)
        fg_mask = self.clean_mask(fg_mask)
        fg_mask = self.remove_border_noise(fg_mask)
        self.publish_foreground(fg_mask, header)

        instance_labels, separated_mask = self.split_touching_pieces(fg_mask, frame_bgr)
        detections = self.extract_blobs(instance_labels, frame_bgr)
        self.get_logger().info(
            f'Se han detectado {len(detections)} blobs candidatos.'
        )

        self.pending_frame_bgr = frame_bgr
        self.pending_header = header
        self.pending_fg_mask = fg_mask
        self.pending_separated_mask = separated_mask
        self.pending_detections = detections
        self.pending_index = 0
        self.pending_pieces_out = []
        self.pending_detections_with_robot = []

        if len(detections) == 0:
            self.finish_snapshot_processing()
            return

        self.process_next_detection()

    def process_next_detection(self):
        if self.pending_index >= len(self.pending_detections):
            self.finish_snapshot_processing()
            return

        det = self.pending_detections[self.pending_index]

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
            'center_method': det.get('center_method', self.center_method),
            'studs_count': int(det.get('studs_count', 0)),
            'studs': det.get('studs', []),
            'robot_x': None,
            'robot_y': None,
            'robot_z': None,
            'reproj_u': None,
            'reproj_v': None,
            'reproj_error_px': None,
        }

        det_with_robot = det.copy()
        det_with_robot['robot_x'] = None
        det_with_robot['robot_y'] = None
        det_with_robot['robot_z'] = None
        det_with_robot['reproj_u'] = None
        det_with_robot['reproj_v'] = None
        det_with_robot['reproj_error_px'] = None

        self.current_piece_data = piece_data
        self.current_det_with_robot = det_with_robot

        req = PixelToRobot.Request()
        req.u = float(det['cx'])
        req.v = float(det['cy'])

        self.get_logger().info(
            f'Pieza {det["id"]}: bbox=({det["x"]},{det["y"]},{det["w"]},{det["h"]}) '
            f'centro_detectado=({det["cx"]},{det["cy"]}) '
            f'area={det["area"]:.1f} angle={det["angle_deg"]:.1f} '
            f'color={det["color"]} size={det["size_class"]}'
        )

        future = self.pixel_to_robot_client.call_async(req)
        future.add_done_callback(self.on_pixel_to_robot_done)

    def store_current_detection_and_continue(self):
        self.pending_pieces_out.append(self.current_piece_data)
        self.pending_detections_with_robot.append(self.current_det_with_robot)

        self.pending_index += 1
        self.process_next_detection()

    def finish_snapshot_processing(self):
        annotated = self.draw_detections(
            self.pending_frame_bgr,
            self.pending_detections_with_robot
        )

        payload = {
            'phase': 'loose_pieces_ready',
            'image_width': int(self.pending_frame_bgr.shape[1]),
            'image_height': int(self.pending_frame_bgr.shape[0]),
            'pieces': self.pending_pieces_out
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.publisher.publish(out_msg)

        for piece in self.pending_pieces_out:
            self.get_logger().info(
                f'RESULTADO FINAL pieza {piece["id"]}: '
                f'pixel=({piece["cx"]},{piece["cy"]}) '
                f'robot=({piece["robot_x"]}, {piece["robot_y"]}) '
                f'reproy=({piece["reproj_u"]}, {piece["reproj_v"]}) '
                f'err={piece["reproj_error_px"]}'
            )

        self.get_logger().info(
            f'Foto procesada. Publicadas {len(self.pending_pieces_out)} piezas sueltas.'
        )

        if self.save_snapshot:
            cv2.imwrite(self.snapshot_path, self.pending_frame_bgr)
            self.get_logger().info(f'Foto guardada en: {self.snapshot_path}')

        if self.save_annotated:
            cv2.imwrite(self.annotated_path, annotated)
            self.get_logger().info(f'Imagen anotada guardada en: {self.annotated_path}')

        if self.show_debug:
            cv2.imshow('Loose pieces snapshot', self.pending_frame_bgr)
            cv2.imshow('Loose pieces foreground', self.pending_fg_mask)
            cv2.imshow('Loose pieces separated', self.pending_separated_mask)
            cv2.imshow('Loose pieces detections', annotated)
            cv2.waitKey(1)

        self.snapshot_taken = True
        self.processed_once = True
        self.processing_snapshot = False

        self.current_piece_data = None
        self.current_det_with_robot = None

    def timer_callback(self):
        if self.processing_snapshot:
            return

        if self.snapshot_requested and not self.processed_once:
            if self.latest_frame is None:
                self.get_logger().warn('No hay frame disponible todavía para procesar.')
                return

            self.processing_snapshot = True
            self.snapshot_requested = False

            try:
                self.get_logger().info('Procesando snapshot fuera del callback de imagen...')
                self.process_snapshot(self.latest_frame.copy(), self.latest_header)
            except Exception as e:
                self.processing_snapshot = False
                self.get_logger().error(f'Error procesando snapshot: {e}')

    def image_callback(self, msg):
        try:
            frame_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = self.undistort_image(frame_raw) if self.use_undistort else frame_raw
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        self.latest_frame = frame.copy()
        self.latest_header = msg.header

        #self.get_logger().info(
        #    f'Frame recibido: resolucion={frame.shape[1]}x{frame.shape[0]}'
        #)

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

            #cv2.namedWindow('Loose pieces live preview', cv2.WINDOW_NORMAL)
            cv2.imshow('Loose pieces live preview', preview)
            #cv2.resizeWindow('Loose pieces live preview', 1920, 1080)
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
                self.snapshot_requested = False
                self.processing_snapshot = False
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
                elif self.processing_snapshot:
                    self.get_logger().warn('Ya hay un snapshot en proceso.')
                else:
                    self.snapshot_requested = True
                    self.get_logger().info('Foto solicitada con tecla p.')
    def save_detections_yaml(self, detections, path="/tmp/detections.yaml"):
        yaml_data = []
        for det in detections:
            det_yaml = {
                'id': int(det['id']),
                'bbox': {
                    'x': int(det['x']),
                    'y': int(det['y']),
                    'w': int(det['w']),
                    'h': int(det['h'])
                },
                'center': {
                    'cx': int(det['cx']),
                    'cy': int(det['cy'])
                },
                'area': float(det['area']),
                'angle_deg': float(det['angle_deg']),
                'size_class': det['size_class'],
                'color': det['color'],
                'center_method': det.get('center_method', 'unknown'),
                'studs_count': int(det.get('studs_count', 0)),
                'studs': [
                    {
                        'cx': int(s['cx']),
                        'cy': int(s['cy']),
                        'r': int(s.get('r', 0))
                    }
                    for s in det.get('studs', [])
                ],
            }
            for key in ('rect_cx', 'rect_cy', 'moment_cx', 'moment_cy', 'bbox_cx', 'bbox_cy'):
                if key in det and det[key] is not None:
                    det_yaml[key] = float(det[key])
            if 'robot_x' in det and det['robot_x'] is not None:
                det_yaml['robot'] = {
                    'x': float(det['robot_x']),
                    'y': float(det['robot_y'])
                }
            if 'reproj_error_px' in det and det['reproj_error_px'] is not None:
                det_yaml['reprojection_error_px'] = float(det['reproj_error_px'])
            if 'contour' in det:
                det_yaml['contour'] = det['contour'].reshape(-1, 2).tolist()
            yaml_data.append(det_yaml)
        with open(path, 'w') as f:
            yaml.dump({'detections': yaml_data}, f, sort_keys=False)
        self.get_logger().info(f'Detections saved to YAML: {path}')

    
    def load_camera_calibration(self):
        self.K = None
        self.D = None

        # En casa puede no existir el path absoluto del laboratorio. Como la imagen
        # no se está desdistorsionando salvo que use_undistort=True, no dejamos que
        # el nodo se caiga por no encontrar este YAML.
        default_path = '/home/ricardo/Desktop/LabRobot/Paula/src/config/camera_intrinsics.yaml'
        calib_path = getattr(self, 'camera_calibration_path', default_path)

        try:
            with open(calib_path, 'r') as f:
                data = yaml.safe_load(f)

            self.K = np.array(data['camera_matrix']['data']).reshape(3, 3)
            self.D = np.array(data['distortion_coefficients']['data'])
            self.get_logger().info(f'Camera calibration loaded: {calib_path}')
        except Exception as e:
            self.get_logger().warn(
                f'No se pudo cargar camera_intrinsics.yaml desde {calib_path}. '
                f'El nodo seguirá sin undistort. Motivo: {e}'
            )

    def undistort_image(self, frame):
        if self.K is None or self.D is None:
            return frame

        h, w = frame.shape[:2]

        new_K, roi = cv2.getOptimalNewCameraMatrix(
            self.K, self.D, (w, h), 1, (w, h)
        )

        undistorted = cv2.undistort(frame, self.K, self.D, None, new_K)

        return undistorted
    
    
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