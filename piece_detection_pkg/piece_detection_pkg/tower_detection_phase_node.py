#!/usr/bin/env python3
import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


def border_median_bgr(img, border=24):
    h, w = img.shape[:2]
    border = max(8, min(border, h // 3, w // 3))
    strips = [
        img[:border, :, :],
        img[h - border:, :, :],
        img[:, :border, :],
        img[:, w - border:, :],
    ]
    pix = np.concatenate([s.reshape(-1, 3) for s in strips], axis=0)
    return np.median(pix, axis=0).astype(np.uint8)


def compute_bg_distance(img):
    h, w = img.shape[:2]
    bg = border_median_bgr(img, border=max(12, min(h, w) // 35))
    bg_img = np.full_like(img, bg)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
    lab_bg = cv2.cvtColor(bg_img, cv2.COLOR_BGR2LAB).astype(np.float32)
    dist = np.linalg.norm(lab - lab_bg, axis=2)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)
    return dist, sat, val



def rgb_to_rgi(bgr_img):
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


def subtract_background_rgi(frame_bgr, background_rgi, bg_threshold):
    frame_rgi = rgb_to_rgi(frame_bgr)
    if background_rgi is None or frame_rgi.shape != background_rgi.shape:
        return np.zeros(frame_bgr.shape[:2], dtype=np.uint8)

    diff = np.abs(frame_rgi - background_rgi)
    dr = diff[:, :, 0]
    dg = diff[:, :, 1]
    dI = diff[:, :, 2] / 255.0
    fg = ((dr > bg_threshold) | (dg > bg_threshold) | (dI > bg_threshold)).astype(np.uint8) * 255
    return fg


def clean_background_mask(mask, median_kernel=5, opening_kernel=3, closing_kernel=7):
    if median_kernel >= 3 and median_kernel % 2 == 1:
        mask = cv2.medianBlur(mask, median_kernel)
    k_open = cv2.getStructuringElement(cv2.MORPH_RECT, (opening_kernel, opening_kernel))
    k_close = cv2.getStructuringElement(cv2.MORPH_RECT, (closing_kernel, closing_kernel))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=2)
    return mask


def keep_large_components(mask, min_area, reject_border_touch=False):
    H, W = mask.shape[:2]
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    out = np.zeros_like(mask)
    boxes = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area:
            continue
        if reject_border_touch and (x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1):
            continue
        out[labels == i] = 255
        boxes.append((x, y, w, h, area))
    return out, boxes


def build_seed_mask(img):
    dist, sat, val = compute_bg_distance(img)
    seed = (((sat > 55) & (val > 55)) | ((dist > 24) & (sat > 30) & (val > 50))).astype(np.uint8) * 255
    k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    seed = cv2.medianBlur(seed, 5)
    seed = cv2.morphologyEx(seed, cv2.MORPH_OPEN, k3, iterations=1)
    seed = cv2.morphologyEx(seed, cv2.MORPH_CLOSE, k5, iterations=2)
    min_area = max(120, int(0.0012 * img.shape[0] * img.shape[1]))
    return keep_large_components(seed, min_area, reject_border_touch=True)


def envelope_from_boxes(boxes, shape):
    h, w = shape[:2]
    if not boxes:
        mx = int(0.18 * w)
        my = int(0.10 * h)
        return (mx, my, w - 2 * mx, h - 2 * my)

    x0 = min(b[0] for b in boxes)
    y0 = min(b[1] for b in boxes)
    x1 = max(b[0] + b[2] for b in boxes)
    y1 = max(b[1] + b[3] for b in boxes)

    pad_x = max(18, int(0.08 * (x1 - x0 + 1)))
    pad_y = max(18, int(0.12 * (y1 - y0 + 1)))
    x0 = max(0, x0 - pad_x)
    y0 = max(0, y0 - pad_y)
    x1 = min(w, x1 + pad_x)
    y1 = min(h, y1 + pad_y)

    if (x1 - x0) > 0.82 * w:
        cx = (x0 + x1) // 2
        half = int(0.41 * w)
        x0 = max(0, cx - half)
        x1 = min(w, cx + half)
    if (y1 - y0) > 0.86 * h:
        cy = (y0 + y1) // 2
        half = int(0.43 * h)
        y0 = max(0, cy - half)
        y1 = min(h, cy + half)

    return (x0, y0, x1 - x0, y1 - y0)


def foreground_mask(img):
    h, w = img.shape[:2]
    dist, sat, val = compute_bg_distance(img)
    seed_mask, seed_boxes = build_seed_mask(img)
    ex, ey, ew, eh = envelope_from_boxes(seed_boxes, img.shape)

    envelope = np.zeros((h, w), dtype=np.uint8)
    envelope[ey:ey + eh, ex:ex + ew] = 255

    fg = ((((dist > 16) & (val > 38)) | ((sat > 26) & (val > 45))) & (envelope > 0)).astype(np.uint8) * 255
    fg = cv2.bitwise_or(fg, seed_mask)

    k5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    k7 = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    fg = cv2.medianBlur(fg, 5)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, k5, iterations=1)
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, k7, iterations=2)

    min_area = int(0.002 * h * w)
    fg, _ = keep_large_components(fg, min_area)
    return fg, (ex, ey, ew, eh), seed_mask


def color_cluster_labels(img, fg_mask, max_clusters=5):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
    ys, xs = np.where(fg_mask > 0)
    if len(xs) == 0:
        return np.full(fg_mask.shape, -1, dtype=np.int32), 0

    samples = np.column_stack([
        lab[ys, xs, 1] * 1.0,
        lab[ys, xs, 2] * 1.0,
        lab[ys, xs, 0] * 0.30,
    ]).astype(np.float32)

    unique_estimate = max(1, min(max_clusters, len(np.unique(np.round(samples[:, :2] / 6), axis=0))))
    K = min(max_clusters, max(2, unique_estimate))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.8)
    _, labels, _ = cv2.kmeans(samples, K, None, criteria, 6, cv2.KMEANS_PP_CENTERS)

    label_img = np.full(fg_mask.shape, -1, dtype=np.int32)
    label_img[ys, xs] = labels.flatten().astype(np.int32)
    return label_img, K


def color_name_from_patch(patch_bgr, mask=None):
    hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)
    if mask is None:
        mask = np.ones(hsv.shape[:2], dtype=bool)
    else:
        mask = mask.astype(bool)

    if not np.any(mask):
        return "unknown"

    h = float(np.median(hsv[:, :, 0][mask]))
    s = float(np.median(hsv[:, :, 1][mask]))
    if s < 22:
        return "neutral"
    if h < 8 or h >= 170:
        return "red"
    if 8 <= h < 22:
        return "orange"
    if 22 <= h < 42:
        return "yellow"
    if 42 <= h < 90:
        return "green"
    if 90 <= h < 135:
        return "blue"
    if 135 <= h < 170:
        return "pink"
    return "unknown"


def split_component_if_needed(img, mask, bbox_global):
    x, y, w, h = bbox_global
    if h < 70:
        return [(mask, bbox_global)]

    patch = img[y:y + h, x:x + w]
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    sobely = np.abs(cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3))

    row_fg = np.count_nonzero(mask, axis=1).astype(np.float32) / max(w, 1)
    row_edge = sobely.mean(axis=1)
    if row_edge.max() > 0:
        row_edge /= row_edge.max()
    score = 0.60 * row_fg + 0.40 * row_edge

    valleys = []
    for i in range(2, len(score) - 2):
        if score[i] <= score[i - 1] and score[i] <= score[i + 1] and row_fg[i] < 0.82 * max(0.05, row_fg.max()):
            valleys.append(i)

    if not valleys:
        return [(mask, bbox_global)]

    best = min(valleys, key=lambda i: score[i])
    if not (18 < best < h - 18):
        return [(mask, bbox_global)]
    if row_fg[best] > 0.58 * row_fg.max():
        return [(mask, bbox_global)]

    top = mask[:best, :]
    bot = mask[best:, :]
    out = []
    for sub, dy in [(top, 0), (bot, best)]:
        ys, xs = np.where(sub > 0)
        if len(xs) == 0:
            continue
        sx0, sy0, sx1, sy1 = xs.min(), ys.min(), xs.max(), ys.max()
        if (sx1 - sx0 + 1) < 18 or (sy1 - sy0 + 1) < 14:
            continue
        sub_mask = sub[sy0:sy1 + 1, sx0:sx1 + 1].copy()
        out.append((sub_mask, (x + sx0, y + dy + sy0, sx1 - sx0 + 1, sy1 - sy0 + 1)))

    return out if len(out) >= 2 else [(mask, bbox_global)]


def candidate_quality(img, piece, envelope):
    x, y, w, h = piece["x"], piece["y"], piece["w"], piece["h"]
    patch = img[y:y + h, x:x + w]
    mask = piece["mask_local"] > 0

    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    sat = hsv[:, :, 1][mask]
    val = hsv[:, :, 2][mask]
    med_sat = float(np.median(sat)) if sat.size else 0.0
    med_val = float(np.median(val)) if val.size else 0.0
    fill_ratio = float(np.count_nonzero(mask)) / max(1.0, w * h)

    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 120)
    edge_ratio = float(np.count_nonzero(edges)) / max(1.0, w * h)

    ex, ey, ew, eh = envelope
    inside = (x >= ex - 8 and y >= ey - 8 and x + w <= ex + ew + 8 and y + h <= ey + eh + 8)

    if not inside:
        return False
    if fill_ratio < 0.38:
        return False
    if med_sat < 28 and med_val < 90:
        return False
    if med_sat < 22:
        return False
    if edge_ratio < 0.006:
        return False
    return True


def detect_tower_pieces(img, forced_fg_mask=None):
    h, w = img.shape[:2]

    if forced_fg_mask is None:
        fg, envelope, seed_mask = foreground_mask(img)
    else:
        fg = forced_fg_mask.copy()
        min_area_env = max(120, int(0.0012 * h * w))
        seed_mask, seed_boxes = keep_large_components(fg, min_area_env, reject_border_touch=True)
        envelope = envelope_from_boxes(seed_boxes, img.shape)
        min_area_fg = int(0.002 * h * w)
        fg, _ = keep_large_components(fg, min_area_fg, reject_border_touch=False)

    label_img, K = color_cluster_labels(img, fg, max_clusters=6)

    pieces = []
    h, w = fg.shape[:2]
    min_area = int(0.003 * h * w)

    for k in range(K):
        cluster_mask = np.where(label_img == k, 255, 0).astype(np.uint8)
        if np.count_nonzero(cluster_mask) == 0:
            continue

        cluster_mask = cv2.morphologyEx(
            cluster_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
            iterations=1,
        )
        n, labels, stats, _ = cv2.connectedComponentsWithStats(cluster_mask, 8)
        for i in range(1, n):
            x, y, ww, hh, area = stats[i]
            if area < min_area:
                continue

            comp = np.where(labels[y:y + hh, x:x + ww] == i, 255, 0).astype(np.uint8)
            ys, xs = np.where(comp > 0)
            if len(xs) == 0:
                continue

            sx0, sy0, sx1, sy1 = xs.min(), ys.min(), xs.max(), ys.max()
            comp = comp[sy0:sy1 + 1, sx0:sx1 + 1]
            gx, gy, gw, gh = x + sx0, y + sy0, sx1 - sx0 + 1, sy1 - sy0 + 1

            for sub_mask, (tx, ty, tw, th) in split_component_if_needed(img, comp, (gx, gy, gw, gh)):
                area2 = int(np.count_nonzero(sub_mask))
                if area2 < max(200, min_area // 2):
                    continue
                patch = img[ty:ty + th, tx:tx + tw]
                color = color_name_from_patch(patch, sub_mask > 0)
                piece = {
                    "x": int(tx),
                    "y": int(ty),
                    "w": int(tw),
                    "h": int(th),
                    "cx": int(tx + tw / 2),
                    "cy": int(ty + th / 2),
                    "area": area2,
                    "color": color,
                    "mask_local": sub_mask,
                }
                if candidate_quality(img, piece, envelope):
                    pieces.append(piece)

    pieces = sorted(pieces, key=lambda p: p["area"], reverse=True)
    kept = []
    for p in pieces:
        ok = True
        for q in kept:
            ix0 = max(p["x"], q["x"])
            iy0 = max(p["y"], q["y"])
            ix1 = min(p["x"] + p["w"], q["x"] + q["w"])
            iy1 = min(p["y"] + p["h"], q["y"] + q["h"])
            inter = max(0, ix1 - ix0) * max(0, iy1 - iy0)
            union = p["w"] * p["h"] + q["w"] * q["h"] - inter
            if union > 0 and inter / union > 0.50:
                ok = False
                break
        if ok:
            kept.append(p)

    pieces = sorted(kept, key=lambda p: (p["y"], p["x"]))
    if not pieces:
        return {"pieces": [], "foreground_mask": fg, "seed_mask": seed_mask, "envelope": envelope}

    centers_y = np.array([p["cy"] for p in pieces], dtype=np.float32)
    order = np.argsort(centers_y)
    levels = []
    for idx in order.tolist():
        p = pieces[idx]
        placed = False
        for lev in levels:
            if abs(p["cy"] - lev["mean_cy"]) < 0.55 * max(p["h"], lev["mean_h"]):
                lev["idxs"].append(idx)
                lev["mean_cy"] = np.mean([pieces[j]["cy"] for j in lev["idxs"]])
                lev["mean_h"] = np.mean([pieces[j]["h"] for j in lev["idxs"]])
                placed = True
                break
        if not placed:
            levels.append({"idxs": [idx], "mean_cy": p["cy"], "mean_h": p["h"]})

    levels = sorted(levels, key=lambda lev: lev["mean_cy"])
    for li, lev in enumerate(levels):
        idxs = sorted(lev["idxs"], key=lambda j: pieces[j]["x"])
        widths = [pieces[j]["w"] for j in idxs]
        median_w = float(np.median(widths)) if widths else 1.0
        for ci, j in enumerate(idxs):
            pieces[j]["level"] = li
            pieces[j]["col"] = ci
            pieces[j]["size_class"] = "rectangular" if pieces[j]["w"] > 1.35 * median_w and len(idxs) > 1 else "square"

    widths = np.array([p["w"] for p in pieces], dtype=np.float32)
    if len(widths) >= 2 and widths.max() / max(1.0, widths.min()) > 1.45:
        c1, c2 = widths.min(), widths.max()
        for _ in range(8):
            g1 = widths[np.abs(widths - c1) <= np.abs(widths - c2)]
            g2 = widths[np.abs(widths - c2) < np.abs(widths - c1)]
            if len(g1):
                c1 = g1.mean()
            if len(g2):
                c2 = g2.mean()
        split = (c1 + c2) / 2.0
        for p in pieces:
            p["size_class"] = "rectangular" if p["w"] >= split else "square"

    for i, p in enumerate(sorted(pieces, key=lambda d: (d["level"], d["x"]))):
        p["id"] = i

    return {"pieces": pieces, "foreground_mask": fg, "seed_mask": seed_mask, "envelope": envelope}


def draw_tower_results(img, result):
    vis = img.copy()
    ex, ey, ew, eh = result["envelope"]
    cv2.rectangle(vis, (ex, ey), (ex + ew, ey + eh), (255, 200, 0), 2)
    for p in result["pieces"]:
        x, y, w, h = p["x"], p["y"], p["w"], p["h"]
        cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(vis, (p["cx"], p["cy"]), 4, (0, 0, 255), -1)
        txt = f"L{p['level']} C{p['col']} {p['size_class']} {p['color']}"
        cv2.putText(vis, txt, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return vis


class TowerDetectionPhaseNode(Node):
    def __init__(self):
        super().__init__('tower_detection_phase_node')
        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', '/tower_plan')
        self.declare_parameter('command_topic', '/vision_phase_command')
        self.declare_parameter('feedback_topic', '/robot_phase_feedback')

        self.declare_parameter('tower_pose_name', 'DetectaTorre')
        self.declare_parameter('next_pose_name', 'DetectaPiezasSueltas')

        self.declare_parameter('copy_offset_x_px', 0)
        self.declare_parameter('copy_offset_y_px', 0)

        self.declare_parameter('bg_threshold', 0.08)
        self.declare_parameter('median_kernel', 5)
        self.declare_parameter('opening_kernel', 3)
        self.declare_parameter('closing_kernel', 7)
        self.declare_parameter('show_debug', True)
        self.declare_parameter('save_debug_image', False)
        self.declare_parameter('debug_image_path', '/tmp/tower_detect_debug.png')

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value
        command_topic = self.get_parameter('command_topic').value
        feedback_topic = self.get_parameter('feedback_topic').value

        self.tower_pose_name = str(self.get_parameter('tower_pose_name').value)
        self.next_pose_name = str(self.get_parameter('next_pose_name').value)

        self.copy_offset_x_px = int(self.get_parameter('copy_offset_x_px').value)
        self.copy_offset_y_px = int(self.get_parameter('copy_offset_y_px').value)

        self.bg_threshold = float(self.get_parameter('bg_threshold').value)
        self.median_kernel = int(self.get_parameter('median_kernel').value)
        self.opening_kernel = int(self.get_parameter('opening_kernel').value)
        self.closing_kernel = int(self.get_parameter('closing_kernel').value)
        self.show_debug = bool(self.get_parameter('show_debug').value)
        self.save_debug_image = bool(self.get_parameter('save_debug_image').value)
        self.debug_image_path = str(self.get_parameter('debug_image_path').value)

        self.phase = 'request_detect_tower_pose'
        self.robot_at_tower_pose = False
        self.background_rgi = None
        self.background_ready = False
        self.snapshot_requested = False
        self.snapshot_taken = False
        self.processed_once = False

        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(String, output_topic, 10)
        self.command_publisher = self.create_publisher(String, command_topic, 10)
        self.robot_feedback_sub = self.create_subscription(String, feedback_topic, self.robot_feedback_callback, 10)

        self.initial_command_sent = False
        self.command_timer = self.create_timer(1.0, self.command_timer_callback)

        self.get_logger().info(f'Escuchando imágenes en: {image_topic}')
        self.get_logger().info(f'Publicando plan en: {output_topic}')
        self.get_logger().info(f'Comandos robot en: {command_topic}')
        self.get_logger().info(f'Feedback robot en: {feedback_topic}')
        self.get_logger().info(f'Pose inicial solicitada: {self.tower_pose_name}')
        self.get_logger().info('Flujo correcto para torre:')
        self.get_logger().info(f'1) El robot va a {self.tower_pose_name}')
        self.get_logger().info('2) Con el entorno/fondo vacio pulsa b')
        self.get_logger().info('3) Introduce la torre manualmente en la escena')
        self.get_logger().info('4) Pulsa p para congelar la imagen y procesarla una sola vez')
        self.get_logger().info('5) Pulsa r si quieres repetir manteniendo el mismo fondo')

    def publish_command(self, payload_dict):
        msg = String()
        msg.data = json.dumps(payload_dict)
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Comando publicado: {msg.data}')

    def command_timer_callback(self):
        if self.robot_at_tower_pose:
            return

        if self.phase == 'request_detect_tower_pose':
            self.publish_go_to_pose(self.tower_pose_name)
            self.get_logger().info(
                f'Reintentando comando para ir a {self.tower_pose_name} hasta recibir feedback...'
            )

    def publish_go_to_pose(self, pose_name):
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

        at_tower = (
            (pose == self.tower_pose_name and status in ('reached', 'ready', 'done')) or
            feedback == f'pose_reached:{self.tower_pose_name}' or
            feedback == f'{self.tower_pose_name}_ready' or
            raw == f'pose_reached:{self.tower_pose_name}' or
            raw == f'{self.tower_pose_name}_ready'
        )

        if at_tower:
            self.robot_at_tower_pose = True
            if hasattr(self, 'command_timer'):
                self.command_timer.cancel()
            self.phase = 'wait_background'
            self.get_logger().info(f'Robot confirma pose {self.tower_pose_name}. Deja el fondo vacio y pulsa b.')

    def build_target_list_and_plan(self, detected_pieces):
        ordered_for_plan = sorted(detected_pieces, key=lambda p: (-p['cy'], p['cx']))
        target_list = []
        for step, p in enumerate(ordered_for_plan):
            target_list.append({
                'step': int(step),
                'piece_id': int(p['id']),
                'level': int(p['level']),
                'col': int(p['col']),
                'size_class': p['size_class'],
                'color': p['color'],
                'place_cx': int(p['cx'] + self.copy_offset_x_px),
                'place_cy': int(p['cy'] + self.copy_offset_y_px),
                'source_cx': int(p['cx']),
                'source_cy': int(p['cy']),
                'bbox': {
                    'x': int(p['x']),
                    'y': int(p['y']),
                    'w': int(p['w']),
                    'h': int(p['h']),
                }
            })
        return target_list

    def publish_plan(self, frame, result):
        detected_pieces = result['pieces']
        target_list = self.build_target_list_and_plan(detected_pieces)

        payload = {
            'phase': 'tower_plan_ready',
            'image_width': int(frame.shape[1]),
            'image_height': int(frame.shape[0]),
            'tower_pose_name': self.tower_pose_name,
            'next_pose_name': self.next_pose_name,
            'copy_offset_x_px': int(self.copy_offset_x_px),
            'copy_offset_y_px': int(self.copy_offset_y_px),
            'detected_pieces': [
                {
                    'id': int(p['id']),
                    'level': int(p['level']),
                    'col': int(p['col']),
                    'x': int(p['x']),
                    'y': int(p['y']),
                    'w': int(p['w']),
                    'h': int(p['h']),
                    'cx': int(p['cx']),
                    'cy': int(p['cy']),
                    'area': int(p['area']),
                    'size_class': p['size_class'],
                    'color': p['color'],
                }
                for p in sorted(detected_pieces, key=lambda d: (d['level'], d['col']))
            ],
            'target_list': target_list,
        }

        out_msg = String()
        out_msg.data = json.dumps(payload)
        self.publisher.publish(out_msg)
        self.get_logger().info(f'Plan publicado con {len(target_list)} piezas objetivo.')

    def capture_background(self, frame_bgr):
        self.background_rgi = rgb_to_rgi(frame_bgr)
        self.background_ready = True
        self.snapshot_requested = False
        self.snapshot_taken = False
        self.processed_once = False
        self.phase = 'wait_tower_snapshot'
        self.get_logger().info(f'Fondo vacio capturado correctamente en la pose {self.tower_pose_name}.')
        self.get_logger().info('Ahora introduce la torre manualmente y pulsa p.')

    def build_foreground_from_background(self, frame_bgr):
        fg_mask = subtract_background_rgi(frame_bgr, self.background_rgi, self.bg_threshold)
        fg_mask = clean_background_mask(
            fg_mask,
            median_kernel=self.median_kernel,
            opening_kernel=self.opening_kernel,
            closing_kernel=self.closing_kernel,
        )
        return fg_mask

    def process_snapshot(self, frame, header=None):
        fg_mask = self.build_foreground_from_background(frame)
        result = detect_tower_pieces(frame, forced_fg_mask=fg_mask)
        debug_vis = draw_tower_results(frame, result)

        if self.show_debug:
            cv2.imshow('Tower input snapshot', frame)
            cv2.imshow('Tower background foreground', fg_mask)
            cv2.imshow('Tower seed', result['seed_mask'])
            cv2.imshow('Tower detections', debug_vis)
            cv2.waitKey(1)

        if len(result['pieces']) == 0:
            self.get_logger().warn('No se detectaron piezas de torre. Revisa el fondo o pulsa r y repite.')
            self.snapshot_taken = False
            self.processed_once = False
            return

        self.publish_plan(frame, result)

        if self.save_debug_image:
            cv2.imwrite(self.debug_image_path, debug_vis)
            self.get_logger().info(f'Imagen debug guardada en: {self.debug_image_path}')

        self.publish_go_to_pose(self.next_pose_name)
        self.phase = 'done'
        self.robot_at_tower_pose = False
        self.snapshot_taken = True
        self.processed_once = True
        self.get_logger().info(f'Fase torre terminada. Se solicito al robot la pose {self.next_pose_name}.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        if self.show_debug:
            preview = frame.copy()
            info_lines = []

            if not self.robot_at_tower_pose:
                info_lines.append(f'Esperando robot en {self.tower_pose_name}')
            elif not self.background_ready:
                info_lines.append('Robot en pose. Fondo vacio: pulsa b')
            elif not self.snapshot_taken and not self.processed_once:
                info_lines.append('Introduce la torre y pulsa p')
            else:
                info_lines.append('Torre procesada. Pulsa r para repetir')

            y0 = 25
            for line in info_lines:
                cv2.putText(preview, line, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                y0 += 30

            cv2.imshow('Tower live preview', preview)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('b'):
                if not self.robot_at_tower_pose:
                    self.get_logger().warn(f'No captures el fondo todavia. Primero el robot debe estar en {self.tower_pose_name}.')
                else:
                    self.capture_background(frame)

            elif key == ord('r'):
                self.snapshot_requested = False
                self.snapshot_taken = False
                self.processed_once = False
                if self.background_ready:
                    self.phase = 'wait_tower_snapshot'
                    self.get_logger().info('Reset hecho. Manteniendo el fondo actual. Introduce la torre y pulsa p.')
                else:
                    self.phase = 'wait_background'
                    self.get_logger().info('Reset hecho. Pulsa b para capturar el fondo vacio.')

            elif key == ord('p'):
                if not self.robot_at_tower_pose:
                    self.get_logger().warn(f'Todavia no hay confirmacion de que el robot este en {self.tower_pose_name}.')
                elif not self.background_ready:
                    self.get_logger().warn('Primero captura el fondo vacio con la tecla b.')
                else:
                    self.snapshot_requested = True
                    self.get_logger().info('Foto de torre solicitada con tecla p. Procesando frame congelado...')

        if self.snapshot_requested and not self.processed_once:
            self.snapshot_requested = False
            self.process_snapshot(frame.copy(), msg.header)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TowerDetectionPhaseNode()
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
