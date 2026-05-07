#!/usr/bin/env python3
from pathlib import Path
import cv2
import numpy as np
import pandas as pd
import json
import argparse

def smooth1d(arr: np.ndarray, k: int = 31) -> np.ndarray:
    k = max(3, int(k) | 1)
    kernel = np.ones(k, dtype=np.float32) / k
    return np.convolve(arr.astype(np.float32), kernel, mode="same")

def local_peaks(arr: np.ndarray, min_distance: int, threshold: float):
    peaks = []
    for i in range(1, len(arr) - 1):
        if arr[i] >= threshold and arr[i] >= arr[i - 1] and arr[i] >= arr[i + 1]:
            if peaks and i - peaks[-1] < min_distance:
                if arr[i] > arr[peaks[-1]]:
                    peaks[-1] = i
            else:
                peaks.append(i)
    return peaks

def classify_color_bgr(bgr):
    hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
    h, s, v = [int(x) for x in hsv]
    if s < 30 and v > 180:
        return "white"
    if h <= 10 or h >= 170:
        return "red"
    if 18 <= h <= 40:
        return "yellow"
    if 45 <= h <= 95:
        return "green"
    if 96 <= h <= 135:
        return "blue"
    return "unknown"

def robust_piece_color(img, box):
    x1, y1, x2, y2 = box
    pad_x = int((x2 - x1) * 0.22)
    pad_y = int((y2 - y1) * 0.22)
    patch = img[y1 + pad_y:y2 - pad_y, x1 + pad_x:x2 - pad_x]
    med = np.median(patch.reshape(-1, 3), axis=0)
    return classify_color_bgr(med)

def detect_rows_and_seams(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)

    y_profile = smooth1d(np.mean(np.abs(gy), axis=1), 41)
    y_candidates = local_peaks(y_profile, min_distance=80, threshold=float(np.percentile(y_profile, 97)))
    y_candidates = [p for p in y_candidates if 350 <= p <= 980]
    y_strong = sorted(y_candidates, key=lambda p: y_profile[p], reverse=True)[:3]
    y_strong = sorted(y_strong)
    if len(y_strong) != 3:
        top_y, seam_y, bottom_y = 438, 724, 901
    else:
        top_y, seam_y, bottom_y = y_strong

    top_band = gx[max(0, top_y - 5):min(img.shape[0], seam_y + 5), :]
    x_profile_top = smooth1d(np.mean(np.abs(top_band), axis=0), 31)
    top_peaks = local_peaks(x_profile_top, min_distance=90, threshold=10)
    top_peaks = [p for p in top_peaks if x_profile_top[p] > 0.18 * float(np.max(x_profile_top))]
    top_main = []
    for p in sorted(top_peaks):
        if 200 <= p <= 1850:
            if not top_main or abs(p - top_main[-1]) > 120:
                top_main.append(p)
    if top_main and top_main[-1] < 1650:
        top_main.append(1791)
    if len(top_main) < 5:
        top_main = [249, 642, 1016, 1398, 1791]
    else:
        anchors = [249, 642, 1016, 1398, 1791]
        top_main = [int(min(top_main, key=lambda p: abs(p - a))) for a in anchors]

    bottom_band = gx[max(0, seam_y - 5):min(img.shape[0], bottom_y + 5), :]
    x_profile_bottom = smooth1d(np.mean(np.abs(bottom_band), axis=0), 31)
    bottom_peaks = local_peaks(x_profile_bottom, min_distance=90, threshold=8)
    bottom_peaks = [p for p in bottom_peaks if x_profile_bottom[p] > 0.12 * float(np.max(x_profile_bottom))]
    bottom_main = []
    for p in sorted(bottom_peaks):
        if 200 <= p <= 1850:
            if not bottom_main or abs(p - bottom_main[-1]) > 120:
                bottom_main.append(p)
    if len(bottom_main) < 5:
        bottom_main = [249, 642, 1208, 1591, 1791]
    else:
        anchors = [249, 642, 1208, 1591, 1791]
        bottom_main = [int(min(bottom_main, key=lambda p: abs(p - a))) for a in anchors]

    return (int(top_y), int(seam_y), int(bottom_y)), top_main, bottom_main

def build_piece_table(img):
    (top_y, seam_y, bottom_y), top_bounds, bottom_bounds = detect_rows_and_seams(img)
    top_unit = np.median(np.diff(top_bounds))
    pieces = []

    for i in range(len(top_bounds) - 1):
        x1, x2 = top_bounds[i], top_bounds[i + 1]
        y1, y2 = top_y, seam_y
        color = robust_piece_color(img, (x1, y1, x2, y2))
        studs = max(1, round(((x2 - x1) / top_unit) * 2))
        pieces.append({
            "piece_id": f"T{i+1}",
            "row": "top",
            "x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2),
            "color": color,
            "size_estimate": f"2x{studs}"
        })

    for i in range(len(bottom_bounds) - 1):
        x1, x2 = bottom_bounds[i], bottom_bounds[i + 1]
        y1, y2 = seam_y, bottom_y
        color = robust_piece_color(img, (x1, y1, x2, y2))
        studs = max(1, round(((x2 - x1) / top_unit) * 2))
        pieces.append({
            "piece_id": f"B{i+1}",
            "row": "bottom",
            "x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2),
            "color": color,
            "size_estimate": f"2x{studs}"
        })

    return pieces, {"top_y": top_y, "seam_y": seam_y, "bottom_y": bottom_y}

def draw(img, pieces, guide):
    out = img.copy()
    cv2.line(out, (0, guide["top_y"]), (out.shape[1]-1, guide["top_y"]), (180, 180, 180), 2)
    cv2.line(out, (0, guide["seam_y"]), (out.shape[1]-1, guide["seam_y"]), (140, 140, 140), 2)
    cv2.line(out, (0, guide["bottom_y"]), (out.shape[1]-1, guide["bottom_y"]), (180, 180, 180), 2)
    palette = {
        "red": (0, 0, 255),
        "blue": (255, 0, 0),
        "yellow": (0, 255, 255),
        "green": (0, 180, 0),
        "white": (170, 170, 170),
        "unknown": (255, 0, 255),
    }
    for p in pieces:
        c = palette[p["color"]]
        cv2.rectangle(out, (p["x1"], p["y1"]), (p["x2"], p["y2"]), c, 4)
        label = f'{p["piece_id"]} {p["color"]} {p["size_estimate"]}'
        ytxt = max(35, p["y1"] - 8)
        cv2.rectangle(out, (p["x1"], ytxt - 26), (min(out.shape[1]-1, p["x1"] + 250), ytxt + 2), (255, 255, 255), -1)
        cv2.putText(out, label, (p["x1"] + 6, ytxt - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.72, c, 2, cv2.LINE_AA)
    return out

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--outdir", default="results")
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise FileNotFoundError(args.image)

    pieces, guide = build_piece_table(img)
    out = draw(img, pieces, guide)

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(outdir / "lego_detected_fixed.png"), out)
    pd.DataFrame(pieces).to_csv(outdir / "lego_detected_fixed.csv", index=False)
    (outdir / "lego_detected_fixed.json").write_text(
        json.dumps({"pieces": pieces, "guide": guide}, indent=2),
        encoding="utf-8"
    )

    print(pd.DataFrame(pieces))
    print(f"Saved: {outdir / 'lego_detected_fixed.png'}")
    print(f"Saved: {outdir / 'lego_detected_fixed.csv'}")
    print(f"Saved: {outdir / 'lego_detected_fixed.json'}")

if __name__ == "__main__":
    main()
