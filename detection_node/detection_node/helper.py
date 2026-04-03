import numpy as np
import cv2
import yaml

def BGR2RGI(frame: np.ndarray):
    """
    Convert BGR image to RGI representation.
    Args:
        frame(np.ndarray): Input image (H, W, 3) in BGR format
    Returns:
        out_put(np.ndarray): Output image (H, W, 3) in RGI format
        out_put_norm(np.ndarray): Output image (H, W, 3) in BGR format normalized
    """
    
    H, W, channels = np.shape(frame)
    if channels != 3:
        raise ValueError("Error converting from BGR2RGI, This doesn't have 3 channels")
    frame = frame.astype(np.float32)
    B = frame[:, :, 0]
    G = frame[:, :, 1]
    R = frame[:, :, 2]
    sum_rgb = R + G + B
    valid_ratio = np.mean(sum_rgb > 10)
    if valid_ratio < 0.05:
        raise ValueError("BGR2RGI error: too few valid pixels")
    sum_rgb += 1e-6
    r = 255 * R / sum_rgb
    g = 255 * G / sum_rgb
    I = sum_rgb / 3.0
    out_put = np.zeros((H,W,3), np.uint8)
    out_put[:, :, 0] = r
    out_put[:, :, 1] = g
    out_put[:, :, 2] = I
    out_put_norm= np.zeros((H,W,3), np.uint8)
    b = 1 - r - g
    out_put_norm[:,:,0] = b
    out_put_norm[:,:,1] = g 
    out_put_norm[:,:,2] = r 

    return out_put, out_put_norm

def background_removal(imageRGI: np.array, image_normalized: np.array, threshold: int, BGimage_address: str):
    """
    Performs a background subtraction
    Args:
        imageRGI (np.ndarray): The image to process (for this case we expect RGI format).
        BGimage_address (str): String with the address to an image of the empty scene.
        image_normalized (np.ndarray): The normalized image
    Returns:
        imageRGI (np.ndarray): Returns the same image with the background removed in RGI format
        image_normalized (np.ndarray): Returns the same normalized image with the background removed in RGI format
    """

    BG_image = cv2.imread(BGimage_address, cv2.IMREAD_COLOR)
    BgH, BgW, Bgchannel = np.shape(BG_image)
    H, W, channel = np.shape(imageRGI)
    NrH, NrW, Nrchannel = np.shape(image_normalized)
    if Bgchannel != 3 or channel != 3 or Nrchannel != 3:
        raise ValueError("Either background or imageRGI or the normalized image do not have 3 channels")
    if (H != BgH or W != BgW):
        raise ValueError("The width or height of background and test image are not the same")
    if (H != NrH or W != NrW):
        raise ValueError("Maybe the normalized image is different")
    BG_image_RGI, _ = BGR2RGI(BG_image)
    F_image = np.absolute(imageRGI.astype(np.uint32) - BG_image_RGI.astype(np.uint32))
    mask = (F_image < threshold).any(axis = 2)
    imageRGI[mask] = 0
    image_normalized[mask] = 0
    return imageRGI, image_normalized

def detect_color(imageRGI: np.array, image_normalized: np.array, yaml_path: str, color: str):
    H, W, channel = np.shape(imageRGI)
    if (channel != 3):
        raise ValueError("imageRGI does not have 3 channels")
    try:
        with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
    except Exception as e:
        print(f"Python says: {e}")
        raise NameError("Wrong path for the Yaml file")
    r = imageRGI[:, :, 0]
    g = imageRGI[:, :, 1]
    I = imageRGI[:, :, 2]

    for color_name, params in config.items():
        if color_name == color:
            mask = (
                (I <= 1) |
                (r <= params['R_min']) | (r >= params['R_max']) |
                (g <= params['G_min']) | (g >= params['G_max'])
            )
            mask_uint8 = mask.astype(np.uint8) * 255
            mask_uint8 = cv2.morphologyEx(mask_uint8, cv2.MORPH_OPEN, np.ones((10,10),np.uint8))
            mask_uint8 = cv2.morphologyEx(mask_uint8, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
            mask = mask_uint8 > 0
            imageRGI[mask] = 0
            image_normalized[mask] = 0
            mask = ~mask
            mask_uint8 = mask.astype(np.uint8) * 255
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_uint8 , 4 , cv2.CV_32S)
            # keep this outside (global or before loop)
            prev_dirs = {}
            center_angle_output = []
            for i in range(1, num_labels):  # skip background
                if stats[i, cv2.CC_STAT_AREA] < 50:
                    continue
                blob_mask = (labels == i).astype(np.uint8) * 255
                contours, _ = cv2.findContours(blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(contours) == 0:
                    continue
                cnt = max(contours, key=cv2.contourArea)
                data_pts = cnt.reshape(-1, 2).astype(np.float32)
                mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
                cx, cy = mean[0]
                vx, vy = eigenvectors[0]
                if i in prev_dirs:
                    prev_vx, prev_vy = prev_dirs[i]
                    if (vx * prev_vx + vy * prev_vy) < 0:
                        vx, vy = -vx, -vy
                prev_dirs[i] = (vx, vy)
                angle = np.degrees(np.arctan2(-vy, vx))
                if angle < 0:
                    angle += 360
                #print(f"Blob {i} | Center: ({cx:.1f}, {cy:.1f}) | Direction angle: {angle:.2f}°")
                center_angle_output.append((i ,cx, cy, angle))
            return [imageRGI, image_normalized, center_angle_output]
    return []


def image_angle_editor(image: np.ndarray, center_angle: tuple):
    (label, cx, cy, angle,) = center_angle
    length = 50  # how long you want the line
    angle_rad = np.radians(angle)
    
    x1 = int(cx)
    y1 = int(cy)
    
    x2 = int(cx + length * np.cos(angle_rad))
    y2 = int(cy - length * np.sin(angle_rad))  # minus because image y goes down
    
    cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return image


def foreground_from_rgi(rgi: np.ndarray, image_normalized: np.ndarray):
    """
    Compute foreground mask using RGI image.
    
    Args:
        rgi (np.ndarray): HxWx3 RGI image (uint32)
        rgi_normalized (np.ndarray): HxWx3 image_normalized (uint32)
    
    Returns:
        rgi (np.ndarray): The same image after applying foreground mask
        image_normalized (np.ndarray): The same image after applying foreground mask
    """
    rgi = rgi.astype(np.float32)

    R = rgi[:, :, 0]
    G = rgi[:, :, 1]
    I = rgi[:, :, 2]

     # --- chromatic invariant ---
    chroma = np.abs(R - G) / (R + G + 1e-6)
    # --- intensity variation ---
    I_blur = cv2.GaussianBlur(I, (5, 5), 0)
    intensity_diff = np.abs(I - I_blur)
    # --- combine ---
    score = 2.0 * chroma + 1.0 * intensity_diff
    # --- normalize ---
    score = cv2.normalize(score, None, 0, 255, cv2.NORM_MINMAX)
    score = score.astype(np.uint8)
    # --- threshold ---
    score_blur = cv2.GaussianBlur(score, (5, 5), 0)
    _, mask = cv2.threshold(
        score_blur,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    # --- boolean mask ---
    bg_mask = (mask == 0)

    # --- convert to foreground mask ---
    fg_mask = (~bg_mask).astype(np.uint8)

    # --- apply safely ---
    rgi = rgi * fg_mask[:, :, None]
    image_normalized = image_normalized * fg_mask[:, :, None]
    return rgi, image_normalized









