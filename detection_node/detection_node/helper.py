import numpy as np
import cv2

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
    F_image = np.absolute(imageRGI - BG_image_RGI, np.uint32)
    mask= (F_image < threshold).any(axis = 2)
    imageRGI[mask] = 0
    image_normalized[mask] = 0
    return imageRGI, image_normalized

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

    








