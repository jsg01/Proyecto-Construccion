import numpy as np
import cv2

def compute_thresholds(rgi: np.ndarray, roi: tuple, lower_pct: int, upper_pct: int):
    """
    This function outputs a dictionary with the max and minimum thresholds for detecting color
    
    Args:
        rgi (np.ndarray): The image where the cropping will be performed, in RGI format.
        roi (tuple): A tuple with the coordinates of the upper left corner of a piece, its width and its height.
        lower_pct (int): The lower percentile of the threshold
        upper_pct (int): The upper percentile of the threshold

    Returns:
        thresholds (dict): {R_min, R_max, G_min, G_max, mass}
    """

    x, y, w, h = roi
    patch = rgi[y:y+h, x:x+w]
    R = patch[:, :, 0].reshape(-1)
    G = patch[:, :, 1].reshape(-1)
    thresholds = {
        "R_min": float(np.percentile(R, lower_pct)),
        "R_max": float(np.percentile(R, upper_pct)),
        "G_min": float(np.percentile(G, lower_pct)),
        "G_max": float(np.percentile(G, upper_pct)),
        "mass": w * h
    }
    return thresholds

def classify_pixels(rgi, thresholds):
    """
    Creates a mask with the pixels that are in between the thresholds
    Args:
        rgi(np.ndarray): Image in RGI format
        thresholds(dict): A dictionary with the thresholds for color classification
    Returns:
        mask(np.ndarray): A boolean matrix with True value for pixels within boundaries
        and False for everything else 
    """

    R = rgi[:, :, 0]
    G = rgi[:, :, 1]
    I = rgi[:, :, 2]

    mask = ((R >= thresholds["R_min"]) & 
            (R <= thresholds["R_max"]) & 
            (G >= thresholds["G_min"]) & 
            (G <= thresholds["G_max"]) &
            (I > 1)
            )
    return mask

def select_roi(image: np.ndarray):
    """
    Lets the user draw a rectangle on the image.
    
    Args:
        image(np.array): Image where to get the ROI
    Returns:
        (x, y, w, h) (int) x, y coordinates for the upper left corner and the width and height of the block 
    """
    
    roi = cv2.selectROI("Select ROI", image, showCrosshair=True, fromCenter=False)
    x, y, w, h = roi
    print(f"Coordinates gotten: x={x}, y={y}, w={w}, h={h}")
    cv2.destroyWindow("Select ROI")
    cv2.waitKey(1)
    return int(x), int(y), int(w), int(h)

def detect_all_colors(rgi_image: np.ndarray, config_dict: dict) -> dict:
    """
    Returns a dictionary of binary masks per color
    """

    masks = {}

    for color_name, config in config_dict.items():
        mask = classify_pixels(rgi_image, config)
        masks[color_name] = mask

    return masks