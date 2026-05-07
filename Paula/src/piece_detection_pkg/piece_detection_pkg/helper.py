import numpy as np
import cv2

cam = np.array([
    [-0.128, 0.421],
    [-0.150, 0.2898],
    [-0.183, 0.3519]
], dtype=np.float32)

robot = np.array([
    [-0.124, 0.410],
    [-0.151, 0.302],
    [-0.189, 0.369]
], dtype=np.float32)

M = cv2.getAffineTransform(cam, robot)

def transformar(p, M=M):
    x, y = p
    xr = M[0,0]*x + M[0,1]*y + M[0,2]
    yr = M[1,0]*x + M[1,1]*y + M[1,2]
    return xr, yr