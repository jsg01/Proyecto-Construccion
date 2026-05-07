from ur_msgs.srv import SetIO
from construccion_interfaces.srv import MoveCartesian, MoveNamedPose
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


def open_gripper():
    request_message_1 = SetIO.Request()
    request_message_1.fun  = 1
    request_message_1.pin  = 17
    request_message_1.state = 0.0
    request_message_2 = SetIO.Request()
    request_message_2.fun  = 1
    request_message_2.pin  = 16
    request_message_2.state = 1.0
    return request_message_1, request_message_2

def close_gripper():
    request_message_1 = SetIO.Request()
    request_message_1.fun  = 1
    request_message_1.pin  = 16
    request_message_1.state = 0.0
    request_message_2 = SetIO.Request()
    request_message_2.fun  = 1
    request_message_2.pin  = 17
    request_message_2.state = 1.0
    return request_message_1, request_message_2

def approach_to_piece(piece_id: int, file: dict, Z: float):
    
    x_test = file["detections"][piece_id]["robot"]["x"]
    y_test = file["detections"][piece_id]["robot"]["y"]
    #xr, yr = transformar((x_test, y_test))
    xr, yr = (x_test, y_test)
    request_cartesian = MoveCartesian.Request()
    request_cartesian.x = xr
    request_cartesian.y = yr
    request_cartesian.z = Z
    request_cartesian.yaw_deg = 90 - file["detections"][piece_id]["angle_deg"]
    request_cartesian.modo = "down"
    return request_cartesian

def move_to_pose(pose: str):
    request_pose = MoveNamedPose.Request()
    request_pose.pose_name = pose
    return request_pose

def send_piece(x: float, y: float, z: float, angle_deg: float ):
    request_cartesian = MoveCartesian.Request()
    request_cartesian.x = x
    request_cartesian.y = y
    request_cartesian.z = z
    request_cartesian.yaw_deg = angle_deg
    request_cartesian.modo = "down"
    return request_cartesian







