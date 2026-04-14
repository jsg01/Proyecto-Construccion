#!/usr/bin/env python3
from __future__ import annotations

import os
import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from construccion_interfaces.srv import PixelToRobot, RobotToPixel


class PlaneTransformServer(Node):
    def __init__(self) -> None:
        super().__init__("plane_transform_server")

        self.declare_parameter(
            "plane_yaml",
            os.path.expanduser(
                "~/ros2_ws/src/Proyecto-Construccion/config/plane_calibration.yaml"
            ),
        )
        self.declare_parameter("z_plane", 0.0)

        plane_yaml = self.get_parameter("plane_yaml").value
        self.z_plane = float(self.get_parameter("z_plane").value)

        self.H = self.load_homography(plane_yaml)
        self.H_inv = np.linalg.inv(self.H)

        self.pixel_srv = self.create_service(
            PixelToRobot,
            "pixel_to_robot_plane",
            self.handle_pixel_to_robot,
        )

        self.robot_srv = self.create_service(
            RobotToPixel,
            "robot_to_pixel",
            self.handle_robot_to_pixel,
        )

        self.get_logger().info(f"Homografía cargada desde: {plane_yaml}")
        self.get_logger().info("Servicio /pixel_to_robot_plane listo")
        self.get_logger().info("Servicio /robot_to_pixel listo")

    def load_homography(self, yaml_path: str) -> np.ndarray:
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"No existe el archivo: {yaml_path}")

        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        h_data = data["homography"]["data"]
        H = np.array(h_data, dtype=np.float64).reshape((3, 3))
        return H

    def apply_homography(self, H: np.ndarray, a: float, b: float):
        p = np.array([a, b, 1.0], dtype=np.float64)
        r = H @ p
        r = r / r[2]
        return float(r[0]), float(r[1])

    def handle_pixel_to_robot(self, request, response):
        try:
            x, y = self.apply_homography(self.H, request.u, request.v)
            response.x = x
            response.y = y
            response.z = self.z_plane
            response.success = True
            response.message = "Transformación correcta"
        except Exception as e:
            response.x = 0.0
            response.y = 0.0
            response.z = self.z_plane
            response.success = False
            response.message = f"Error: {e}"

        return response

    def handle_robot_to_pixel(self, request, response):
        try:
            u, v = self.apply_homography(self.H_inv, request.x, request.y)
            response.u = u
            response.v = v
            response.success = True
            response.message = "Transformación correcta"
        except Exception as e:
            response.u = 0.0
            response.v = 0.0
            response.success = False
            response.message = f"Error: {e}"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PlaneTransformServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
