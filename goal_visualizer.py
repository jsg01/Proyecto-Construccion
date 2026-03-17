#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA


class GoalVisualizer(Node):
    def __init__(self):
        super().__init__('goal_visualizer')

        self.marker_pub = self.create_publisher(Marker, '/debug_markers', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/debug_goal_pose', 10)

        self.sub = self.create_subscription(
            Point,
            '/goal_point',
            self.goal_callback,
            10
        )

        self.timer = self.create_timer(0.5, self.publish_debug_data)

        self.detected_point = (0.40, 0.10, 0.20)
        self.target_point = (0.45, 0.05, 0.25)
        self.target_yaw = math.radians(90.0)

        self.get_logger().info('Goal visualizer iniciado. Escuchando /goal_point')

    def goal_callback(self, msg: Point):
        self.target_point = (msg.x, msg.y, msg.z)
        self.get_logger().info(
            f'Nuevo goal_point: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}'
        )

    def make_color(self, r, g, b, a=1.0):
        c = ColorRGBA()
        c.r = r
        c.g = g
        c.b = b
        c.a = a
        return c

    def make_sphere_marker(self, marker_id, xyz, color, ns):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = xyz[0]
        m.pose.position.y = xyz[1]
        m.pose.position.z = xyz[2]
        m.pose.orientation.w = 1.0

        m.scale.x = 0.03
        m.scale.y = 0.03
        m.scale.z = 0.03

        m.color = color
        return m

    def make_arrow_marker(self, marker_id, xyz, yaw, color, ns):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD

        start = Point()
        start.x = xyz[0]
        start.y = xyz[1]
        start.z = xyz[2]

        end = Point()
        end.x = xyz[0] + 0.08 * math.cos(yaw)
        end.y = xyz[1] + 0.08 * math.sin(yaw)
        end.z = xyz[2]

        m.points = [start, end]

        m.scale.x = 0.01
        m.scale.y = 0.02
        m.scale.z = 0.02

        m.color = color
        return m

    def publish_goal_pose(self, xyz, yaw):
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = xyz[0]
        msg.pose.position.y = xyz[1]
        msg.pose.position.z = xyz[2]

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.pose_pub.publish(msg)

    def publish_debug_data(self):
        detected_marker = self.make_sphere_marker(
            marker_id=0,
            xyz=self.detected_point,
            color=self.make_color(0.0, 1.0, 0.0, 1.0),
            ns='detected_point'
        )

        target_marker = self.make_sphere_marker(
            marker_id=1,
            xyz=self.target_point,
            color=self.make_color(1.0, 0.0, 0.0, 1.0),
            ns='target_point'
        )

        yaw_marker = self.make_arrow_marker(
            marker_id=2,
            xyz=self.target_point,
            yaw=self.target_yaw,
            color=self.make_color(0.0, 0.0, 1.0, 1.0),
            ns='target_yaw'
        )

        self.marker_pub.publish(detected_marker)
        self.marker_pub.publish(target_marker)
        self.marker_pub.publish(yaw_marker)

        self.publish_goal_pose(self.target_point, self.target_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = GoalVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
