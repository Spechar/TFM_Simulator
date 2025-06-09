#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        # Parámetros configurables
        self.declare_parameter('lookahead_distance', 0.3)
        self.declare_parameter('wheel_base', 0.3302)
        self.declare_parameter('velocity', 0.3)
        self.declare_parameter('path_topic', '/racing_line_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.Ld = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.velocity = self.get_parameter('velocity').get_parameter_value().double_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        
        # Lista de waypoints (x, y)
        self.path = []

        # Suscripciones
        self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 50)

        # Publicador de Twist en /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

    def path_callback(self, msg: Path):
        # Guarda solo (x, y) de cada PoseStamped
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f"Recibido path con {len(self.path)} waypoints")

    def odom_callback(self, msg: Odometry):
        if not self.path:
            return

        # Pose actual
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Calcular yaw desde cuaternión
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Filtrar puntos hacia adelante
        ahead = []  # lista de (indice, distancia)
        for i, (px, py) in enumerate(self.path):
            dx = px - x
            dy = py - y
            # proyección sobre la dirección del robot
            if dx * math.cos(yaw) + dy * math.sin(yaw) > 0:
                ahead.append((i, math.hypot(dx, dy)))

        # Seleccionar target: primer punto con distancia ≥ Ld
        if ahead:
            idx_candidates = [i for i, d in ahead if d >= self.Ld]
            if idx_candidates:
                idx = idx_candidates[0]
            else:
                idx = ahead[-1][0]
        else:
            idx = len(self.path) - 1  # fallback al último punto

        # Coordenadas del punto de look-ahead
        px, py = self.path[idx]
        dx = px - x
        dy = py - y

        # Ángulo relativo alpha
        angle_to_point = math.atan2(dy, dx)
        alpha = math.atan2(math.sin(angle_to_point - yaw), math.cos(angle_to_point - yaw))

        # Ley de Pure Pursuit
        delta = math.atan2(2 * self.wheel_base * math.sin(alpha), self.Ld)
        # Velocidad angular omega
        omega = self.velocity * math.tan(delta) / self.wheel_base

        # Publicar comando
        cmd = Twist()
        cmd.linear.x = self.velocity
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(f"Target idx: {idx}, delta: {delta:.3f}, omega: {omega:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()