#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
import yaml
import os
import csv

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        # Parámetros configurables
        self.declare_parameter('k', 1.0)  # ganancia del controlador
        self.declare_parameter('wheel_base', 0.3302)
        self.declare_parameter('velocity', 0.5)
        self.declare_parameter('trajectory_file', '/home/spech/tfm_ws/src/rcauto_sim_pkg/config/pruebaPath.yaml')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.velocity = self.get_parameter('velocity').get_parameter_value().double_value
        trajectory_file = self.get_parameter('trajectory_file').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        # Cargar trayectoria desde YAML
        if not os.path.exists(trajectory_file):
            self.get_logger().error(f"Archivo de trayectoria no encontrado: {trajectory_file}")
            rclpy.shutdown()
            return

        with open(trajectory_file, 'r') as f:
            data = yaml.safe_load(f)
            self.path = [(pt['x'], pt['y'], pt['theta']) for pt in data['racing_line']]

        self.get_logger().info(f"Trayectoria cargada con {len(self.path)} puntos.")
        
        # Abrir archivo CSV para guardar trayectoria real
        self.csv_file = open('/home/spech/tfm_ws/src/rcauto_sim_pkg/output/real_path.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'yaw'])


        # Suscripciones y publicador
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 50)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

    def odom_callback(self, msg):
        if not self.path:
            return

        # Pose actual
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        # Guardar posición en el CSV
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv_writer.writerow([timestamp, x, y, yaw])

        # Punto más cercano en la trayectoria
        dists = [math.hypot(x - px, y - py) for px, py, _ in self.path]
        idx = int(np.argmin(dists))
        target_idx = min(idx + 1, len(self.path) - 1)
        px, py, path_yaw = self.path[target_idx]

        # Heading error
        heading_error = math.atan2(math.sin(path_yaw - yaw), math.cos(path_yaw - yaw))

        # Cross-track error
        dx = px - x
        dy = py - y
        cte = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # Control Stanley
        steer = heading_error + math.atan2(self.k * cte, self.velocity)
        steer = max(min(steer, 0.34), -0.34)  # limitar dirección

        # Publica comando Twist
        cmd = Twist()
        cmd.linear.x = self.velocity
        cmd.angular.z = steer * self.velocity / self.wheel_base
        self.cmd_pub.publish(cmd)

        self.get_logger().debug(f"cte={cte:.3f}, heading_error={heading_error:.3f}, steer={steer:.3f}")

    def destroy_node(self):
        self.get_logger().info("Cerrando archivo CSV de trayectoria real.")
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
