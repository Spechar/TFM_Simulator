#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
import yaml
import os
import csv

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')
        self.declare_parameter('k', 0.1)
        self.declare_parameter('wheel_base', 0.3302) #0.6 for spechCar, 0.3302 for rc_car
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('min_speed', 0.4)
        self.declare_parameter('speed_gain', 5.0)
        self.declare_parameter('trajectory_file', '/home/spech/tfm_ws/src/rcauto_sim_pkg/config/line_levineF1.yaml')
        self.declare_parameter('output_dir', '/home/spech/tfm_ws/src/rcauto_sim_pkg/output')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.k = self.get_parameter('k').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.speed_gain = self.get_parameter('speed_gain').get_parameter_value().double_value
        trajectory_file = self.get_parameter('trajectory_file').get_parameter_value().string_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        if not os.path.exists(trajectory_file):
            self.get_logger().error(f"Archivo de trayectoria no encontrado: {trajectory_file}")
            rclpy.shutdown()
            return
        with open(trajectory_file, 'r') as f:
            data = yaml.safe_load(f)
        self.path = [(pt['x'], pt['y'], pt['theta']) for pt in data.get('racing_line', [])]
        self.get_logger().info(f"Trayectoria cargada con {len(self.path)} puntos.")

        csv_path = os.path.join(output_dir, 'real_pathLevineSimPropio2.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x', 'y', 'yaw', 'curvature', 'speed', 'steer'])

        self.lap_completed = False
        self.start_idx = None
        self.max_idx = 0
        self.visited_indices = set()
        self.start_time = None

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.prev_speed = 0.0

    def odom_callback(self, msg: Odometry):
        if not self.path:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = t

        dists = [math.hypot(x - px, y - py) for px, py, _ in self.path]
        idx = int(np.argmin(dists))

        if self.start_idx is None:
            self.start_idx = idx
        self.max_idx = max(self.max_idx, idx)
        self.visited_indices.add(idx)

        if idx > 0 and idx < len(self.path) - 1:
            x_prev, y_prev, _ = self.path[idx - 1]
            x_curr, y_curr, _ = self.path[idx]
            x_next, y_next, _ = self.path[idx + 1]
            v1 = np.array([x_curr - x_prev, y_curr - y_prev])
            v2 = np.array([x_next - x_curr, y_next - y_curr])
            dot = np.dot(v1, v2)
            norm = np.linalg.norm(v1) * np.linalg.norm(v2)
            ang = math.acos(max(-1, min(1, dot / (norm if norm > 1e-6 else 1e-6))))
            ds = np.linalg.norm(v2)
            curvature = ang / ds if ds > 1e-6 else 0.0
        else:
            curvature = 0.0

        target_idx = min(idx + 1, len(self.path) - 1)
        px, py, path_yaw = self.path[target_idx]
        heading_error = math.atan2(math.sin(path_yaw - yaw), math.cos(path_yaw - yaw))
        dx = px - x
        dy = py - y
        cte = -math.sin(yaw) * dx + math.cos(yaw) * dy

        steer = heading_error + math.atan2(self.k * cte, self.max_speed)
        steer = max(-0.34, min(0.34, steer))

        v = self.max_speed / (1.0 + self.speed_gain * abs(curvature))
        v = max(self.min_speed, min(self.max_speed, v))
        v = 0.1 * v + 0.9 * self.prev_speed
        self.prev_speed = v

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = v * steer / self.wheel_base
        self.cmd_pub.publish(cmd)

        if not self.lap_completed:
            self.csv_writer.writerow([t, x, y, yaw, curvature, v, steer])

            elapsed_time = t - self.start_time
            enough_points = len(self.visited_indices) == len(self.path)
            returned_near_start = abs(idx - self.start_idx) < 5

            if elapsed_time > 3.0 and enough_points and returned_near_start:
                self.csv_file.close()
                self.lap_completed = True
                self.get_logger().info('Trayectoria completada. CSV cerrado.')

    def destroy_node(self):
        if not self.csv_file.closed:
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
