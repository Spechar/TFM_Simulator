#!/usr/bin/env python3

import os
import yaml
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.pub = self.create_publisher(Path, 'racing_line_path', 10)

        # Carga tu YAML
        pkg = get_package_share_directory('rcauto_sim_pkg')
        yaml_file = os.path.join(pkg, 'config', 'line_levine1.yaml')
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        # Construye el Path
        path = Path()
        path.header.frame_id = 'map'
        now = self.get_clock().now().to_msg()
        path.header.stamp = now

        for wp in data['racing_line']:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = now
            ps.pose.position.x = wp['x']
            ps.pose.position.y = wp['y']
            ps.pose.position.z = 0.0
            theta = wp['theta']
            ps.pose.orientation.z = math.sin(theta/2.0)
            ps.pose.orientation.w = math.cos(theta/2.0)
            path.poses.append(ps)

        # Publica una sola vez (o en bucle si prefieres)
        self.timer = self.create_timer(1.0, lambda: self.pub.publish(path))

def main():
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()