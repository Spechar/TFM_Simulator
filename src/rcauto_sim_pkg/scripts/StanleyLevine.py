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

def resample_path(path, step=0.1):
    """
    Re­samplea una trayectoria dada como [(x0,y0,θ0), (x1,y1,θ1), ...]
    para generar puntos equidistantes en el espacio, separados aproximadamente 'step' metros.
    La orientación θ se interpola suavemente usando seno/coseno.
    """
    if len(path) < 2:
        return path.copy()

    # 1) Calcular distancias acumuladas entre puntos
    distances = [0.0]
    for i in range(1, len(path)):
        x0, y0, _ = path[i - 1]
        x1, y1, _ = path[i]
        d = math.hypot(x1 - x0, y1 - y0)
        distances.append(distances[-1] + d)
    total_length = distances[-1]
    if total_length < 1e-6:
        return path.copy()

    # 2) Generar nuevos s uniformes: 0, step, 2*step, ..., total_length
    num_samples = int(total_length // step)
    if total_length - (num_samples * step) > 1e-6:
        num_samples += 1
    s_new = np.linspace(0.0, total_length, num_samples + 1)

    # 3) Extraer arrays de x, y, θ y de distancias
    xs = np.array([pt[0] for pt in path])
    ys = np.array([pt[1] for pt in path])
    thetas = np.array([pt[2] for pt in path])
    s_old = np.array(distances)

    # 4) Interpolar x(s) e y(s)
    xs_new = np.interp(s_new, s_old, xs)
    ys_new = np.interp(s_new, s_old, ys)

    # Para θ(s), interpolamos seno y coseno por separado
    sin_t = np.sin(thetas)
    cos_t = np.cos(thetas)
    sin_new = np.interp(s_new, s_old, sin_t)
    cos_new = np.interp(s_new, s_old, cos_t)
    thetas_new = np.arctan2(sin_new, cos_new)

    # 5) Construir lista de triples (x,y,θ)
    resampled = [
        (float(xs_new[i]), float(ys_new[i]), float(thetas_new[i]))
        for i in range(len(s_new))
    ]
    return resampled


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # --------------------------------------------------
        # 1) Declarar parámetros
        # --------------------------------------------------
        self.declare_parameter('k', 0.5)                     # Ganancia de cross-track
        self.declare_parameter('wheel_base', 0.3302)         # Distancia entre ejes (m)
        self.declare_parameter('max_speed', 1.0)             # Velocidad máxima (m/s) en recta
        self.declare_parameter('min_speed', 0.3)             # Velocidad mínima (m/s)
        self.declare_parameter('speed_gain', 5.0)            # Ganancia de frenado por curvatura
        self.declare_parameter('step', 0.1)                  # Paso de resample (m)
        self.declare_parameter('lookahead_pts', 0)           # Cuántos puntos de "adelanto" usar
        self.declare_parameter('alpha_steer', 0.2)           # Low-pass en steer
        self.declare_parameter('alpha_speed', 0.1)           # Low-pass en velocidad
        self.declare_parameter('horizon_pts', 5)             # Cantidad de puntos a revisar para la max curvatura
        self.declare_parameter('trajectory_file', '/home/spech/tfm_ws/src/rcauto_sim_pkg/config/line_levine.yaml')
        self.declare_parameter('output_dir', '/home/spech/tfm_ws/src/rcauto_sim_pkg/output')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # --------------------------------------------------
        # 2) Leer parámetros
        # --------------------------------------------------
        self.k            = self.get_parameter('k').get_parameter_value().double_value
        self.wheel_base   = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_speed    = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed    = self.get_parameter('min_speed').get_parameter_value().double_value
        self.speed_gain   = self.get_parameter('speed_gain').get_parameter_value().double_value
        self.step         = self.get_parameter('step').get_parameter_value().double_value
        self.lookahead    = self.get_parameter('lookahead_pts').get_parameter_value().integer_value
        self.alpha_steer  = self.get_parameter('alpha_steer').get_parameter_value().double_value
        self.alpha_speed  = self.get_parameter('alpha_speed').get_parameter_value().double_value
        self.horizon_pts  = self.get_parameter('horizon_pts').get_parameter_value().integer_value

        trajectory_file = self.get_parameter('trajectory_file').get_parameter_value().string_value
        output_dir      = self.get_parameter('output_dir').get_parameter_value().string_value
        odom_topic      = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic       = self.get_parameter('cmd_topic').get_parameter_value().string_value

        # --------------------------------------------------
        # 3) Cargar y re­samplear trayectoria YAML
        # --------------------------------------------------
        if not os.path.exists(trajectory_file):
            self.get_logger().error(f"Archivo de trayectoria no encontrado: {trajectory_file}")
            rclpy.shutdown()
            return

        with open(trajectory_file, 'r') as f:
            data = yaml.safe_load(f)

        raw_list = data.get('racing_line', [])
        if not raw_list:
            self.get_logger().error("La clave 'racing_line' no existe o está vacía")
            rclpy.shutdown()
            return

        raw_path = [(pt['x'], pt['y'], pt['theta']) for pt in raw_list]
        self.get_logger().info(f"Ruta original cargada: {len(raw_path)} puntos")

        # Resamplea a puntos cada ~ self.step [m]
        self.path = resample_path(raw_path, step=self.step)
        self.get_logger().info(f"Ruta resampled: {len(self.path)} puntos equidistantes (step={self.step} m)")

        # --------------------------------------------------
        # 4) Prepara CSV de salida
        # --------------------------------------------------
        os.makedirs(output_dir, exist_ok=True)
        csv_path = os.path.join(output_dir, 'real_pathLevineSim.csv')
        self.csv_file   = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x', 'y', 'yaw', 'curvature', 'speed', 'steer'])

        self.lap_completed = False
        self.visited_indices = set()
        self.start_idx = None
        self.start_time = None

        # Variables para suavizar
        self.prev_speed = 0.0
        self.prev_steer = 0.0

        # --------------------------------------------------
        # 5) Subscripciones y publicador
        # --------------------------------------------------
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 50)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

    def odom_callback(self, msg: Odometry):
        if self.lap_completed:
            return

        # --------------------------------------------------
        # 5.1) Leer pose actual
        # --------------------------------------------------
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Guardar tiempo de inicio
        if self.start_time is None:
            self.start_time = t

        # --------------------------------------------------
        # 5.2) Encontrar índice del punto resampled más cercano
        # --------------------------------------------------
        dists = [math.hypot(x - px, y - py) for px, py, _ in self.path]
        idx = int(np.argmin(dists))

        # Registrar índices visitados para detectar final de vuelta
        if self.start_idx is None:
            self.start_idx = idx
        self.visited_indices.add(idx)

        # --------------------------------------------------
        # 5.3) Cálculo de curvatura con círculo circunscrito
        # --------------------------------------------------
        if 0 < idx < len(self.path) - 1:
            x_prev, y_prev, _ = self.path[idx - 1]
            x_curr, y_curr, _ = self.path[idx]
            x_next, y_next, _ = self.path[idx + 1]
            a = math.hypot(x_curr - x_prev, y_curr - y_prev)
            b = math.hypot(x_next - x_curr, y_next - y_curr)
            c = math.hypot(x_next - x_prev, y_next - y_prev)

            # Área del triángulo mediante determinante (cross product)
            cross = (x_curr - x_prev)*(y_next - y_prev) - (y_curr - y_prev)*(x_next - x_prev)
            area = abs(cross) * 0.5

            if area < 1e-6 or a < 1e-6 or b < 1e-6 or c < 1e-6:
                curvature = 0.0
            else:
                curvature = (4.0 * area) / (a * b * c)
        else:
            curvature = 0.0

        # --------------------------------------------------
        # 5.4) Look-ahead: saltar varios puntos para suavizar
        # --------------------------------------------------
        target_idx = min(idx + self.lookahead, len(self.path) - 1)
        px, py, path_yaw = self.path[target_idx]

        # --------------------------------------------------
        # 5.5) Cálculo heading_error y CTE (cross-track error)
        # --------------------------------------------------
        heading_error = math.atan2(
            math.sin(path_yaw - yaw), math.cos(path_yaw - yaw)
        )
        dx = px - x
        dy = py - y
        cte = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # --------------------------------------------------
        # 5.6) Ley de Stanley para steer
        # --------------------------------------------------
        raw_steer = heading_error + math.atan2(self.k * cte, self.max_speed)
        raw_steer = max(-0.34, min(0.34, raw_steer))  # saturar ángulo
        steer = self.alpha_steer * raw_steer + (1 - self.alpha_steer) * self.prev_steer
        self.prev_steer = steer

        # --------------------------------------------------
        # 5.7) Velocidad adaptativa con predicción de curvatura
        #     Buscamos la curvatura máxima en los próximos 'horizon_pts' puntos
        # --------------------------------------------------
        next_idxs = range(idx, min(idx + self.horizon_pts, len(self.path) - 1))
        max_curv = 0.0
        # Para cada sub-índice, calculemos curvatura aproximada entre i y i+1
        for j in next_idxs:
            if 0 < j < len(self.path) - 1:
                x1, y1, _ = self.path[j - 1]
                x2, y2, _ = self.path[j]
                x3, y3, _ = self.path[j + 1]
                a_ = math.hypot(x2 - x1, y2 - y1)
                b_ = math.hypot(x3 - x2, y3 - y2)
                c_ = math.hypot(x3 - x1, y3 - y1)
                cross_ = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
                area_ = abs(cross_) * 0.5
                if area_ > 1e-6 and a_ > 1e-6 and b_ > 1e-6 and c_ > 1e-6:
                    curv_j = (4.0 * area_) / (a_ * b_ * c_)
                else:
                    curv_j = 0.0
                if curv_j > max_curv:
                    max_curv = curv_j
        # Ley de velocidad en base a max_curv
        v_desired = self.max_speed / (1.0 + self.speed_gain * abs(max_curv))
        v_desired = max(self.min_speed, min(self.max_speed, v_desired))
        # Low-pass en velocidad
        v = self.alpha_speed * v_desired + (1 - self.alpha_speed) * self.prev_speed
        self.prev_speed = v

        # --------------------------------------------------
        # 5.8) Publicar comando Twist
        # --------------------------------------------------
        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = v * steer / self.wheel_base
        self.cmd_pub.publish(cmd)

        # --------------------------------------------------
        # 5.9) Guardar en CSV
        # --------------------------------------------------
        self.csv_writer.writerow([t, x, y, yaw, curvature, v, steer])

        # --------------------------------------------------
        # 5.10) Detectar final de la vuelta (80 % de puntos visitados
        #        y haber retornado cerca del inicio tras >3 s)
        # --------------------------------------------------
        elapsed_time = t - self.start_time
        enough_points = len(self.visited_indices) > 0.8 * len(self.path)
        returned_near_start = abs(idx - self.start_idx) < 5

        if elapsed_time > 3.0 and enough_points and returned_near_start:
            self.lap_completed = True
            self.csv_file.close()
            self.get_logger().info("Trayectoria completada. CSV cerrado.")

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
