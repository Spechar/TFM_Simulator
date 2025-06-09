import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def previsualizar_mapa(img_bin, scale=0.05, wall_height=1.5, wall_thickness=0.01):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    height, width = img_bin.shape
    map_width_m = width * scale
    map_height_m = height * scale

    for y in range(height):
        x = 0
        while x < width:
            if img_bin[y, x] == 255:
                start_x = x
                while x < width and img_bin[y, x] == 255:
                    x += 1
                end_x = x

                length = (end_x - start_x) * scale
                center_x = ((start_x + end_x) / 2) * scale - (map_width_m / 2)
                center_y = (height - y) * scale - (map_height_m / 2)

                ax.bar3d(center_x, center_y, 0, length, wall_thickness, wall_height,
                         color='gray', edgecolor='black', alpha=0.8)
            else:
                x += 1

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Altura (m)')
    ax.set_title("Previsualización del Mapa 3D (centrado en origen)")
    plt.tight_layout()
    plt.show()


def mapa_2d_a_3d(input_map_path, output_world_path, scale=0.05, wall_height=1.5, wall_thickness=0.05):
    img = cv2.imread(input_map_path, cv2.IMREAD_GRAYSCALE)
    _, img_bin = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)

    height, width = img_bin.shape
    map_width_m = width * scale
    map_height_m = height * scale
    sdf_blocks = ""

    block_id = 0

    for y in range(height):
        x = 0
        while x < width:
            if img_bin[y, x] == 255:
                start_x = x
                while x < width and img_bin[y, x] == 255:
                    x += 1
                end_x = x

                length = (end_x - start_x) * scale
                center_x = ((start_x + end_x) / 2) * scale - (map_width_m / 2)
                center_y = (height - y) * scale - (map_height_m / 2)

                sdf_blocks += f"""
      <collision name='collision_{block_id}'>
        <pose>{center_x} {center_y} {wall_height / 2} 0 0 0</pose>
        <geometry>
          <box>
            <size>{length} {wall_thickness} {wall_height}</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual_{block_id}'>
        <pose>{center_x} {center_y} {wall_height / 2} 0 0 0</pose>
        <geometry>
          <box>
            <size>{length} {wall_thickness} {wall_height}</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
                """
                block_id += 1
            else:
                x += 1

    world_template = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <world name='default'>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <model name="map_model">
      <static>true</static>
      <link name="map_link">
        {sdf_blocks}
      </link>
    </model>

  </world>
</sdf>
"""

    with open(output_world_path, "w") as f:
        f.write(world_template)

    print(f"Mundo generado: {output_world_path} con modelo único")

    # Mostrar previsualización 3D centrada
    previsualizar_mapa(img_bin, scale, wall_height, wall_thickness)


# EJEMPLO DE USO
if __name__ == "__main__":
    ruta_mapa_2d = "/home/spech/tfm_ws/src/rcauto_sim_pkg/mapas/levine/levine1.png"        # Ruta de entrada
    ruta_salida_3d = "/home/spech/tfm_ws/src/rcauto_sim_pkg/worlds/levine1.world"  # Ruta de salida

    mapa_2d_a_3d(ruta_mapa_2d, ruta_salida_3d)