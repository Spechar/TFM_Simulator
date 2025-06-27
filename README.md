# Simulador de Vehículo Autónomo RC en Gazebo + ROS 2

Este repositorio contiene el código y los archivos de configuración desarrollados para un sistema de conducción autónoma aplicado a un coche a escala, como parte de un Trabajo de Fin de Máster (TFM). El proyecto combina una plataforma física basada en F1TENTH con un simulador personalizado construido en ROS 2 Foxy y Gazebo.

## 🚗 Descripción general

El objetivo principal es facilitar la validación de algoritmos de control y planificación en un entorno simulado realista antes de ser desplegados en el vehículo físico. Se incluye:

- Modelo URDF del coche con arquitectura Ackermann
- Circuitos simulados personalizados y oficiales
- Generación automática de mapas y trayectorias
- Controladores de seguimiento (Pure Pursuit y Stanley)
- Comparación entre simulador propio y simulador oficial F1TENTH

## 🧩 Estructura del repositorio
rcauto_sim_pkg/
├── launch/ # Archivos de lanzamiento
├── urdf/ # Modelo del vehículo en formato URDF/XACRO
├── worlds/ # Escenarios de simulación (.world)
├── mapas/ # Mapas SLAM y sus imágenes asociadas
├── config/ # Parámetros y trayectorias en YAML
├── scripts/ # Controladores y generadores de trayectoria
└── output/ # Resultados de simulación y CSVs
## 📦 Requisitos

- Ubuntu 20.04
- ROS 2 Foxy
- Gazebo 11
- Python 3.8
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## ⚙️ Instalación
mkdir -p ~/tfm_ws/src
cd ~/tfm_ws/src
git clone https://github.com/usuario/repositorio-tfm.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

Uso del simulador
-----------------
Para lanzar el simulador con el coche y el entorno:

ros2 launch rcauto_sim_pkg simulador.launch.py

Para lanzar el controlador se necesita navegar hasta la carpeta scripts y ejecutar:
python3 "nombre del archivo"

Visualización de resultados
---------------------------
Los resultados se almacenan en /output como archivos .csv, que contienen información de posición, velocidad, curvatura, y errores del controlador. Pueden analizarse con herramientas como Jupyter, Python o Excel.

Vídeos demostrativos
--------------------
Puedes ver el comportamiento del vehículo en diferentes entornos y simuladores en esta lista de reproducción:
👉 YouTube - Pruebas del simulador: https://www.youtube.com/playlist?list=PLbNqFeyLDhhWiyQo5MnNuhEXPMdDcjFBS

Autor
-----
Trabajo realizado por Sergio Peña, dirigido por Sergio Campos en Universidad Carlos III de Madrid.
Trabajo Fin de Máster del Máster de Robótica y automatización.
