cmake_minimum_required(VERSION 3.5)
project(rcauto_sim_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Instala archivos de lanzamiento
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---------------------------------------------------------------------------
# Añade aquí para instalar tus scripts Python como ejecutables de ROS 2
# ---------------------------------------------------------------------------
install(
  PROGRAMS
    scripts/show_path.py
    scripts/skelet_path.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Copia la carpeta config/ para que nav2 pueda cargar nav2_config.yaml y racing_line.yaml
# ---------------------------------------------------------------------------
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
# Copia todo tu directorio de mapas para que Nav2 pueda encontrar el YAML
install(
  DIRECTORY mapas/
  DESTINATION share/${PROJECT_NAME}/mapas
)


ament_package()
