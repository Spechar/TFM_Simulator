import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    run_slam = LaunchConfiguration('run_slam', default='false')

    # Declarar argumentos
    use_gui = LaunchConfiguration('use_gui')
    world_name = LaunchConfiguration('world_name')
    car_model = LaunchConfiguration('car_model')
    slam = LaunchConfiguration('slam_localization')

    # Declarar argumentos para la posición
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='0.0')
    z_pos = LaunchConfiguration('z_pos', default='0.0')
    yaw   = LaunchConfiguration('yaw', default='0.0')

    # Directorio del paquete
    package_dir = os.path.join(os.path.expanduser('~/tfm_ws/src'), 'rcauto_sim_pkg')
    
    # Construir rutas usando PathJoinSubstitution
    world_path = PathJoinSubstitution([package_dir, 'worlds', world_name])
    car_model_path = PathJoinSubstitution([package_dir, 'urdf', car_model])
    slam_config = PathJoinSubstitution([package_dir, 'config', slam])
    # Procesar el archivo xacro para obtener el URDF
    robot_description = Command(['xacro ', car_model_path])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='levine1.world',
            description='Nombre del mundo a cargar en Gazebo'
        ),
        DeclareLaunchArgument(
            'car_model',
            default_value='rc_car.urdf.xacro',
            description='Modelo de coche RC a cargar'
        ),
        DeclareLaunchArgument(
            'slam_localization',
            default_value='slam_localization.yaml',
            description='Configuración de SLAM a cargar'
        ),
        # Argumentos para la posición y orientación
        DeclareLaunchArgument(
            'x_pos',
            default_value='0.0',
            description='Posición en X para el spawn del coche'
        ),
        DeclareLaunchArgument(
            'y_pos',
            default_value='0.0',
            description='Posición en Y para el spawn del coche'
        ),
        DeclareLaunchArgument(
            'z_pos',
            default_value='0.4',
            description='Posición en Z para el spawn del coche'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Orientación (yaw) en radianes para el spawn del coche'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description},
                        {'use_sim_time': True}],
            output='screen'
        ),
        
        # Lanza el conversor de PointCloud2 a LaserScan
        #Node(
        #    package='pointcloud_to_laserscan',
        #    executable='pointcloud_to_laserscan_node',
        #    name='pointcloud_to_laserscan',
        #    output='screen',
        #    remappings=[
        #        ('/cloud_in', '/gazebo_ros_ray_sensor/out'),
        #        ('/scan', '/lidar_scan')
        #    ],
        #    parameters=[{'target_frame': 'lidar_link'}]
        #),
    
        # Lanza slam_toolbox para que utilice el LaserScan en /lidar_scan
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=['/home/spech/tfm_ws/src/rcauto_sim_pkg/config/slam_localization.yaml'],
            condition=IfCondition(run_slam)
        ),
        
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                  # Inicializa ROS en Gazebo
                  '-s', 'libgazebo_ros_init.so',
                  # Permite spawn_entity
                  '-s', 'libgazebo_ros_factory.so',
                  world_path
                ],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'rc_car',
                '-topic', 'robot_description',
                '-x', x_pos,
                '-y', y_pos,
                '-z', z_pos,
                '-Y', yaw
            ],
            output='screen'
        ),
    ])
