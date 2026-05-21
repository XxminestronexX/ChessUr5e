import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'robot_arms'
    xacro_file = 'uclv_right_ur5e.urdf.xacro'
    
    # Percorso del file Xacro
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_file)
    
    # Genera la descrizione del robot tramite Xacro
    robot_description_content = Command(['xacro ', xacro_path])
    
    # Nodo Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )
    
    # Azione per avviare Gazebo Sim (Mondo vuoto)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )
    
    # Nodo per inserire (spawn) il robot dentro Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur5e_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])