import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'robot_arms'
    robot_xacro = 'uclv_right_ur5e.urdf.xacro'
    board_xacro = 'chessboard.urdf.xacro'
    
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    robot_xacro_path = os.path.join(pkg_share, 'urdf', robot_xacro)
    board_xacro_path = os.path.join(pkg_share, 'urdf', board_xacro)
    
    # Configurazione percorsi risorse Gazebo Sim (src + install)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    src_dir = os.path.join(workspace_root, 'src')
    install_share = os.path.dirname(pkg_share)
    
    set_env_src = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', src_dir)
    set_env_install = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', install_share)

    # Robot State Publisher
    robot_description_content = Command(['xacro ', robot_xacro_path])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )
    
    # Core Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur5e_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )

    # Bridge ROS-Gazebo
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Spawners ROS 2 Control
    controller_nodes = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["arm_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["gripper_controller"])
    ]

    # --- COORDINATE SCACCHIERA (Tavolo sul pavimento, superficie = 0.556631) ---
    BOARD_X = -0.5
    BOARD_Y = -0.15
    BOARD_Z = 0 
    BOARD_YAW = 3.14159  # 180 gradi in radianti, per allineare con il robot 

    # Spawn Scacchiera Indipendente
    chessboard_description = Command(['xacro ', board_xacro_path])
    spawn_chessboard = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'chessboard',
            '-string', chessboard_description,
            '-x', str(BOARD_X), '-y', str(BOARD_Y), '-z', str(BOARD_Z), '-Y', str(BOARD_YAW)
        ],
        output='screen'
    )

    # Lista nodi pulita (Pezzi totalmente rimossi)
    launch_nodes = [
        set_env_src,
        set_env_install,
        gazebo, 
        robot_state_publisher, 
        spawn_robot, 
        spawn_chessboard, 
        clock_bridge
    ] + controller_nodes
    
    return LaunchDescription(launch_nodes)