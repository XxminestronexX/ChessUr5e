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
        arguments=['-name', 'ur5e_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )

    # Bridge ROS-Gazebo (Clock + SetEntityPose per presa pezzi fluida)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen'
    )

    # Spawners ROS 2 Control
    controller_nodes = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["arm_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["gripper_controller"])
    ]

    # --- COORDINATE SCACCHIERA OTTIMIZZATE PER PICK & PLACE VERTICALE (64/64 CASELLE) ---
    # Tavolo: in Gazebo il robot è spawnato a z=1.0, quindi il piano del tavolo è a z=1.0m.
    # Posizione (-0.58, -0.22): garantisce 100% di raggiungibilità verticale (Roll=pi, Pitch=0)
    # su tutte le 64 caselle sia in presa (Z=+0.035m) sia in hover (Z=+0.16m), lontano dalle singolarità.
    BOARD_X = -0.58
    BOARD_Y = -0.22
    BOARD_Z = 1.01
    BOARD_ROLL = 0.0
    BOARD_PITCH = 0.0
    BOARD_YAW = 3.14159  # 180 gradi in radianti, per allineare con il robot 

    # Spawn Scacchiera Indipendente
    chessboard_description = Command(['xacro ', board_xacro_path])
    spawn_chessboard = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'chessboard',
            '-string', chessboard_description,
            '-x', str(BOARD_X), '-y', str(BOARD_Y), '-z', str(BOARD_Z), '-R', str(BOARD_ROLL), '-P', str(BOARD_PITCH), '-Y', str(BOARD_YAW)
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