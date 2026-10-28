import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command


def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def load_file(file_path):
    with open(file_path, 'r') as f:
        return f.read()


def generate_launch_description():
    package_name = 'robot_arms'
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    robot_xacro_path = os.path.join(pkg_share, 'urdf', 'uclv_right_ur5e.urdf.xacro')
    board_xacro_path = os.path.join(pkg_share, 'urdf', 'chessboard.urdf.xacro')
    srdf_path = os.path.join(pkg_share, 'config', 'moveit', 'ur5e.srdf')
    kinematics_path = os.path.join(pkg_share, 'config', 'moveit', 'kinematics.yaml')
    joint_limits_path = os.path.join(pkg_share, 'config', 'moveit', 'joint_limits.yaml')
    moveit_controllers_path = os.path.join(pkg_share, 'config', 'moveit', 'moveit_controllers.yaml')
    ompl_planning_path = os.path.join(pkg_share, 'config', 'moveit', 'ompl_planning.yaml')

    # Configurazione percorsi risorse Gazebo Sim (src + install)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    src_dir = os.path.join(workspace_root, 'src')
    install_share = os.path.dirname(pkg_share)

    set_env_src = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', src_dir)
    set_env_install = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', install_share)

    # Robot Description (URDF) e Semantic (SRDF)
    robot_description_content = Command(['xacro ', robot_xacro_path])
    robot_description = {'robot_description': robot_description_content}
    robot_description_semantic = {'robot_description_semantic': load_file(srdf_path)}
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(kinematics_path)}
    robot_description_planning = {'robot_description_planning': load_yaml(joint_limits_path)}

    # Configurazione OMPL Planning Pipeline per MoveIt 2 (Jazzy)
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
        }
    }
    ompl_yaml = load_yaml(ompl_planning_path)
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    moveit_controllers = load_yaml(moveit_controllers_path)

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Nodo MoveIt 2: move_group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    world_sdf_path = os.path.join(pkg_share, 'worlds', 'chess_world.sdf')

    # Core Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': f'-r {world_sdf_path}'}.items()
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur5e_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )

    # Bridge ROS-Gazebo (Clock + SetEntityPose + DeleteEntity per catture + Camera D435)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # Spawners ROS 2 Control
    controller_nodes = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["arm_controller"]),
        Node(package="controller_manager", executable="spawner", arguments=["gripper_controller"]),
    ]

    # Coordinate Scacchiera Ottimizzate (-0.58, -0.22, 1.01)
    BOARD_X = -0.58
    BOARD_Y = -0.22
    BOARD_Z = 1.01
    BOARD_ROLL = 0.0
    BOARD_PITCH = 0.0
    BOARD_YAW = 3.14159

    chessboard_description = Command(['xacro ', board_xacro_path])
    spawn_chessboard = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'chessboard',
            '-string', chessboard_description,
            '-x', str(BOARD_X), '-y', str(BOARD_Y), '-z', str(BOARD_Z),
            '-R', str(BOARD_ROLL), '-P', str(BOARD_PITCH), '-Y', str(BOARD_YAW)
        ],
        output='screen'
    )

    camera_node = Node(
        package='robot_arms',
        executable='camera_board_recognition_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    stockfish_node = Node(
        package='robot_arms',
        executable='stockfish_node.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    launch_nodes = [
        set_env_src,
        set_env_install,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_chessboard,
        clock_bridge,
        move_group_node,
        camera_node,
        stockfish_node,
    ] + controller_nodes

    return LaunchDescription(launch_nodes)
