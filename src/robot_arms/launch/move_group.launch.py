import os
import yaml
from launch import LaunchDescription
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
    srdf_path = os.path.join(pkg_share, 'config', 'moveit', 'ur5e.srdf')
    kinematics_path = os.path.join(pkg_share, 'config', 'moveit', 'kinematics.yaml')
    joint_limits_path = os.path.join(pkg_share, 'config', 'moveit', 'joint_limits.yaml')
    moveit_controllers_path = os.path.join(pkg_share, 'config', 'moveit', 'moveit_controllers.yaml')
    ompl_planning_path = os.path.join(pkg_share, 'config', 'moveit', 'ompl_planning.yaml')

    robot_description = {'robot_description': Command(['xacro ', robot_xacro_path])}
    robot_description_semantic = {'robot_description_semantic': load_file(srdf_path)}
    robot_description_kinematics = {'robot_description_kinematics': load_yaml(kinematics_path)}
    robot_description_planning = {'robot_description_planning': load_yaml(joint_limits_path)}

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
    ompl_planning_pipeline_config['move_group'].update(load_yaml(ompl_planning_path))

    moveit_controllers = load_yaml(moveit_controllers_path)

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

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

    return LaunchDescription([move_group_node])
