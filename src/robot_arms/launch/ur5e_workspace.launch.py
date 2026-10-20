from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type of UR robot.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="robot_arms",
            description="Package containing robot URDF.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="uclv_right_ur5e.urdf.xacro",
            description="URDF/XACRO description file.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names.",
        ),
    ]

    ur_type = LaunchConfiguration("ur_type")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "workspace_view.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    workspace_visualizer_node = Node(
        package="robot_arms",
        executable="workspace_visualizer.py",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        workspace_visualizer_node,
        rviz_node,
    ])
