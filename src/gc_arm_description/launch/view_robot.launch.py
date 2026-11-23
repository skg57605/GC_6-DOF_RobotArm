"""URDF 확인용: robot_state_publisher + joint_state_publisher_gui + RViz (하드웨어/ros2_control 없음).

ros2 launch gc_arm_description view_robot.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare("gc_arm_description")
    joints_config = LaunchConfiguration("joints_config")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([desc_share, "urdf", "gc_arm.urdf.xacro"]),
            " joints_config:=", joints_config,
            " use_ros2_control:=false",
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "joints_config",
            default_value=PathJoinSubstitution([desc_share, "config", "joints.yaml"]),
            description="관절/하드웨어 설정 YAML",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([desc_share, "rviz", "view.rviz"])],
        ),
    ])
