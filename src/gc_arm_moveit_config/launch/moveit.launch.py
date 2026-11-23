"""ros2_control(gc_arm_bringup) + MoveIt move_group + RViz MotionPlanning.

  # 하드웨어 없이
  ros2 launch gc_arm_moveit_config moveit.launch.py mock_all:=true

  # 실제 하드웨어 (joints.yaml에서 enabled: true인 관절만 구동, 나머지 mock)
  ros2 launch gc_arm_moveit_config moveit.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    desc_share = get_package_share_directory("gc_arm_description")
    bringup_share = get_package_share_directory("gc_arm_bringup")
    moveit_share = get_package_share_directory("gc_arm_moveit_config")

    joints_config = LaunchConfiguration("joints_config").perform(context) or os.path.join(
        desc_share, "config", "joints.yaml")
    mock_all = LaunchConfiguration("mock_all").perform(context)
    can_interface = LaunchConfiguration("can_interface").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() == "true"

    # ros2_control 쪽과 동일한 xacro 인자로 robot_description 생성 (불일치 방지)
    moveit_config = (
        MoveItConfigsBuilder("gc_arm", package_name="gc_arm_moveit_config")
        .robot_description(
            file_path=os.path.join(desc_share, "urdf", "gc_arm.urdf.xacro"),
            mappings={
                "joints_config": joints_config,
                "mock_all": mock_all,
                "can_interface": can_interface,
            },
        )
        .robot_description_semantic(file_path="config/gc_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "gc_arm.launch.py")),
        launch_arguments={
            "joints_config": joints_config,
            "mock_all": mock_all,
            "can_interface": can_interface,
            "rviz": "false",
        }.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    actions = [bringup, move_group]
    if use_rviz:
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("joints_config", default_value="",
                              description="관절/하드웨어 설정 YAML (비우면 gc_arm_description 기본값)"),
        DeclareLaunchArgument("mock_all", default_value="false", description="true: 전 관절 mock"),
        DeclareLaunchArgument("can_interface", default_value="auto",
                              description="auto: YAML 값 | can0 | sim"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
