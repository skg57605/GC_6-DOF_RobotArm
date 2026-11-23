"""GC 6축 로봇팔 ros2_control 기동.

  # 하드웨어 없이 전 관절 mock
  ros2 launch gc_arm_bringup gc_arm.launch.py mock_all:=true

  # 시뮬레이션 CAN 버스 (joints.yaml에서 enabled: true인 관절이 실제 프로토콜 경로로 동작)
  ros2 launch gc_arm_bringup gc_arm.launch.py can_interface:=sim

  # 실제 하드웨어 (joints.yaml의 enabled: true 관절만 can0으로 구동, 나머지 mock)
  ros2 launch gc_arm_bringup gc_arm.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = FindPackageShare("gc_arm_description")
    bringup_share = FindPackageShare("gc_arm_bringup")

    joints_config = LaunchConfiguration("joints_config")
    mock_all = LaunchConfiguration("mock_all")
    can_interface = LaunchConfiguration("can_interface")
    rviz = LaunchConfiguration("rviz")

    declared = [
        DeclareLaunchArgument(
            "joints_config",
            default_value=PathJoinSubstitution([desc_share, "config", "joints.yaml"]),
            description="관절/하드웨어 설정 YAML",
        ),
        DeclareLaunchArgument("mock_all", default_value="false", description="true: 전 관절 mock"),
        DeclareLaunchArgument(
            "can_interface", default_value="auto",
            description="auto: YAML 값 사용 | can0 | sim (시뮬레이션 버스)",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
    ]

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([desc_share, "urdf", "gc_arm.urdf.xacro"]),
            " joints_config:=", joints_config,
            " mock_all:=", mock_all,
            " can_interface:=", can_interface,
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="both",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[PathJoinSubstitution([bringup_share, "config", "controllers.yaml"])],
        remappings=[("~/robot_description", "/robot_description")],
        output="both",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([desc_share, "rviz", "view.rviz"])],
        condition=IfCondition(rviz),
    )

    # joint_state_broadcaster가 뜬 뒤 arm_controller 기동
    arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )

    return LaunchDescription(declared + [
        robot_state_publisher,
        control_node,
        jsb_spawner,
        arm_after_jsb,
        rviz_node,
    ])
