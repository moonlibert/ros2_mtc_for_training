"""Panda 仿真环境（本项目专用）

与官方 demo.launch.py 基本相同，唯一区别是给 move_group
追加了 ExecuteTaskSolutionCapability —— MTC 的 task.execute() 依赖它。
官方 demo 没加载这个能力，直接 execute 会报错误码 99999。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit_mtc.rviz",
        description="RViz configuration file",
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    # 关键区别：追加 ExecuteTaskSolutionCapability
    # capabilities 参数是在默认能力之上"追加"，不会覆盖默认能力
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([
                FindPackageShare("mtc_tutorial"),
                "rviz",
                LaunchConfiguration("rviz_config"),
            ]),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
        static_tf_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["panda_arm_controller", "-c", "/controller_manager"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["panda_hand_controller", "-c", "/controller_manager"]),
    ])
