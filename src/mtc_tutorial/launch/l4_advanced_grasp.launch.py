from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="mtc_tutorial",
            executable="l4_advanced_grasp",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )
    ])
