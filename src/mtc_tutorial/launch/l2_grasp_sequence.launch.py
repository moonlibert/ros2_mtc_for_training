from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 与 L1 相同：注入机器人描述 + 规划管线参数
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    return LaunchDescription([
        Node(
            package="mtc_tutorial",
            executable="l2_grasp_sequence",
            output="screen",
            parameters=[moveit_config],
        )
    ])
