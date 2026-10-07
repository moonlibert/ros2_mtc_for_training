from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # L3 需要机器人描述 + 规划管线参数（IK 求解器配置也在这里）
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    return LaunchDescription([
        Node(
            package="mtc_tutorial",
            executable="l3_pick_place",
            output="screen",
            parameters=[moveit_config],
        )
    ])
