from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveItConfigsBuilder 会把机器人描述、SRDF、OMPL/运动学等参数
    # 打包成 dict，注入到我们的节点中
    # （MTC 的 PipelinePlanner 需要知道用哪个规划插件）
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    return LaunchDescription([
        Node(
            package="mtc_tutorial",
            executable="l1_minimal_task",
            output="screen",
            parameters=[moveit_config],
        )
    ])
