from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Get the moveit parameters for the grasping node
    moveit_config = MoveItConfigsBuilder("arctos").to_moveit_configs()

    # Define the grasping node
    move_demo = Node(
        name="move_demo",
        package="arctos_motion_planning",
        executable="move_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    launched_nodes = LaunchDescription([
        move_demo
    ])

    return launched_nodes