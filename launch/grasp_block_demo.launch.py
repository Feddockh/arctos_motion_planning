from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Get the moveit parameters for the grasping node
    moveit_config = MoveItConfigsBuilder("arctos").to_moveit_configs()

    # Define the grasping node
    grasp_block_demo = Node(
        name="grasp_block_demo",
        package="arctos_motion_planning",
        executable="grasp_block_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_scene_monitor,
            moveit_config.joint_limits,
            moveit_config.move_group_capabilities,
            moveit_config.planning_pipelines,
            moveit_config.moveit_cpp,
            moveit_config.trajectory_execution
        ],
    )

    launched_nodes = LaunchDescription([
        grasp_block_demo
    ])

    return launched_nodes