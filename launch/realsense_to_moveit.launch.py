from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Find the package where the launch files are located
    realsense2_camera_path = FindPackageShare('realsense2_camera')

    # Launch the realsense package
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense2_camera_path, '/launch/rs_launch.py']),
        launch_arguments = {
            'camera_name':                      'camera',
            'camera_namespace':                 'camera',
            'accelerate_gpu_with_glsl':         'false',
            'output':                           'screen',
            'enable_color':                     'true',
            'enable_depth':                     'true',
            'depth_module.enable_auto_exposure':'true',
            'enable_sync':                      'false',
            'clip_distance':                    '-2.0',
            'publish_tf':                       'true',
            'tf_publish_rate':                  '14.0',
            'pointcloud.enable':                'true',
            'pointcloud.stream_filter':         '2',
            'pointcloud.stream_index_filter':   '0',
            'pointcloud.ordered_pc':            'false',
            'align_depth.enable':               'true',
            'colorizer.enable':                 'false',
            'decimation_filter.enable':         'true',
            'spatial_filter.enable':            'true',
            'temporal_filter.enable':           'true',
            'disparity_filter.enable':          'false',
            'hole_filling_filter.enable':       'false',
            'hdr_merge.enable':                 'false',
            'wait_for_device_timeout':          '-1.0',
            'reconnect_timeout':                '6.0',
        }.items(),
    )

    # Get the moveit parameters for the grasping node
    moveit_config = MoveItConfigsBuilder("arctos").to_moveit_configs()

    # Define the grasping node
    realsense_to_moveit_node = Node(
        name="realsense_to_moveit_node",
        package="arctos_motion_planning",
        executable="realsense_to_moveit_node",
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
        realsense_node,
        realsense_to_moveit_node
    ])

    return launched_nodes