from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    DeclareBooleanLaunchArg,
)


def generate_demo_launch(moveit_config):

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld

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

    # Bring up the node to detect the qr code
    qr_code_detector_node = Node(
        name="qr_code_detector",
        package="simple_object_detection",
        executable="qr_code_detector"
    )

    # Bring up the script to save the cube position
    capture_cube_node = Node(
        name="capture_cube_pose",
        package="arctos_motion_planning",
        executable="capture_cube_pose"
    )

    # RViz Node
    current_package = FindPackageShare('arctos_motion_planning')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            current_package, 'config', 'motion_planning_display.rviz'
        ])]
    )

    # moveit_config = MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config").to_moveit_configs()
    # demo = generate_demo_launch(moveit_config)    

    entities = [
        rviz_node,
        realsense_node,
        qr_code_detector_node,
        capture_cube_node
    ]
    # entities.extend(demo.entities)

    return LaunchDescription(entities)


