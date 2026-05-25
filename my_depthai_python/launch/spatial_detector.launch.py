from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_urdf_arg = DeclareLaunchArgument(
        'start_urdf',
        default_value='true',
        description='Start camera URDF/TF publisher launch',
    )
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='OAK-D',
        description='DepthAI camera model used by the URDF',
    )
    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value='oak',
        description='TF prefix / camera name for URDF frames',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='oak-d_frame',
        description='Base frame name for camera URDF',
    )
    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='world',
        description='Parent frame to attach camera URDF',
    )
    cam_pos_x_arg = DeclareLaunchArgument(
        'cam_pos_x',
        default_value='0.25',
        description='Camera X position relative to parent frame',
    )
    cam_pos_y_arg = DeclareLaunchArgument(
        'cam_pos_y',
        default_value='0.0',
        description='Camera Y position relative to parent frame',
    )
    cam_pos_z_arg = DeclareLaunchArgument(
        'cam_pos_z',
        default_value='0.5',
        description='Camera Z position relative to parent frame',
    )
    cam_roll_arg = DeclareLaunchArgument(
        'cam_roll',
        default_value='0.0',
        description='Camera roll relative to parent frame',
    )
    cam_pitch_arg = DeclareLaunchArgument(
        'cam_pitch',
        default_value='0.0',
        description='Camera pitch relative to parent frame',
    )
    cam_yaw_arg = DeclareLaunchArgument(
        'cam_yaw',
        default_value='0.0',
        description='Camera yaw relative to parent frame',
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz2 alongside the detector node',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('my_depthai_python'), 'rviz', 'spatial_detector.rviz']
        ),
        description='Absolute path to the RViz2 configuration file',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('my_depthai_python'), 'config', 'spatial_detector.yaml']
        ),
        description='Path to YAML file with spatial_detector ROS parameters',
    )

    spatial_detector_node = Node(
        package='my_depthai_python',
        executable='spatial_detector',
        name='spatial_detector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[LaunchConfiguration('params_file')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('my_depthai'), 'launch', 'urdf_launch.py']
            )
        ),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'tf_prefix': LaunchConfiguration('tf_prefix'),
            'base_frame': LaunchConfiguration('base_frame'),
            'parent_frame': LaunchConfiguration('parent_frame'),
            'cam_pos_x': LaunchConfiguration('cam_pos_x'),
            'cam_pos_y': LaunchConfiguration('cam_pos_y'),
            'cam_pos_z': LaunchConfiguration('cam_pos_z'),
            'cam_roll': LaunchConfiguration('cam_roll'),
            'cam_pitch': LaunchConfiguration('cam_pitch'),
            'cam_yaw': LaunchConfiguration('cam_yaw'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_urdf')),
    )

    return LaunchDescription(
        [
            start_urdf_arg,
            camera_model_arg,
            tf_prefix_arg,
            base_frame_arg,
            parent_frame_arg,
            cam_pos_x_arg,
            cam_pos_y_arg,
            cam_pos_z_arg,
            cam_roll_arg,
            cam_pitch_arg,
            cam_yaw_arg,
            start_rviz_arg,
            rviz_config_arg,
            params_file_arg,
            urdf_launch,
            spatial_detector_node,
            rviz_node,
        ]
    )
