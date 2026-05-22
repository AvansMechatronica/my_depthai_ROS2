from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='camera/rgb',
        description='Output image topic for DepthAI RGB frames',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='stereo/depth',
        description='Output topic for raw DepthAI depth frames',
    )
    depth_preview_topic_arg = DeclareLaunchArgument(
        'depth_preview_topic',
        default_value='stereo/depth_color',
        description='Output topic for colorized DepthAI depth preview frames',
    )
    detections_topic_arg = DeclareLaunchArgument(
        'detections_topic',
        default_value='spatial_detections',
        description='Output topic for SpatialDetectionArray messages',
    )
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Output image width in pixels',
    )
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='400',
        description='Output image height in pixels',
    )
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='10.0',
        description='Camera FPS',
    )
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='2',
        description='DepthAI output queue size for RGB and detections',
    )
    reconnect_cooldown_arg = DeclareLaunchArgument(
        'reconnect_cooldown_sec',
        default_value='2.0',
        description='Seconds between DepthAI reconnect attempts after link loss',
    )
    max_reconnect_attempts_arg = DeclareLaunchArgument(
        'max_reconnect_attempts',
        default_value='20',
        description='Maximum reconnect attempts; set -1 to retry forever',
    )
    pipeline_mode_arg = DeclareLaunchArgument(
        'pipeline_mode',
        default_value='v3',
        choices=['v3', 'legacy'],
        description='Detector pipeline implementation: v3 (Camera API) or legacy',
    )
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
    depth_source_arg = DeclareLaunchArgument(
        'depth_source',
        default_value='stereo',
        choices=['stereo', 'neural'],
        description='Depth source: stereo (StereoDepth) or neural (NeuralDepth)',
    )
    stereo_extended_disparity_arg = DeclareLaunchArgument(
        'stereo_extended_disparity',
        default_value='false',
        choices=['true', 'false'],
        description='Enable StereoDepth extended disparity (higher disparity range, higher load)',
    )
    blob_name_arg = DeclareLaunchArgument(
        'blob_name',
        default_value='SimpleFruitsYoloV5.blob',
        description='Network blob from the package resources directory',
    )
    config_name_arg = DeclareLaunchArgument(
        'config_name',
        default_value='SimpleFruitsYoloV5.json',
        description='Network config JSON from the package resources directory',
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

    spatial_detector_node = Node(
        package='my_depthai_python',
        executable='spatial_detector',
        name='spatial_detector',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'depth_preview_topic': LaunchConfiguration('depth_preview_topic'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'queue_size': LaunchConfiguration('queue_size'),
                'reconnect_cooldown_sec': LaunchConfiguration('reconnect_cooldown_sec'),
                'max_reconnect_attempts': LaunchConfiguration('max_reconnect_attempts'),
                'pipeline_mode': LaunchConfiguration('pipeline_mode'),
                'depth_source': LaunchConfiguration('depth_source'),
                'stereo_extended_disparity': LaunchConfiguration('stereo_extended_disparity'),
                'blob_name': LaunchConfiguration('blob_name'),
                'config_name': LaunchConfiguration('config_name'),
            }
        ],
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
            image_topic_arg,
            depth_topic_arg,
            depth_preview_topic_arg,
            detections_topic_arg,
            width_arg,
            height_arg,
            fps_arg,
            queue_size_arg,
            reconnect_cooldown_arg,
            max_reconnect_attempts_arg,
            pipeline_mode_arg,
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
            depth_source_arg,
            stereo_extended_disparity_arg,
            blob_name_arg,
            config_name_arg,
            start_rviz_arg,
            rviz_config_arg,
            urdf_launch,
            spatial_detector_node,
            rviz_node,
        ]
    )
