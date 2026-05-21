from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='camera/rgb',
        description='Output image topic for DepthAI RGB frames',
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
        default_value='20.0',
        description='Camera FPS',
    )
    depth_source_arg = DeclareLaunchArgument(
        'depth_source',
        default_value='stereo',
        choices=['stereo', 'neural'],
        description='Depth source: stereo (StereoDepth) or neural (NeuralDepth)',
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
        parameters=[
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'depth_source': LaunchConfiguration('depth_source'),
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

    return LaunchDescription(
        [
            image_topic_arg,
            detections_topic_arg,
            width_arg,
            height_arg,
            fps_arg,
            depth_source_arg,
            blob_name_arg,
            config_name_arg,
            start_rviz_arg,
            rviz_config_arg,
            spatial_detector_node,
            rviz_node,
        ]
    )
