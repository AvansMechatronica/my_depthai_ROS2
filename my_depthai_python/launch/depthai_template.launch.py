from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='camera/rgb',
        description='Output image topic for DepthAI RGB frames',
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
        default_value='30.0',
        description='Camera FPS',
    )
    queue_size_arg = DeclareLaunchArgument(
        'queue_size',
        default_value='4',
        description='DepthAI output queue size',
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz2 with the configured RViz file',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('my_depthai_python'), 'rviz', 'depthai_template.rviz']
        ),
        description='Absolute path to the RViz2 configuration file',
    )

    depthai_node = Node(
        package='my_depthai_python',
        executable='depthai_template',
        name='depthai_publisher',
        output='screen',
        parameters=[
            {
                'topic_name': LaunchConfiguration('topic_name'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'queue_size': LaunchConfiguration('queue_size'),
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
            topic_name_arg,
            width_arg,
            height_arg,
            fps_arg,
            queue_size_arg,
            start_rviz_arg,
            rviz_config_arg,
            depthai_node,
            rviz_node,
        ]
    )
