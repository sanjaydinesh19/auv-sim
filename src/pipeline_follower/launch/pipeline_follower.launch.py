from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Input camera topic'
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Output velocity command topic'
    )

    pipeline_follower_node = Node(
        package='pipeline_follower',
        executable='pipeline_follower_node',
        name='pipeline_follower_node',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic')),
            ('/cmd_vel',          LaunchConfiguration('cmd_vel_topic')),
        ],
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        cmd_vel_topic_arg,
        pipeline_follower_node,
    ])
