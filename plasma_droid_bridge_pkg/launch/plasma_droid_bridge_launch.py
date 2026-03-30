from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                   '/model/box/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   '/model/box/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/target_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/target_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[
            ('/model/box/pose', '/tf'),
            ('/model/target_0/pose', '/tf'),
            ('/model/target_1/pose', '/tf')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        bridge
    ])