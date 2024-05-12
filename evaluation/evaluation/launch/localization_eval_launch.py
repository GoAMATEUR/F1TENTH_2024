from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='evaluation',  
            executable='localization_eval',
            name='localization_eval', 
            output='screen',
            parameters=[
                {'gt_pose_topic_name': '/ego_racecar/odom'},
                {'estimated_pose_topic_name': '/ego_racecar/odom'}
            ] 
        )
    ])