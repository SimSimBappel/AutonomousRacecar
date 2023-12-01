from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import TimerAction
# from launch.conditions import LaunchCondition
# from launch.conditions import Unless
# from launch.substitutions import LaunchConfiguration
# import launch.conditions


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planner',
            executable='pathplanner',
            name='pathplanner',
            output='screen',
        ),

        Node(
            package='serial2ros',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d' + os.path.join(get_package_share_directory('package_name'), 'config', 'config_file.rviz')]
        ),

        # TimerAction(
        #     period=10.0,  # adjust this period as needed
        #     actions=[
        #         Node(
        #             package='path_planner',
        #             executable='datasim',
        #             name='datasim',
        #             output='screen',
        #         ),
        #     ],
        # ),
        
        

        
    ])