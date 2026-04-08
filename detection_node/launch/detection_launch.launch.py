from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
         Node(
            package='detection_node',
            executable='detection_node',
            name='detection_node',
            output='screen',
            emulate_tty=True,
        ),
    #     Node(
    #        package='detection_node',
    #        executable='detection_publisher',
    #        name='detection_publisher',
    #        output='screen',
    #        emulate_tty=True,
    #    ),
    ])