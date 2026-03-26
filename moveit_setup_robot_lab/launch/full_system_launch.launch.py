import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
   ld = LaunchDescription()
   ld.add_action(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for simulation or not.",
        )
   )
   ld.add_action(
      DeclareLaunchArgument(
         "robot_ip", 
         description="IP address by which the robot can be reached.",
         default_value="192.168.0.20"
      )
   )
   robot_ip = LaunchConfiguration("robot_ip")
   use_fake_hardware = LaunchConfiguration("use_fake_hardware")
   simulation_robot_control_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('moveit_setup_robot_lab'), 'launch'),
         '/official_moveit_launch.launch.py']),
      launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
         }.items()
   )
   ld.add_action(simulation_robot_control_launch)
   return ld