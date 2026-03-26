from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml
from os import path

def generate_launch_description():
    ld = LaunchDescription()
    package_name = "moveit_setup_robot_lab"
    ld.add_action(
        DeclareLaunchArgument(
            "launch_rviz_moveit",
            default_value="true",
            description="Launch rviz for moveit",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Launch rviz for moveit",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_ip", 
            description="IP address by which the robot can be reached.",
            default_value="192.168.56.101"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "description_package",
            default_value="moveit_setup_robot_lab",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_ip = LaunchConfiguration("robot_ip")
    # Launch the UR Drive
    # include: robot_description publisher, joint_state publishser, controller_manager
    gantry_system_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("moveit_setup_robot_lab"), "/launch", "/system_control_driver.launch.py"]
        ),
        launch_arguments={
            "launch_rviz": "false",
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )
    # Moveit parameters setup
    moveit_config = MoveItConfigsBuilder(robot_name="gantry_system", package_name=package_name).to_moveit_configs()
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_robot_description_semantic": True,
    }
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": "~/.ros/warehouse_ros.sqlite",
        }
    # Start moveit group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
            planning_scene_monitor_parameters,
        ],
    )
    # Start Rviz node 
    rviz_config_file = PathJoinSubstitution(
    [FindPackageShare(package_name), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
        ],
        condition=IfCondition(launch_rviz_moveit),
    )
    delay_after_gantry_sys_driver = TimerAction(
                            period=3.0,
                            actions=[move_group_node, 
                                    rviz_node,
                                     ])
    # Activate ROS2 nodes and launches
    ld.add_action(gantry_system_driver_launch)
    ld.add_action(delay_after_gantry_sys_driver)
    return ld