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
    
    # Declare ROS parameters
    package_name = "robot_system"

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
    # Initial parameters
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz_moveit = LaunchConfiguration("launch_rviz_moveit")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_ip = LaunchConfiguration("robot_ip")
   


    # Launch the UR Drive
    # include: robot_description publisher, joint_state publishser, controller_manager
    gantry_system_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gantry_description"), "/launch", "/system_control_driver.launch.py"]
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
        # "publish_state_updates": True,
        # "publish_transforms_updates": True,
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

    # Servo for realtime control 
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    gantry_control_node = Node(
        package="gantry_control",
        executable="gantry_control",
        output="log",
        condition=UnlessCondition(use_fake_hardware),
    )

    gantry_joint_state_node = Node(
        package="gantry_control",
        executable="gantry_joint_state_publisher",
        output="log",
        condition=UnlessCondition(use_fake_hardware),        
    )

    gantry_emergency_stop_node = Node(
        package="gantry_control",
        executable="gantry_stop",
        output="log",
        condition=UnlessCondition(use_fake_hardware),        
    )
    
    gantry_joint_state_node_delay = RegisterEventHandler(
            OnProcessStart(
                target_action=gantry_control_node,
                on_start=[gantry_joint_state_node]
            )
        )
    
    gantry_emergency_stop_node_delay = RegisterEventHandler(
            OnProcessStart(
                target_action=gantry_joint_state_node,
                on_start=[gantry_emergency_stop_node]
            )
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

    rviz_node_delay = RegisterEventHandler(
            OnProcessStart(
                target_action=gantry_emergency_stop_node,
                on_start=[rviz_node]
            )
        )

    # node for service move by planning
    moveit_robot_node =  Node(
            package='moveit_lux',
            executable='moveit_service',
            name='moveit_service_lux',
            output='both', 
            parameters=[
                        {"use_sim_time": use_sim_time}
                        ]
        )
    moveit_fake_gantry_node =  Node(
            package='moveit_lux',
            executable='moveit_service_node_fake_gantry',
            name='moveit_service_node_fake_gantry',
            output='both',
            condition=IfCondition(use_fake_hardware),
            parameters=[
                        {"use_sim_time": use_sim_time}
                        ]      
    )

    moveit_cartesian =  Node(
            package='moveit_lux',
            executable='moveit_service_node_cartesian',
            name='moveit_service_node_cartesian',
            output='both',
            parameters=[
                        {"use_sim_time": use_sim_time}
                        ]      
    )


    moveit_cartesian =  Node(
            package='moveit_lux',
            executable='moveit_service_node_cartesian',
            name='moveit_cartesian',
            output='both',
            parameters=[
                        {"use_sim_time": use_sim_time}
                        ]      
    )

    move_linear_client_node=Node(
            package='move_linear_client',
            executable='move_linear_client',
            name='move_linear_client_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_fake_hardware': use_fake_hardware}
            ]
    )

    pid_controller_node = Node(
         package='move_linear_client',
            executable='pid_controller',
            name='pid_controller_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_fake_hardware': use_fake_hardware}
            ]

    )

    move_linear_brush = Node(
            package='move_linear_client',
            executable='move_linear_brush',
            name='move_linear_brush_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'gap_between_tcp_and_brush': gap_between_tcp_and_brush},
                {'brushing_distance': brushing_distance}
            ]
        )
    
    hand_eye_orientation_node = Node(
        package='hand_eye_orientation',
        executable='hand_eye_orientation',
        name='hand_eye_orientation_node',
        output='screen'
    )

  
    
    delay_after_gantry_sys_driver = TimerAction(
                            period=3.0,
                            actions=[move_group_node, 
                                    #  servo_node, # (Temporary disable since require use_sim_time:=false)
                                     moveit_robot_node,
                                     moveit_fake_gantry_node,
                                     gantry_control_node,
                                     gantry_joint_state_node_delay,
                                     gantry_emergency_stop_node_delay,
                                     rviz_node,
                                     move_linear_client_node,
                                     #pid_controller_node,
                                     #pilz_planner
                                     moveit_cartesian,
                                   
                                    #hand_eye_orientation_node
                                    #  rviz_node_delay, 
                                     ]
                            )
    
    force_torque_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robotiq_ft_sensor_hardware"), "/launch", "/ft_sensor_standalone.launch.py"]
        ),
         condition=UnlessCondition(use_fake_hardware) 
        
    )

    
    # Activate ROS2 nodes and launches
    ld.add_action(gantry_system_driver_launch)
    ld.add_action(force_torque_sensor_launch)
    ld.add_action(delay_after_gantry_sys_driver)
    
    return ld