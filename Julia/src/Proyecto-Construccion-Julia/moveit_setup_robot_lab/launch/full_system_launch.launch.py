from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    camera_setup = ExecuteProcess(
        cmd=[
            'v4l2-ctl', '-d', '/dev/video2',
            '-c', 'white_balance_automatic=0',
            '-c', 'focus_automatic_continuous=0',
            '-c', 'focus_absolute=45',
            '-c', 'white_balance_temperature=4652',
            '-c', 'auto_exposure=1',
            '-c', 'exposure_time_absolute=250',
            '-c', 'gain=2',
            '-c', 'power_line_frequency=1'
        ],
        output='screen'
    )

    pkg = FindPackageShare("moveit_setup_robot_lab")

    # 1️⃣ Launch UR driver
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg, "/launch/driver_ur34.launch.py"]
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
             "launch_rviz": "false"
        }.items()
    )

    # 2️⃣ Load controller
    load_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "scaled_joint_trajectory_controller"
        ],
        output="screen"
    )

    # 3️⃣ Launch move_group (after delay)
    move_group = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [pkg, "/launch/move_group.launch.py"]
                )
            )
        ]
    )
    movimiento_julia = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("robot_control_julia"),
                "launch",
                "movimiento_robot.launch.py"
            ])
        )
    )
    camera = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': '/dev/video2'},
                {'image_width': 1280},
                {'image_height': 720},
            ]
        )
    calibration_node = Node(
        package="calibration_services",
        executable="plane_transform_server",
        name="plane_transform_server",
        output="screen"
    )

    # 4️⃣ Launch RViz (after delay)
    rviz = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [pkg, "/launch/moveit_rviz.launch.py"]
                )
            ), movimiento_julia, calibration_node
        ]
    )


    # 5️⃣ Delay controller load slightly (important)
    delayed_controller = TimerAction(
        period=2.0,
        actions=[load_controller]
    )

    return LaunchDescription([
        driver,
        delayed_controller,
        move_group,
        rviz,
        #camera,
        #camera_setup,
    ])