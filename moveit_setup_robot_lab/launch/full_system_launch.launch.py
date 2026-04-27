from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    pkg = FindPackageShare("moveit_setup_robot_lab")

    # 1️⃣ Launch UR driver
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg, "/launch/driver_ur34.launch.py"]
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware
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

    # 4️⃣ Launch RViz (after delay)
    rviz = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [pkg, "/launch/moveit_rviz.launch.py"]
                )
            )
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
        rviz
    ])