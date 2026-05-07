from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur3e", package_name="moveit_setup_robot_lab")
        .to_moveit_configs()
    )

    # 🔹 argumentos desde terminal
    x_arg = DeclareLaunchArgument("x", default_value="0.2")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1")

    nodo_xyz = Node(
        package="robot_control_julia",
        executable="move_to_point_ok",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            {
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
            },
        ],
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        nodo_xyz
    ])
