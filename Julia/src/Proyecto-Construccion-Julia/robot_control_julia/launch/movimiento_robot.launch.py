from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("ur3e", package_name="moveit_setup_robot_lab")
        .to_moveit_configs()
    )

    nodo = Node(
        package="robot_control_julia",
        executable="movimiento_robot",
        name="movimiento_robot",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([
        nodo,
    ])
