from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller_manager",
            "/controller_manager"
        ]
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller_manager",
            "/controller_manager"
        ]
    )
    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simple_controller
    ])