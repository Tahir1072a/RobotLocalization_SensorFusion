from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"]
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pilbot_controller",
                   "--controller-manager", "/controller_manager"]
    )

    pilbot_real_pose_broadcaster = Node(
        package="pilbot_controller",
        executable="pilbot_real_pose_broadcaster"
    )

    return LaunchDescription([
        joint_state_broadcaster,
        pilbot_real_pose_broadcaster,
        wheel_controller_spawner
    ])