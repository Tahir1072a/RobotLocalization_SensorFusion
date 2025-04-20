import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("pilbot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("pilbot_controller"),
            "launch",
            "controller.launch.py"       
        )
    )


    localization = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("pilbot_localization"),
            "launch",
            "local_localization.launch.py"
        )
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(
            get_package_share_directory("pilbot_description"),
            "rviz",
            "pilbot.rviz"
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        localization,
        controller,
        rviz2
    ])