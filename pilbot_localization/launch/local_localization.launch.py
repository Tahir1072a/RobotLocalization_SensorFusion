from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os

def generate_launch_description():

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )
    
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("pilbot_localization"), "config", "ekf.yaml")]
    )
    
    robot_localization_1 = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_single1",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("pilbot_localization"), "config", "ekf.yaml")],
        remappings=[
            ("odometry/filtered", "odometry/filtered_1"),
        ]
    )
    
    robot_localization_2 = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_single2",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("pilbot_localization"), "config", "ekf.yaml")],
        remappings=[
            ("odometry/filtered", "odometry/filtered_2"),
        ]
    )

    robot_localization_3 = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_single3",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("pilbot_localization"), "config", "ekf.yaml")],
        remappings=[
            ("odometry/filtered", "odometry/filtered_3"),
        ]
    )

    imu_republisher_py = Node(
        package="pilbot_localization",
        executable="imu_republisher.py"
    )

    pose_prediction = Node(
        package="pilbot_localization",
        executable="pose_prediction.py"
    )

    imu_logger = Node(
        package="pilbot_localization",
        executable="imu_logger.py"
    )

    pose_logger = Node(
        package="pilbot_localization",
        executable="pose_logger.py"
    )

    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        robot_localization_1,
        robot_localization_2,
        robot_localization_3,
        imu_republisher_py,
        pose_prediction,
        imu_logger,
        pose_logger
    ])