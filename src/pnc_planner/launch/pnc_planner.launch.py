import os

# --- 核心 ---
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("pnc_planner")

    urdf_path = os.path.join(pkg_dir, "urdf", "urdf", "car.urdf")
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    pnc_planner_node = Node(
        package="pnc_planner",
        executable="pnc_planner_node",
        name="pnc_node",
        output="screen",
        parameters=[os.path.join(pkg_dir, "config", "planner_config.yaml")],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "rviz", "default.rviz")],
    )

    return LaunchDescription(
        [
            pnc_planner_node,
            robot_state_pub,
            rviz,
        ]
    )
