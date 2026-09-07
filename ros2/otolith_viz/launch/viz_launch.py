from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc = get_package_share_directory("otolith_description")
    urdf = os.path.join(desc, "urdf", "go2.urdf")
    with open(urdf, "r") as f:
        robot_desc = f.read()
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(robot_desc, value_type=str)}],
             remappings=[("/joint_states", "/otolith/joint_states")]),
        Node(package="otolith_fusion", executable="fusion_node", name="otolith_fusion"),
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
             parameters=[{"port": 8765}]),
    ])
