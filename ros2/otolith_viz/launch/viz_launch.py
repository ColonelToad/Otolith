from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc = get_package_share_directory("otolith_description")
    # Use HTTP-hosted mesh URDF for Foxglove (package:// needs bridge asset fetch which is flaky with .obj + Chinese mtllib)
    # Fall back to package:// if HTTP not available — both are installed
    urdf_http = os.path.join(desc, "urdf", "go2_http.urdf")
    urdf_pkg = os.path.join(desc, "urdf", "go2.urdf")
    urdf_path = urdf_http if os.path.exists(urdf_http) else urdf_pkg
    with open(urdf_path, "r") as f:
        robot_desc = f.read()
    # HTTP asset server for meshes (so Foxglove can fetch http://localhost:8000/*.obj)
    mesh_dir = os.path.join(desc, "meshes")
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(robot_desc, value_type=str)}],
             remappings=[("/joint_states", "/otolith/joint_states")]),
        Node(package="otolith_fusion", executable="fusion_node", name="otolith_fusion"),
        Node(package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
             parameters=[{"port": 8765}]),
        ExecuteProcess(cmd=["python3", "-m", "http.server", "8000", "--directory", mesh_dir],
                       name="mesh_server", output="screen"),
    ])
