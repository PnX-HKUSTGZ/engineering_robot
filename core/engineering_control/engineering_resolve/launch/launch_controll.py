from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
import os
import xacro
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import launch_ros.descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, LogInfo
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from srdfdom.srdf import SRDF
from launch.conditions import IfCondition

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("engineering_robot", package_name="engineering_robot_moveit")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines("ompl", ["ompl"])
        .to_moveit_configs()
    )

    robot_controllerPath = get_package_share_directory("engineering_robot_controller")
    engineering_robot_hardware_interfacePath = get_package_share_directory("engineering_robot_hardware_interface")
    config_path = os.path.join(robot_controllerPath,"config/sample_config.yaml")
    use_sim_time={"use_sim_time": True}
    real_controllers_config={"":os.path.join(engineering_robot_hardware_interfacePath,"engineering_robot_hardware_interface.xml")}

    ld = LaunchDescription()

    ld.add_action(
        Node(
        package="engineering_resolve",
        executable="engineering_resolve",
        name="engineering_resolve",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            use_sim_time,
            ],
        )
    )

    return ld

# ros2 launch engineering_robot_controller launch.py  use_rviz:="True"