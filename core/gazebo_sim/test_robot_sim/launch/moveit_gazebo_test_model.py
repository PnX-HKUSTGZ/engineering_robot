from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import os
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from srdfdom.srdf import SRDF

'''

这个文件启动整个有关于仿真器的部分，并会启动gazebo模拟器本身,moveit,rviz2等

'''
ld= LaunchDescription()

launch_package_path=get_package_share_directory('test_robot_sim')

def generate_launch_description():

    moveit_action=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path + "/launch/moveit_test_model.py")
        ),
    )

    gazebo_action=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path + "/launch/gazebo_test_model.py")
        ),
    )

    ld.add_action(
        moveit_action
    )
    ld.add_action(
        gazebo_action
    )


    return ld
