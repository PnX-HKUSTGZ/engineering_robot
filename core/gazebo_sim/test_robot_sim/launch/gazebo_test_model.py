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

namespace="test_robot/"

use_sim_time = LaunchConfiguration('use_sim_time', default=True)

ld.add_action(DeclareLaunchArgument("use_sim_time", default_value=use_sim_time))

test_robot_urdf_path = os.path.join(
    get_package_share_directory("test_robot_sim"),
    "config",
    "test_robot.urdf.xacro"
)

world_path = os.path.join(
    get_package_share_directory("test_robot_sim"),
    "config",
    "empty_world.sdf"
)


def generate_gazebo_launch_description():

    robot_description_content = xacro.process_file(test_robot_urdf_path).toprettyxml(indent="  ") # type: ignore

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('test_robot_sim'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description,],
        # namespace=namespace,
    )
    ld.add_action(node_robot_state_publisher)

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', "robot_description",
                   '-name', 'test_robot', '-allow_renaming', 'true'],
        # namespace=namespace,
    )
    ld.add_action(gz_spawn_entity)

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            "--controller-manager-timeout", "1200", "--switch-timeout", "1000"
            ],
        # namespace=namespace,
    )
    ld.add_action(joint_state_broadcaster_spawner)

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable="spawner",
        name='arm_controller',
        arguments=[
            'arm_controller',
            '--param-file',
            robot_controllers,
            ],
        # namespace=namespace,
    )
    ld.add_action(joint_trajectory_controller_spawner)

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable="spawner",
        name='chasis_controller',
        arguments=[
            'chasis_controller',
            '--param-file',
            robot_controllers,
            ],
        # namespace=namespace,
    )
    ld.add_action(mecanum_drive_controller_spawner)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [f' -r -v 1 {world_path}'])]),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    ld.add_action(bridge)


def generate_launch_description():

    # generate_moveit_launch_description()

    generate_gazebo_launch_description()

    return ld
