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
import yaml
import os
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from srdfdom.srdf import SRDF


def extract_controller_names(yaml_str):
    """
    从 YAML 字符串中提取 controller_manager: ros__parameters: 下除了 update_rate 以外的所有键。

    Args:
        yaml_str (str): 包含 YAML 配置的字符串。

    Returns:
        list: 包含控制器名称的列表。
    """
    try:
        # 解析 YAML 字符串
        data = yaml.safe_load(yaml_str)

        # 检查结构是否存在
        if 'controller_manager' in data and \
           'ros__parameters' in data['controller_manager']:
            
            controllers_params = data['controller_manager']['ros__parameters']
            
            # 提取所有键，并排除 'update_rate'
            controller_names = [
                key for key in controllers_params.keys()
                if key != 'update_rate'
            ]
            return controller_names
        else:
            print("YAML 结构不符合预期：缺少 'controller_manager' 或 'ros__parameters' 键。")
            return []
    except yaml.YAMLError as e:
        print(f"解析 YAML 时发生错误: {e}")
        return []

'''

这个文件启动有关于仿真器的部分，启动gazebo模拟器本身

'''
ld= LaunchDescription()

use_sim_time = LaunchConfiguration('use_sim_time', default=True)

ld.add_action(DeclareLaunchArgument("use_sim_time", default_value=use_sim_time))

package_name="exchange_station_sim"
robot_name="exchange_station"

namespace = robot_name

urdf_path = os.path.join(
    get_package_share_directory(package_name),
    "config",
    f"{robot_name}.urdf.xacro"
)

world_path = os.path.join(
    get_package_share_directory(package_name),
    "config",
    "empty_world.sdf"
)

ros2_controllers_path = os.path.join(
    get_package_share_directory(package_name),
    "config",
    "ros2_controllers.yaml"
)

with open(ros2_controllers_path, 'r') as file:
    ros2_controllers_content = file.read()

def generate_gazebo_launch_description():

    robot_description_content = xacro.process_file(urdf_path).toprettyxml(indent="  ") # type: ignore

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description,],
        # remappings=[
        #     ("robot_description", f"{robot_name}_robot_description"),
        #     ],
        namespace=namespace,
    )
    ld.add_action(node_robot_state_publisher)

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', f"robot_description",
                   '-name', robot_name, '-allow_renaming', 'true'],
        namespace=namespace,
    )
    ld.add_action(gz_spawn_entity)

    controller_names = extract_controller_names(ros2_controllers_content)

    print(f"controller_names {controller_names}")

    for controller_name in controller_names:
        spawner = Node(
            package='controller_manager',
            executable='spawner',
            name=controller_name,
            arguments=[controller_name,
                '--param-file',
                robot_controllers,
                "--controller-manager-timeout", "1200", "--switch-timeout", "1000"
                ],
            namespace=namespace,
        )
        ld.add_action(spawner)

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
