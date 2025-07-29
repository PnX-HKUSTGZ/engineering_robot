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
import yaml

use_sim_time = {"use_sim_time": True}


def extract_controller_names(yaml_str,
    namespace = ""):
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

        if namespace != "":
            data = data[f"/{namespace}"]
        
        print(data)

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


def load_robot(
    ld: LaunchDescription,
    package_name: str,
    robot_name: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
    namespace: str = "",
    ):

    ros2_controllers_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "ros2_controllers.yaml"
    )

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        f"{robot_name}.urdf.xacro"
    )

    with open(ros2_controllers_path, 'r') as file:
        ros2_controllers_content = file.read()

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
        remappings=[
            ("robot_description", f"{robot_name}_robot_description"),
            ],
        namespace=namespace,
    )
    ld.add_action(node_robot_state_publisher)

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', f"{robot_name}_robot_description",
                   '-name', robot_name, '-allow_renaming', 'true'
                   '-x',str(x),
                   '-y',str(y),
                   '-z',str(z),
                   '-R',str(roll),
                   '-P',str(pitch),
                   '-Y',str(yaw),
                   ],
        namespace=namespace,
    )
    ld.add_action(gz_spawn_entity)

    controller_names = extract_controller_names(ros2_controllers_content,namespace)

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

def load_moveit(
        ld: LaunchDescription,
        package_name: str,
        robot_name: str,
        namespace: str = "",
        rviz_config_path: str = "",
):
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        f"{robot_name}.urdf.xacro"
    )

    srdf_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        f"{robot_name}.srdf"
    )

    kinematics_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "kinematics.yaml",
    )

    joint_limits_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "joint_limits.yaml",
    )

    moveit_config = (MoveItConfigsBuilder(robot_name, package_name=package_name)
                     .robot_description(urdf_path)
                     .robot_description_kinematics(kinematics_path)
                     .robot_description_semantic(srdf_path)
                     .joint_limits(joint_limits_path)
                     .to_moveit_configs())

    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )

    name_counter = 0
    virtual_joint_node : Node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
    )
    ## 写来防止报错的
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            virtual_joint_node= Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"static_transform_publisher{name_counter}",
                output="log",
                arguments=[
                    "--frame-id",
                    vj.parent_frame,
                    "--child-frame-id",
                    vj.child_link,
                ],
                parameters=[use_sim_time],
                namespace=namespace,
            )
            name_counter += 1

    # move_group
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        use_sim_time,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        namespace=namespace,
    )

    if namespace != "":
        rviz_parameters = [
            {f"{robot_name}_planning_pipelines": moveit_config.planning_pipelines["planning_pipelines"]},
            {f"{robot_name}_robot_description_kinematics": moveit_config.robot_description_kinematics["robot_description_kinematics"]},
            {f"{robot_name}_robot_description_semantic": moveit_config.robot_description_semantic["robot_description_semantic"]},
            {f"{robot_name}_robot_description": moveit_config.robot_description["robot_description"]},
            {f"{robot_name}_joint_limits": moveit_config.joint_limits},
            use_sim_time,
        ]
    else:
        rviz_parameters = [
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description,
            moveit_config.joint_limits,
            use_sim_time,
        ]


    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            # namespace=namespace,
            output="screen",
            arguments=["-d", rviz_config_path],
            parameters=rviz_parameters,
        )
    
    ld.add_action(rviz_node)

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=rviz_node,
                on_start=[move_group_node],
            )
        ),
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=move_group_node,
                on_start=[virtual_joint_node],
            )
        ),
    )


def generate_launch_description():

    world_path = os.path.join(
        get_package_share_directory("test_whole_sim"),
        "config",
        "empty_world.sdf"
    )

    ld = LaunchDescription()

    load_robot(
        ld,
        "exchange_station_sim",
        "exchange_station",
        0,0,0,0,0,0,
        "exchange_station",
    )

    load_robot(
        ld,
        "test_robot_sim",
        "test_robot",
        1,3,0
    )

    load_moveit(
        ld,
        "exchange_station_sim",
        "exchange_station",
        namespace="exchange_station",
        rviz_config_path=os.path.join(
            get_package_share_directory("exchange_station_sim"),
            "config",
            "moveit_gazebo_rviz.rviz"
        ),
    )

    load_moveit(
        ld,
        "test_robot_sim",
        "test_robot",
        namespace="",
        rviz_config_path=os.path.join(
            get_package_share_directory("test_robot_sim"),
            "config",
            "moveit_gazebo_rviz.rviz"
        ),
    )

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

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/test_robot/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        output='screen'
    )

    ld.add_action(imu_bridge)

    image_bridge=Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/test_robot/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        output='screen'
    )

    ld.add_action(image_bridge)

    pc2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/test_robot/image/image@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen'
    )

    ld.add_action(pc2_bridge)

    return ld