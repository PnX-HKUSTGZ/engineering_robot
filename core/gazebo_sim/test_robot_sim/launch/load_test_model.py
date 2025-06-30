from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('test_robot_sim'),
                 'config', 'test_robot.urdf.xacro']
            ),
        ]
    )

    # robot_description_content=ParameterValue(
    #     PythonExpression([
    #         "str(",
    #         robot_description_content,
    #         ").replace('package://",
    #         "test_robot", 
    #         "', str('",            
    #         FindPackageShare('test_robot'),        
    #         "'))"                            
    #     ]),
    #     value_type=str
    # )

    joint_state_frame_prefix={"frame_prefix":"test_robot/"}
    namespace="test_robot/"

    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('test_robot_sim'),
            'config',
            'ros2_controllers.yaml',
        ]
    )

    # 
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        namespace=namespace,
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', "robot_description",
                   '-name', 'test_robot', '-allow_renaming', 'true'],
        namespace=namespace,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[{"use_local_topics": True}],
        arguments=['joint_state_broadcaster',
            '--param-file',
            robot_controllers,
            "--controller-manager-timeout", "1200", "--switch-timeout", "1000"
            ],
        namespace=namespace,
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable="spawner",
        name='arm_controller',
        arguments=[
            'arm_controller',
            '--param-file',
            robot_controllers,
            ],
        namespace=namespace,
    )

    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable="spawner",
        name='chasis_controller',
        arguments=[
            'chasis_controller',
            '--param-file',
            robot_controllers,
            ],
        namespace=namespace,
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        mecanum_drive_controller_spawner,
    ])
