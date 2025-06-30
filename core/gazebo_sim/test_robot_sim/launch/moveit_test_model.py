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
use_sim_time = {"use_sim_time": True}

ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="true"))

test_robot_urdf_path = os.path.join(
    get_package_share_directory("test_robot_sim"),
    "config",
    "test_robot.urdf.xacro"
)

def generate_moveit_launch_description():

    moveit_config = (MoveItConfigsBuilder("test_robot", package_name="test_robot_sim")
                     .to_moveit_configs())
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # 发布虚拟关节的tf
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
                # namespace=namespace,
                # remappings=[
                #     ("moveit_robot_description", "robot_description"),
                #     ("robot_description","moveit_robot_description"),
                #     ],
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
        # namespace=namespace,
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        # remappings=[
        #     ("moveit_robot_description", "robot_description"),
        #     ("robot_description","moveit_robot_description"),
        #     ],
    )

    # rviz
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(os.path.join(get_package_share_directory("test_robot_sim"), "config", "moveit_gazebo_rviz.rviz")), # type: ignore
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        use_sim_time,
    ]

    rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            # namespace=namespace,
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=rviz_parameters,
            # remappings=[
            #     ("moveit_robot_description", "robot_description"),
            #     ("robot_description","moveit_robot_description"),
            #     ],
        )

    robot_description_content = xacro.process_file(test_robot_urdf_path).toprettyxml(indent="  ") # type: ignore

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

    # ld.add_action(
    #     RegisterEventHandler(
    #         event_handler=OnProcessStart(
    #             target_action=rsp_node,
    #             on_start=[virtual_joint_node],
    #         )
    #     ),
    # )

def generate_launch_description():

    generate_moveit_launch_description()

    return ld
