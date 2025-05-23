# robot_model

这个仓库用于储存机器人模型以及moveit的有关配置

## engineering_robot

这个包储存工程机器人的urdf描述以及stl模型

## engineering_robot_hardware_interface

这个包是一个插件，实现了上位机和下位机的通讯

其派生于 `hardware_interface::SystemInterface`，是一个moveit的插件

这个节点默认的串口名称为 `/dev/ttyACM0`

### hardware_interface

这个插件会启动一个节点，监听以下topic：

+ `/player_command`
    类型为 `command_interfaces::msg::ComputerState`，监听上位机状态

发布以下topic：

+ `/computer_state`
    类型为 `command_interfaces::msg::PlayerCommand`，发布操作手命令

## engineering_robot_moveit

这个是模型的moveit和ros2 control配置文件