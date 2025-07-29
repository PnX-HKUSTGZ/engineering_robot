#!/bin/bash

# --- 脚本配置 ---
# 如果你使用的不是 gnome-terminal，请修改为你自己的终端模拟器
# 例如: konsole -e, xterm -e 等
TERMINAL_CMD="gnome-terminal --"

# --- 脚本开始 ---

# 1. 检查 ROS 2 环境是否已配置
if [ -z "$ROS_DISTRO" ]; then
    echo "错误：ROS 2 环境未配置。"
    echo "请先执行 'source /opt/ros/ humble/setup.bash' (请替换为你的ROS版本) 后再运行此脚本。"
    exit 1
fi

# 2. 提示用户输入话题名称
read -p "请输入要发布消息的话题名称 (例如 /my_topic): " topic_name

# 检查用户是否输入了话题名称
if [ -z "$topic_name" ]; then
    echo "错误：话题名称不能为空。"
    exit 1
fi

echo "将在话题 '$topic_name' 上发布 std_msgs/msg/Int32 类型的消息。"
echo "---------------------------------------------------------"

# 3. 在新终端中启动订阅者以显示输出
echo "正在新的终端窗口中启动话题订阅者..."
$TERMINAL_CMD bash -c "echo '正在监听话题: $topic_name'; echo '按 Ctrl+C 关闭此窗口'; ros2 topic echo $topic_name std_msgs/msg/Int32; exec bash" &
# exec bash 是为了在 ros2 命令结束后保持终端窗口打开

# 等待一小会儿，确保订阅者启动
sleep 2

# 4. 主循环，用于接收用户输入并发布消息
echo "现在你可以输入一个整数并按回车键来发布消息。"
echo "输入 'q' 或 'quit' 来退出脚本。"
echo "---------------------------------------------------------"

while true; do
    # 提示用户输入
    read -p "请输入要发布的整数 (或输入 'q' 退出): " user_input

    # 检查用户是否想退出
    if [[ "$user_input" == "q" ]] || [[ "$user_input" == "quit" ]]; then
        echo "正在退出脚本..."
        break
    fi

    # 检查输入是否为整数
    if ! [[ "$user_input" =~ ^-?[0-9]+$ ]]; then
        echo "无效输入: '$user_input' 不是一个有效的整数。请重试。"
        continue
    fi

    # 5. 构建并执行 ros2 topic pub 命令
    # 我们使用 --once 选项，这样命令在发布一次后就会立即退出，不会阻塞脚本
    echo "正在发布: $user_input 到话题 $topic_name"
    ros2 topic pub --once "$topic_name" std_msgs/msg/Int32 "data: $user_input"

    # 检查上一条命令是否成功执行
    if [ $? -ne 0 ]; then
        echo "错误：发布消息失败。请检查话题名称和类型是否正确。"
    fi
done

# 清理：关闭由脚本启动的订阅者终端窗口
# pkill 会杀死所有包含特定命令行的进程，这里需要谨慎使用
# 如果 gnome-terminal 进程的标题或命令行是唯一的，可以这样做
# 一个更安全的方法是记录进程ID (PID)，但对于这个简单脚本，提示用户手动关闭即可。
echo "脚本已结束。请手动关闭订阅者终端窗口。"

exit 0