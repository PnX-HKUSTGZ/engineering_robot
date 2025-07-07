#!/usr/bin/env bash

# ---
# A script to set up a development environment for robotmaster pnx engineering robot.
# @parm -y Assume yes for all prompts.
# @author   Your Name
# @version  1.0.0
# ---

set -e
set -u
set -o pipefail

assume_yes=false
declare -a SUDO_CMD

readonly COLOR_RESET='\033[0m'
readonly COLOR_RED='\033[0;31m'
readonly COLOR_GREEN='\033[0;32m'
readonly COLOR_YELLOW='\033[0;33m'

# --- 工具函数 (Utility Functions) ---
info() {
    echo -e "${COLOR_GREEN}[INFO] ${1}${COLOR_RESET}"
}

warn() {
    echo -e "${COLOR_YELLOW}[WARN] ${1}${COLOR_RESET}"
}

error() {
    echo -e "${COLOR_RED}[ERROR] ${1}${COLOR_RESET}" >&2
    exit 1
}

usage() {
    echo "Usage: $0 [-y]" >&2
    exit 1
}

apt_install() {
    local package="$1"
    if "${SUDO_CMD[@]}" apt-get install -y "$package"; then
        info "Successfully installed $package"
    else
        error "Failed to install $package. Please check your network connection or apt sources."
    fi
}

confirm() {

    if [ "$assume_yes" = true ]; then
        return 0 # 返回成功
    fi

    # ${1:-"Are you sure?"} 的意思是：如果第1个参数存在，就用它；否则，使用默认值 "Are you sure?"
    local prompt_message="${1:-Are you sure?}"
    
    read -p "${prompt_message} [y/N] " -n 1 -r
    echo


    if [[ $REPLY =~ ^[Yy]$ ]]; then
        return 0 # 返回成功
    else
        return 1 # 返回失败
    fi
}

apt_source_setup(){

    confirm "Do you want to set up the apt source list? This will back up your current sources.list file." || {
        info "Skipping apt source setup."
        return 0
    }

    if "${SUDO_CMD[@]}" mv /etc/apt/sources.list /etc/apt/sources.list.bak; then
        info "Backed up /etc/apt/sources.list to /etc/apt/sources.list.bak"
    else {
        error "Failed to back up sources.list. Please check your permissions or if the file exists."
    }
    fi

    if "${SUDO_CMD[@]}" cp ./tsinghua.source /etc/apt/sources.list; then
        info "Copied new sources.list from ./tsinghua.source to /etc/apt/sources.list"
    else {
        error "Failed to copy new sources.list. Please check the file path. Ensure that the file exists and you have the necessary permissions."
    }
    fi

    "${SUDO_CMD[@]}" apt-get update || {
        error "Failed to update apt sources. Please check your network connection or sources.list file."
    }

    info "Apt source setup completed successfully."

}

ros2_install(){
    # 设置 locale 保证支持 UTF-8
    locale
    "${SUDO_CMD[@]}" apt-get update && "${SUDO_CMD[@]}" apt-get install -y locales
    "${SUDO_CMD[@]}" locale-gen en_US en_US.UTF-8
    "${SUDO_CMD[@]}" update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale

    # 设置时区

    TIMEZONE="Asia/Shanghai"
    "${SUDO_CMD[@]}" apt-get install -y tzdata
    "${SUDO_CMD[@]}" ln -snf "/usr/share/zoneinfo/$TIMEZONE" /etc/localtime
    "${SUDO_CMD[@]}" echo "$TIMEZONE" > /etc/timezone

    # 导入ros2 apt 密钥
    "${SUDO_CMD[@]}" apt-get update && "${SUDO_CMD[@]}" apt-get install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') || {
        error "Failed to get the latest ros2 apt source version. Please check your network connection or the GitHub API."
    }
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" || {
        error "Failed to download ros2 apt source. Please check your network connection or the URL."
    }
    "${SUDO_CMD[@]}" dpkg -i /tmp/ros2-apt-source.deb || {
        error "Failed to install ros2 apt source. Please check your network connection or the downloaded file."
    }
    "${SUDO_CMD[@]}" apt-get update || {
        error "Failed to update apt sources after installing ros2 apt source. Please check your network connection or sources.list file."
    }

    # 执行升级系统的操作
    if confirm "Do you want to upgrade your system packages? This may take a while. If no, the system may broken after install ros2" ; then
        "${SUDO_CMD[@]}" apt-get upgrade -y
    else
        info "Skipping system upgrade."
    fi

    "${SUDO_CMD[@]}" apt-get install -y ros-humble-desktop || {
        error "Failed to install ROS 2 Humble desktop. Please check your network connection or apt sources."
    }

    if confirm "do you want to add source /opt/ros/humble/setup.bash to your ~/.bashrc?"; then
        if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
            echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
            info "Added source /opt/ros/humble/setup.bash to ~/.bashrc"
        else
            warn "source /opt/ros/humble/setup.bash already exists in ~/.bashrc"
        fi
    else
        info "Skipping adding source to ~/.bashrc."
    fi

    # 安装 ros2 额外工具

    apt_install ros-humble-asio-cmake-module
    apt_install ros-humble-serial-driver
    apt_install ros-humble-rosbridge-server

    info "ROS 2 Humble installation completed successfully."

}

base_tools_install() {
    "${SUDO_CMD[@]}" apt-get update || {
        error "Failed to update apt sources. Please check your network connection or sources.list file."
    }
    "${SUDO_CMD[@]}" apt-get install -y \
        nano \
        build-essential \
        git \
        ssh \
        apt-utils \
        wget \
        curl \
        vim \
        tmux \
        htop \
        unzip \
        cmake \
        pkg-config \
        net-tools \
        python3-pip \
        python3-venv || 
    {
        error "Failed to install base tools. Please check your network connection or apt sources."
    }
}

yaml_install(){
    # 从源码安装yaml-cpp (Install yaml-cpp)
    yaml_name="yaml-cpp-0.8.0"
    tar_file="0.8.0.tar.gz"

    wget -O /tmp/${tar_file} https://github.com/jbeder/yaml-cpp/archive/refs/tags/${tar_file} || {
        error "Failed to download yaml-cpp 0.8.0. Please check your network connection or the URL."
    }
    "${SUDO_CMD[@]}" tar -xzf /tmp/${tar_file} -C /tmp/${yaml_name} --strip-components=1 || {
        error "Failed to extract yaml-cpp 0.8.0. Please check the downloaded file."
    }

    cd /tmp/${yaml_name} || {
        error "Failed to change directory to /tmp/${yaml_name}. Please check if the directory exists."
    }

    mkdir build
    cd build || {
        error "Failed to change directory to /tmp/${yaml_name}/build. Please check if the directory exists."
    }
    cmake -DYAML_BUILD_SHARED_LIBS=on ..
    make -j5
    "${SUDO_CMD[@]}" make install || {
        error "Failed to install yaml-cpp 0.8.0. Please check your network connection or the build process."
    }
    info "yaml-cpp 0.8.0 installed successfully."
}

mvs_install() {
    # 安装 海康 MVS SDK (Install Hikvision MVS SDK)
    mkdir -p /tmp/mvs_sdk
    cd /tmp/mvs_sdk || {
        error "Failed to change directory to /tmp/mvs_sdk. Please check if the directory exists."
    }
    wget /tmp/mvs_sdk/MVS_STD_V3.0.1_241128.zip https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_V3.0.1_241128.zip || {
        error "Failed to download MVS SDK. Please check your network connection or the URL."
    }
    unzip /tmp/mvs_sdk/MVS_STD_V3.0.1_241128.zip
    "${SUDO_CMD[@]}" dpkg -i MVS-3.0.1_x86_64_20241128.deb || {
        error "Failed to install MVS SDK MVS-3.0.1_x86_64_20241128.deb"
    }
    info "MVS SDK installed successfully."
}


libary_install() {
    # 安装常用库 (Install Common Libraries)
    info "Installing common libraries..."
    apt_install "libusb-1.0-0-dev"
    apt_install "libpcl-dev"
    apt_install "libopencv-dev"
    apt_install "python3-opencv"
    apt_install "libopencv-contrib-dev"

    yaml_install
    mvs_install

}

gazebo_install(){
    # install gazebo
    info "Installing ignition Fortress..."
    "${SUDO_CMD[@]}" apt-get update
    "${SUDO_CMD[@]}" apt-get install -y lsb-release gnupg
    "${SUDO_CMD[@]}" curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg || {
        error "Failed to download OSRF GPG key. Please check your network connection or the URL."
    }
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    "${SUDO_CMD[@]}" apt-get update
    "${SUDO_CMD[@]}" apt-get install -y ignition-fortress

    info "install ros-gz packages..."

    "${SUDO_CMD[@]}" apt-get install ros-humble-ros-gz

    info "Gazebo installation completed successfully."
}

# --- 主脚本 (Main Script) ---

## 处理命令行参数 (Handle Command Line Arguments)
## 处理 -y

if [ "$#" -gt 1 ]; then
    error "Usage: $0 [-y]"
fi

if [ "$#" -eq 1 ]; then
    if [ "$1" == "-y" ]; then
        # 参数正确，设置标志为 true
        export DEBIAN_FRONTEND=noninteractive
        assume_yes=true
    else
        # 参数不是 "-y"，判定为非法参数
        usage
    fi
fi

## 检查sudo
# 检查当前用户是否为 root
if [ "$(id -u)" -ne 0 ]; then
    if command -v sudo &> /dev/null; then
        SUDO_CMD=(sudo) # 赋值为一个包含 "sudo" 的数组
    else
        error "This script requires root privileges. Please run as root or install sudo."
    fi
fi

## 检查是否在 Ubuntu 22.04 系统上运行
info "Checking if the system is Ubuntu 22.04..."
if ! grep -q "Ubuntu 22.04" /etc/os-release; then
    error "This script is designed to run on Ubuntu 22.04. Please check your system version."
fi

info "Starting robotmaster pnx engineering robot setup..."

info "apt source setup..."

apt_source_setup

info "Install Base Tools ..."

base_tools_install || {
    error "Failed to install base tools. Please check your network connection or apt sources."
}

info "Installing ros2 packages..."

ros2_install || {
    error "Failed to install ros2 packages. Please check your network connection or apt sources."
}

libary_install || {
    error "Failed to install libraries. Please check your network connection or apt sources."
}

gazebo_install || {
    error "Failed to install gazebo. Please check your network connection or apt sources."
}

info "Robotmaster pnx engineering robot setup completed successfully."