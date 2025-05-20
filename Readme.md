# Engineering_robot

这是一个工程机器人的整合安装库，用于一键安装和部署。

```
├── core
│   ├── ......
│   ├── ......
|   └── ......
├── material
├── launch
|   ├──......
│   ├──......
|   └──......
├── bringup.sh
├── project.repos
└── Readme.md
```

## 项目结构

+ core 文件夹包含了机器人的核心组件，如控制器、传感器、运动学等（完全配置后将被创建）
+ project.repos 用于vcs工具定义每个仓库的位置
+ bringup.sh 环境配置脚本，用于安装依赖
+ build.sh 编译脚本，用于编译所有的包
+ material 文件夹包含了一些材料，开源资料，规则手册等
+ launch 文件夹包含了启动文件


## 安装

> 注意：
> 移除任何意义上的虚拟环境，如conda，venv等。
> e.g. 运行 `conda deactivate`

1. 克隆这个仓库
    ```bash

    git clone git@github.com:PnX-HKUSTGZ/engineering_robot.git

    # 或者
    # git clone https://github.com/PnX-HKUSTGZ/engineering_robot.git

    ```

2. 安装VCS工具
    ```bash
    sudo apt-get update
    sudo apt-get install python3-vcstool
    ```

3. 导入仓库

    ```bash
    vcs import src < project.repos
    ```

4. 安装依赖
    ```bash
    sudo chmod +x bringup.sh
    sudo ./bringup.sh
    ```
    完成后重启终端

5. 编译
    ```bash
    sudo chmod +x build.sh
    ./build.sh
    ```

6. 启动
    ```bash
    ros2 launch <launch file>
    ```