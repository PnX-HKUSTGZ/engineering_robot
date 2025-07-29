#include "exchange_station_controll/exchange_station_controll.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>
#include <chrono>


namespace Engineering_robot_Pnx{

Exchange_Station_Controll::Exchange_Station_Controll(
        rclcpp::NodeOptions node_options
    ):
    rclcpp::Node("exchange_station_controll",node_options){

        std::string package_name="exchange_station_controll";
        std::string config_file_path;
        try{
            std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
            RCLCPP_INFO(this->get_logger(), "Share directory for package '%s' is: %s", package_name.c_str(), package_share_directory.c_str());
            config_file_path=package_share_directory+"/config/config.yaml";
            config=YAML::LoadFile(config_file_path);
        }
        catch(const std::exception& e){
            RCLCPP_FATAL(this->get_logger(), "Failed to get share directory for package '%s': %s", package_name.c_str(), e.what());
            return;
        }
        loadParam();

        tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf2_listenser_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
        tf2_pub_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(this->get_logger(),"Load tf2 ok!");

        RCLCPP_INFO(this->get_logger(),"Exchange_Station_Controll Node created ok!");

    }

bool Exchange_Station_Controll::MoveitInit(){

    // movegroup action 接入设置
    moveit::planning_interface::MoveGroupInterface::Options opt(ARM_CONTROL_GROUP);

    opt.group_name_=ARM_CONTROL_GROUP;
    opt.robot_description_=ROBOT_DISCRIPTION_PARAM;
    opt.move_group_namespace_=NAMESPACE;

    try{
        move_group_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(),
            opt,
            tf2_buffer_,
            rclcpp::Duration::from_seconds(5)
        );
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to get move_group_: %s", e.what());
        return false;
    }

    try{

        planning_scene_interface_=std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        station_controll_sub_=this->create_subscription<std_msgs::msg::Int32>(
            "exchange_station_controll",
            10,
            std::bind(&Exchange_Station_Controll::stationControllCallback,this,std::placeholders::_1)
        );
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to get move_group_: %s", e.what());
        return false;
    }

    RCLCPP_INFO(this->get_logger(),"Moveit init ok!");

    return true;

}

void Exchange_Station_Controll::loadParam(){
    try{
        ARM_CONTROL_GROUP=config["ARM_CONTROL_GROUP"].as<std::string>();
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to get ARM_CONTROL_GROUP: %s", e.what());
        return;
    }

    try{
        BOTTOM_LINK=config["BOTTOM_LINK"].as<std::string>();
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to get BOTTOM_LINK: %s", e.what());
        return;
    }

    try{
        NAMESPACE=config["NAMESPACE"].as<std::string>();
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to get NAMESPACE: %s", e.what());
        return;
    }

    try{
        ROBOT_DISCRIPTION_PARAM=config["ROBOT_DISCRIPTION_PARAM"].as<std::string>();
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to get ROBOT_DISCRIPTION_PARAM: %s", e.what());
        return;
    }

    try{
        END_EFFECTOR_LINK=config["END_EFFECTOR_LINK"].as<std::string>();
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to get END_EFFECTOR_LINK: %s", e.what());
        return;
    }

    std::string level="level";
    for(int i=0;i<=4;i++){
        try{
            std::string this_level=level+std::to_string(i);
            ExchangeStationStateLimit this_limit;
            this_limit.x_min=config["ExchangeStationStateLimit"][this_level]["x_min"].as<double>();
            this_limit.x_max=config["ExchangeStationStateLimit"][this_level]["x_max"].as<double>();
            this_limit.y_min=config["ExchangeStationStateLimit"][this_level]["y_min"].as<double>();
            this_limit.y_max=config["ExchangeStationStateLimit"][this_level]["y_max"].as<double>();
            this_limit.z_min=config["ExchangeStationStateLimit"][this_level]["z_min"].as<double>();
            this_limit.z_max=config["ExchangeStationStateLimit"][this_level]["z_max"].as<double>();
            this_limit.theta_min=config["ExchangeStationStateLimit"][this_level]["theta_min"].as<double>();
            this_limit.theta_max=config["ExchangeStationStateLimit"][this_level]["theta_max"].as<double>();
            this_limit.phi_min=config["ExchangeStationStateLimit"][this_level]["phi_min"].as<double>();
            this_limit.phi_max=config["ExchangeStationStateLimit"][this_level]["phi_max"].as<double>();
            this_limit.alpha_min=config["ExchangeStationStateLimit"][this_level]["alpha_min"].as<double>();
            this_limit.alpha_max=config["ExchangeStationStateLimit"][this_level]["alpha_max"].as<double>();
            state_limits_.push_back(this_limit);
        }
        catch(const std::exception& e){
            RCLCPP_FATAL(this->get_logger(), "Failed to get level %d: %s", i, e.what());
            return;
        }
    }

    RCLCPP_INFO(this->get_logger(),"Load param ok!");

}

void Exchange_Station_Controll::stationControllCallback(const std_msgs::msg::Int32::SharedPtr msg){

    const int state = msg->data;
    if(state>=5){
        RCLCPP_ERROR(this->get_logger(),"Exchange station state is error!");
        return;
    }
    RCLCPP_INFO(this->get_logger(),"Exchange station state is %d",state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success=move_group_->setEndEffectorLink(END_EFFECTOR_LINK);
    if(!success){
        RCLCPP_ERROR(this->get_logger(),"setEndEffectorLink %s failed!",END_EFFECTOR_LINK.c_str());
        return;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"setEndEffectorLink %s success!",END_EFFECTOR_LINK.c_str());
    }

    while(1){
        ExchangeStationState goal_state=generate_exchange_station(state_limits_[state]);

        geometry_msgs::msg::PoseStamped pose_stamped=to_pose_stamped(goal_state,BOTTOM_LINK);
        move_group_->setPlanningTime(0.5);

        // 输出预期位置
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: x: %lf",state, pose_stamped.pose.position.x);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: y: %lf",state, pose_stamped.pose.position.y);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: z: %lf",state, pose_stamped.pose.position.z);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: theta: %lf",state, goal_state.theta);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: phi: %lf",state, goal_state.phi);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: alpha: %lf",state, goal_state.alpha);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: orientation.x: %f",state, pose_stamped.pose.orientation.x);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: orientation.y: %f",state, pose_stamped.pose.orientation.y);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: orientation.z: %f",state, pose_stamped.pose.orientation.z);
        RCLCPP_INFO(this->get_logger(),"Exchange station state %d: orientation.w: %f",state, pose_stamped.pose.orientation.w);

        // 结果

        move_group_->setPoseTarget(pose_stamped);

        moveit::core::MoveItErrorCode error=move_group_->plan(plan);
        if(error!=moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_ERROR(this->get_logger(),"Plan error! %s try again",moveit::core::error_code_to_string(error).c_str());
            continue;
        }

        try{
            move_group_->execute(plan);
        }
        catch(const std::exception & e){
            RCLCPP_ERROR(this->get_logger(),"move failed! with %s",e.what());
            return;
        }
        break;
    }

    RCLCPP_INFO(this->get_logger(),"Exchange station state switch to state %d successfully",state);

}


void Exchange_Station_Controll::clearConstraintsState(){
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    move_group_->clearTrajectoryConstraints();
}

//********************** 非类成员函数 **********************

ExchangeStationState generate_exchange_station(const ExchangeStationStateLimit & limit){
    ExchangeStationState state;
    state.x=random_double(limit.x_min,limit.x_max);
    state.y=random_double(limit.y_min,limit.y_max);
    state.z=random_double(limit.z_min,limit.z_max);
    state.theta=random_double(limit.theta_min,limit.theta_max);
    state.phi=random_double(limit.phi_min,limit.phi_max);
    state.alpha=random_double(limit.alpha_min,limit.alpha_max);
    return state;
}

double random_double(double min, double max) {
    unsigned seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    static std::mt19937 gen(seed);

    std::uniform_real_distribution<double> distrib(min, max);

    return distrib(gen);
}

geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw){
    geometry_msgs::msg::Quaternion quaternion;
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(roll, pitch, yaw);
    quaternion.x=tf_quaternion.x();
    quaternion.y=tf_quaternion.y();
    quaternion.z=tf_quaternion.z();
    quaternion.w=tf_quaternion.w();
    return quaternion;
}

geometry_msgs::msg::PoseStamped to_pose_stamped(
    const ExchangeStationState& state,
    const std::string& goal_frame_id) {
    
    // --- 1. 定义基础坐标系和目标坐标系 ---

    // 默认姿态的Z轴（通常是(0,0,1)）
    Eigen::Vector3d base_z_axis = Eigen::Vector3d::UnitZ();

    // 根据 state.theta 和 state.phi 计算目标姿态的Z轴 (即向量 e)
    Eigen::Vector3d target_z_axis;
    target_z_axis.x() = std::sin(state.phi) * std::cos(state.theta);
    target_z_axis.y() = std::sin(state.phi) * std::sin(state.theta);
    target_z_axis.z() = std::cos(state.phi);
    target_z_axis.normalize();

    // --- 2. 计算旋转 ---

    // 第一步：计算从 base_z_axis 旋转到 target_z_axis 的主方向旋转
    Eigen::Quaterniond direction_rotation;
    direction_rotation.setFromTwoVectors(base_z_axis, target_z_axis);

    // 第二步：计算绕着新的Z轴 (target_z_axis) 进行的自旋 (alpha)
    Eigen::AngleAxisd spin_rotation(state.alpha, target_z_axis);

    // 最终的旋转是主方向旋转和自旋的结合
    Eigen::Quaterniond final_rotation = direction_rotation * spin_rotation;

    // --- 3. 填充 geometry_msgs::msg::PoseStamped 消息 ---

    geometry_msgs::msg::PoseStamped pose_stamped_msg;

    // 填充 Header (注意: 在实际的ROS节点中，应该从节点获取时间)
    // pose_stamped_msg.header.stamp = node->get_clock()->now(); 
    pose_stamped_msg.header.frame_id = goal_frame_id;

    // 填充 Pose -> position
    pose_stamped_msg.pose.position.x = state.x;
    pose_stamped_msg.pose.position.y = state.y;
    pose_stamped_msg.pose.position.z = state.z;

    // 填充 Pose -> orientation
    pose_stamped_msg.pose.orientation.w = final_rotation.w();
    pose_stamped_msg.pose.orientation.x = final_rotation.x();
    pose_stamped_msg.pose.orientation.y = final_rotation.y();
    pose_stamped_msg.pose.orientation.z = final_rotation.z();

    return pose_stamped_msg;
}

}// Engineering_robot_Pnx