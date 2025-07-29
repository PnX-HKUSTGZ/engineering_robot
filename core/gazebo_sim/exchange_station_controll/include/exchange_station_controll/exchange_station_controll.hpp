#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <command_interfaces/msg/computer_state.hpp>
#include <command_interfaces/msg/player_command.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>
#include <random>
#include <thread>
#include <atomic>
#include <pthread.h>

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pluginlib/class_loader.hpp>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include <moveit_msgs/msg/position_constraint.h>
#include <moveit_msgs/msg/object_color.h>
#include <std_msgs/msg/color_rgba.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#ifndef EXCHANGE_STATION_CONTROLL_H_
#define EXCHANGE_STATION_CONTROLL_H_

namespace Engineering_robot_Pnx{

struct ExchangeStationStateLimit{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
    double theta_min;
    double theta_max;
    double phi_min;
    double phi_max;
    double alpha_min;
    double alpha_max;
};

struct ExchangeStationState{
    double x;
    double y;
    double z;
    double theta;
    double phi;
    double alpha;
};

/**
 * @brief 兑换站状态转换为PoseStamped类型
 * 
 * @param state 兑换站状态
 * @param goal_frame_id 目标坐标系
 * @return geometry_msgs::msg::PoseStamped 转换后的PoseStamped类型
 */
geometry_msgs::msg::PoseStamped to_pose_stamped(
    const ExchangeStationState& state,
    const std::string& goal_frame_id);
/**
 * @brief 将欧拉角转换为四元数
 * 
 * @param roll 横滚角
 * @param pitch 俯仰角
 * @param yaw 偏航角
 * @return geometry_msgs::msg::Quaternion 转换后的四元数
 */
geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw);

/**
 * @brief 生成随机的兑换站状态
 * 
 * @param limit 兑换站状态限制
 * @return ExchangeStationState 随机生成的兑换站状态
 */
ExchangeStationState generate_exchange_station(const ExchangeStationStateLimit & limit);

/**
 * @brief 生成随机的double类型数据
 * 
 * @param min 最小值
 * @param max 最大值
 * @return double 随机生成的double类型数据
 */
double random_double(double min, double max);

class Exchange_Station_Controll : public rclcpp::Node{
public:
    Exchange_Station_Controll(
        rclcpp::NodeOptions node_options=rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    /**
     * @brief 初始化moveit
     * 
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool MoveitInit();
private:

    // moveit
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_=nullptr;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_=nullptr;

    // config
    YAML::Node config;
    std::string ARM_CONTROL_GROUP;
    std::string BOTTOM_LINK;
    std::string NAMESPACE;
    std::string ROBOT_DISCRIPTION_PARAM;
    std::string END_EFFECTOR_LINK;
    std::vector<ExchangeStationStateLimit> state_limits_;

    // tf2
    tf2_ros::Buffer::SharedPtr tf2_buffer_=nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listenser_=nullptr;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_pub_=nullptr;

    // 用于操作兑换站状态
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr station_controll_sub_=nullptr;

    /**
     * @brief 兑换站状态回调函数
     * 
     * @param msg 兑换站状态 0,1,2,3,4 分别表示初始状态，一级状态，二级状态，三级状态，四级状态
     */
    void stationControllCallback(const std_msgs::msg::Int32::SharedPtr msg);

    /**
     * @brief 加载参数
     * 
     */
    void loadParam();

    /**
     * @brief 清除规划约束状态
     * 
     */
    void clearConstraintsState();


};

} // Engineering_robot_Pnx

#endif