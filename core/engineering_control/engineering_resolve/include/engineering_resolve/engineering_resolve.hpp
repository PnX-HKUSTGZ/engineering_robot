#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <command_interfaces/msg/computer_state.hpp>
#include <command_interfaces/msg/player_command.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>


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

#ifndef MOTION_PLANNING_API_NODE_HPP
#define MOTION_PLANNING_API_NODE_HPP

#define VISUALIZE

#define STATE_ONE 1
#define STATE_TWO 2
#define STATE_THREE 3
#define STATE_ERROR 4
#define STATE_WAIT 0

#define FINISH 2
#define PLANNING 0
#define MOVING 1

#define REC_SUCCESS 1
#define REC_FAIL 0

#define PIPE_THREAD_RUNNING 0
#define PIPE_THREAD_ERROR 1
#define PIPE_THREAD_OK 2
#define PIPE_THREAD_NOLAUNCH 3


namespace Engineering_robot_Pnx {
namespace rvt = rviz_visual_tools;
using namespace std::placeholders;
using namespace std::chrono_literals;

enum class MultithreadState {
    PLANE_THREAD_NOLAUNCH = 0,
    PLANE_THREAD_RUNNING = 1,
    PLANE_THREAD_OK = 2,
    PLANE_THREAD_ERROR = 3
};

struct PlayerCommandContent{
    rclcpp::Time command_time=rclcpp::Time(0,0);
    bool is_started=0;
    bool is_attach=0;
    bool is_finish=0;
    bool breakout=0;
};

struct ComputerState{
    uint8_t current_state;
    uint8_t recognition:2;
    uint8_t pos1_state:2;
    uint8_t pos2_state:2;
    uint8_t pos3_state:2;
};

class Resolving_Node: public rclcpp::Node{

public:

// init the node, after this you have to run MoveitInit to init moveit!
Resolving_Node(
    std::string name,
    const std::string & ARM_CONTROL_GROUP,
    const std::string & END_EFFECTOR_CONTROL_GROUP,
    const std::string & end_link,
    rclcpp::NodeOptions node_options=rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

bool MoveitInit();

void clear_constraints_state();
bool setPoseTarget(const geometry_msgs::msg::Pose & targetpose);
void setGoalOrientationTolerance(double tolerance);
void setGoalPositionTolerance(double tolerance);
void setMaxVelocityScalingFactor(double factor);
void setMaxAccelerationScalingFactor(double factor);
void setPlanningTime(double time);
void setReplanAttempts(int times);
bool setEndEffectorLink(const std::string & end_link);
moveit::core::MoveItErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan &plan);

private:

std::string ARM_CONTROL_GROUP;
std::string END_EFFECTOR_CONTROL_GROUP;
std::string end_link;

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

};

class Engineering_robot_Controller: public rclcpp::Node{

public:

Engineering_robot_Controller(rclcpp::NodeOptions node_options=rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

// @return init successfully
bool MoveitInit();
void LoadParam();

private:

std::string ARM_CONTROL_GROUP;
std::string END_EFFECTOR_CONTROL_GROUP;

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
// std::vector<planning_scene_monitor::PlanningSceneMonitorPtr> planning_scene_monitors_;
// std::vector<std::shared_ptr<rclcpp::Node>> planning_scene_monitor_nodes_;
// std::vector<robot_model_loader::RobotModelLoaderPtr> robot_model_loaders_;
std::vector<std::shared_ptr<Resolving_Node> > resolving_nodes_;
const moveit::core::JointModelGroup* arm_model_group;
std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_;

// rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planner_trigger_;

tf2_ros::Buffer::SharedPtr tf2_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf2_listenser_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_pub_;

// state_controll

// load RedeemBox and mine
bool LoadAttachMine();
bool LoadRedeemBox();
bool LoadRedeemBox(geometry_msgs::msg::TransformStamped msg);
bool RemoveRedeemBox();

/**
 * @brief 控制 MoveIt 规划场景中两个指定物体之间是否允许碰撞检测。
 *
 * @param name1 第一个物体的名称（或机器人连杆名称）。
 * @param name2 第二个物体的名称（或机器人连杆名称）。
 * @param enable_collision 如果为 true，则允许碰撞（即恢复碰撞检测）；如果为 false，则禁用碰撞（即忽略碰撞检测）。
 * @return bool 如果成功应用了规划场景更新，则返回 true，否则返回 false。
 */
bool setCollisionsBetween(const std::string& name1, const std::string& name2, bool enable_collision);

bool RemoveObject(const std::string & name);
bool disableObjectRobotCollision(const std::string& object_id, const std::vector<std::string>& robot_link_names);
bool disableObjectRobotCollision(const std::string& object_id, const std::string robot_link_name);
bool IsObjectInScene(const std::string& object_id);
void clearPlanScene();

std::string MineMesh="package://engineering_resolve/meshes/Mine.STL";
std::string RedeemBoxMesh="package://engineering_resolve/meshes/RedeemBox.STL";
std::string RedeemBoxFram="object/fixedbox";
std::string robot_base="robot_base_link";
std::string end_link="end_link";
const std::string package_name="engineering_resolve";

double minOrientationTolerance=0.1;
double minPositionTolerance=0.01;
double maxOrientationTolerance=0.5;
double maxPositionTolerance=0.05;
double minPlanTime=10;
double maxPlanTime=15;
int AllowPlanAttempt=5;
int MultithreadNum=5;
int NumPlanningAttempts=3;

std::string planner;

double OrientationToleranceStep=0;
double PositionToleranceStep=0;
double PlanTimeStep=0;

// exchange_state_controller

PlayerCommandContent player_command;
std::mutex player_command_mutex;

ComputerState computer_state;
std::mutex computer_state_mutex;

std::shared_ptr<rclcpp::Subscription<command_interfaces::msg::PlayerCommand> > player_command_sub_;
std::shared_ptr<rclcpp::Publisher<command_interfaces::msg::ComputerState> > computer_state_pub_;
// 以30hz的频率发布上位机状态
rclcpp::TimerBase::SharedPtr computer_state_pub_timer_;
std::shared_ptr<std::thread> commmand_executor_thread_;
std::shared_ptr<std::thread> regonition_update_thread_;
rclcpp::Duration player_commmand_time_threshold=rclcpp::Duration(0,1e8);

// 0 no ok
// 1 ok
std::atomic<int> mine_exchange_pipe_state;

std::shared_ptr<rclcpp::TimerBase> RedeemBox_pos_pub_timer=nullptr;

void player_command_sub_callback(const command_interfaces::msg::PlayerCommand::ConstSharedPtr & msg);
PlayerCommandContent get_player_command();
ComputerState get_computer_state();
void set_player_command(const PlayerCommandContent & input_command);
void set_computer_state(const ComputerState & input_state);
void set_computer_state(int statenum,int state);
void set_regonition_state(int statenum,int state);

void computer_state_pub_callback();
void regonition_update();

void commmand_executor();
void mine_exchange_pipe();
void clear_constraints_state();

//debug

std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> robot_get_min_sub_;
std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> robot_go_home_sub_;
std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> robot_auto_exchange_sub_;
std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> robot_clear_scense_sub_;
std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> demon_run_sub_;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> plan_time_pub_;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> success_rate_pub_;

int try_times=0;
int success_times=0;

bool robot_go_pose(const std::string & name);
bool AutoExchangeMine();

geometry_msgs::msg::TransformStamped fix_RedeemBox_pos();

void unfix_RedeemBox_pos();

/*
    @brief 多线程规划
    @param req 规划请求
    @param res 规划响应
    @param threadnum 线程数
    @return Success 1 or Fail 0
*/
bool MultithreadedPlanne(
    const planning_interface::MotionPlanRequest& req, 
    planning_interface::MotionPlanResponse & res,
    int threadnum);

bool MultithreadedPlanne(
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    int threadnum);

bool MultithreadedPlanne(
    moveit::planning_interface::MoveGroupInterface::Plan& plan);

bool DemonRun();

void Multiclear_constraints_state();
bool MultisetPoseTarget(const geometry_msgs::msg::Pose & targetpose);
void MultisetGoalOrientationTolerance(double tolerance);
void MultisetGoalPositionTolerance(double tolerance);
void MultisetMaxVelocityScalingFactor(double factor);
void MultisetMaxAccelerationScalingFactor(double factor);
void MultisetPlanningTime(double time);
void MultisetReplanAttempts(int times);
bool MultisetEndEffectorLink(const std::string & end_link);

};// Engineering_robot_Controller


std::vector<double> eulerToQuaternion(double roll, double pitch, double yaw);
std::vector<double> eulerToQuaternion(const std::vector<double>& euler);
std::vector<double> quaternionToEuler(const std::vector<double>& q);

void doPointTransform(
    const geometry_msgs::msg::Point &data_in,
    geometry_msgs::msg::Point &data_out,
    const geometry_msgs::msg::TransformStamped &transform
);

/**
 * @brief Transforms a geometry_msgs::msg::Pose by a geometry_msgs::msg::TransformStamped.
 * Applies the translational and rotational components of the transform to the pose's
 * position and orientation respectively.
 *
 * @param data_in The input pose in the source frame.
 * @param data_out The transformed pose in the target frame.
 * @param transform The transform from the source frame to the target frame.
 */
void doPoseTransform(
    const geometry_msgs::msg::Pose &data_in,
    geometry_msgs::msg::Pose &data_out,
    const geometry_msgs::msg::TransformStamped &transform);

} // namespace motion_planning_api

#endif // Engineering_robot_Pnx
