#include "sensor_msgs/msg/image.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

#include <thread>
#include <algorithm>
#include <sstream>
#include <random>

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

#ifndef __POSITION_CONTROLLER__
#define __POSITION_CONTROLLER__

namespace Engineering_robot_Pnx{

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node{

// Generall part

public:

    PositionController(rclcpp::NodeOptions options=rclcpp::NodeOptions());

private:
    //tf2_buffer_
    tf2_ros::Buffer::SharedPtr tf2_buffer_;

    // tf2_transform_listensr_ 
    std::shared_ptr<tf2_ros::TransformListener> tf2_transform_listensr_;

// RedeemBox_detector part
public:

    //may throw error
    geometry_msgs::msg::TransformStamped GetBoxPosition(rclcpp::Time time);

private:

    bool use_virtual_box_position;

    //RedeemBox_frame target fram of box
    std::string RedeemBox_frame;

    rclcpp::TimerBase::SharedPtr virtual_box_position_timer_;

    // future: 加入模型载入rviz

// robot_position part
public:

private:

    std::shared_ptr<tf2_ros::TransformBroadcaster> Robot_base_tf2_pub_;
    rclcpp::TimerBase::SharedPtr virtual_robot_position_timer_;

    std::string robot_base;
    std::string fixed_frame;

    bool use_virtual_robot_position;

    std::mutex mutex_;

    geometry_msgs::msg::TransformStamped redeembox;

};

}// namespace Engineering_robot_Pnx{

#endif