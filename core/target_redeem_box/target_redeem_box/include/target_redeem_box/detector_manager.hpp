#include <thread>
#include <algorithm>
#include <sstream>
#include <random>
#include <functional>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/int32.hpp"

#include <yaml-cpp/yaml.h>

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

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

#include "base_detector.hpp"

using namespace std::chrono;
using namespace std::placeholders;
using namespace std::chrono_literals;

#ifndef DETECTOR_MANAGER
#define DETECTOR_MANAGER

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;


namespace Engineering_robot_Pnx{

/**
 * @brief 目标检测管理器类
 * 
 */
class DetectorManager : public rclcpp::Node{
private:
    // 配置文件
    YAML::Node config;
    // 目标检测器列表
    std::vector<std::shared_ptr<BaseDetector>> detectors;
    // 目标检测线程列表
    std::vector<std::thread> detect_threads;

    // 输入数据，会随着时间更新，有锁 input_data_mutex
    InputData::SharedPtr input_data;
    // input_data 的锁
    std::mutex input_data_mutex;



    tf2_ros::Buffer::SharedPtr tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_subscriber_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> point_cloud_subscriber_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> posed_image_publisher_;

    void image_point_cloud_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& point_cloud_msg);

    void handle_detect_result(const DetectorOutput & output);

// 运行时参数
private:

    std::string image_topic;
    std::string image_frame;
    std::string point_cloud_topic;
    std::string point_cloud_frame;

    rclcpp::Duration input_data_time_out=rclcpp::Duration(0,0);


public:
    /**
     * @brief 初始化目标检测器
     * @return 是否全部初始化成功
     */
    bool init_detectors();

    /**
     * 启动目标检测循环循环
     */
    void start_detect();

    /**
     * @brief 构造函数
     */
    DetectorManager(rclcpp::NodeOptions options);

};


}; // Engineering_robot_Pnx

#endif