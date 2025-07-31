#include <thread>
#include <algorithm>
#include <sstream>
#include <random>
#include <functional>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

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
#include <pcl/registration/icp_nl.h> 
#include <pcl/features/normal_3d_omp.h> 

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono;
using namespace std::placeholders;
using namespace std::chrono_literals;

#ifndef BASE_DETECTOR
#define BASE_DETECTOR

namespace Engineering_robot_Pnx{

/*
输入数据的结构体，用于存储输入的图像和点云数据
在使用这个结构的时候请保证point_cloud_的坐标系和image的坐标系一致
*/
struct InputData{
    rclcpp::Time update_time;
    const std::shared_ptr<const cv::Mat> image;
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud_=nullptr;
    
    /**
     * @brief 构造函数 复制对应变量
     */
    InputData(const rclcpp::Time& time_, 
        const std::shared_ptr<const cv::Mat>& image_, 
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc_);
    
    using SharedPtr = std::shared_ptr<InputData>;

};


/**
 * @brief 目标检测输出数据的结构体
 * @param tvec 目标在相机坐标系下的位置
 * @param rvec 目标在相机坐标系下的旋转
 * @param result_image_ 目标检测结果的图像
 * 
 * 都是基于image系的转变
 */
struct DetectorOutput{
    cv::Mat tvec;
    cv::Mat rvec;
    std::shared_ptr<cv::Mat> result_image_=nullptr;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud_=nullptr;
};

class BaseDetector{

protected:

// 配置文件
YAML::Node config;
// 调用Detector的节点
rclcpp::Node::SharedPtr node_;
// 名字
const std::string name;

public:

BaseDetector() = delete;

/**
 * @brief 析构函数
 */
virtual ~BaseDetector();

/**
 * @brief 构造函数
 * @param config 配置文件
 * @param node 随便给个node，记得是要已经初始化完成的
 * @param name 检测器的名字
 */
BaseDetector(const YAML::Node& config, rclcpp::Node::SharedPtr node, const std::string & name);

/**
 * @brief 构造函数 这个函数会根据 name 来自动生成 node
 * @param config 配置文件
 * @param name 检测器的名字
 */
BaseDetector(const YAML::Node& config, const std::string & name);

/**
 * @brief 目标检测函数
 * @param input_data 输入数据
 * @param output_data 输出数据
 * @return 是否检测到对象
 * 
 * 注意错误处理
 */
virtual bool detect(InputData input_data,
    DetectorOutput& output_data) = 0;

virtual std::string const getDetectorName();

using SharedPtr = std::shared_ptr<BaseDetector>;

};

} // Engineering_robot_Pnx

#endif