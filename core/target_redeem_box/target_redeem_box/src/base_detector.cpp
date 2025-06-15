#include "target_redeem_box/base_detector.hpp"

namespace Engineering_robot_Pnx{

InputData::InputData(const rclcpp::Time& time_, 
    const cv::Mat& image_, 
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc_):
    update_time(time_),
    image(image_),
    point_cloud_(pc_)
{}

std::string const BaseDetector::getDetectorName(){
    return "BaseDetector";
}

BaseDetector::BaseDetector(const YAML::Node& config, const std::string & name){
    BaseDetector(config, std::shared_ptr<rclcpp::Node>(new rclcpp::Node(name)),name);
}

BaseDetector::BaseDetector(const YAML::Node& config, rclcpp::Node::SharedPtr node, const std::string & name):
    config(config),
    node_(node),
    name(name){
}


}// Engineering_robot_Pnx