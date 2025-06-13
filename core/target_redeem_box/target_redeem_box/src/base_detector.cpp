#include "target_redeem_box/base_detector.hpp"

namespace Engineering_robot_Pnx{

InputData::InputData(const rclcpp::Time& time_, 
    const cv::Mat& image_, 
    const pcl::PointCloud<const pcl::PointXYZ>::Ptr pc_):
    update_time(time_),
    image(image_),
    point_cloud_(pc_)
{}

std::string const BaseDetector::get_detector_name(){
    return "BaseDetector";
}

}// Engineering_robot_Pnx