#include "target_redeem_box/detector_manager.hpp"

#include "target_redeem_box/arrow_detector.hpp"
#include "target_redeem_box/retangle_detector.hpp"
#include "target_redeem_box/arrow_detector_pcl.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace Engineering_robot_Pnx{

DetectorManager::DetectorManager() : rclcpp::Node("target_redeem_box_detector_manager",rclcpp::NodeOptions()){
    std::string package_name="target_redeem_box";
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

    try{
    image_topic=config["topics"]["image_topic"].as<std::string>();
    image_frame=config["topics"]["image_frame"].as<std::string>();
    point_cloud_topic=config["tf_frames"]["point_cloud_topic"].as<std::string>();
    point_cloud_frame=config["tf_frames"]["point_cloud_frame"].as<std::string>();
    input_data_time_out = rclcpp::Duration::from_seconds(config["input_data_time_out"].as<std::vector<int>>()[0]) + 
        rclcpp::Duration::from_nanoseconds(config["input_data_time_out"].as<std::vector<int>>()[1]);
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(), "Failed to load config file '%s': %s", config_file_path.c_str(), e.what());
        return;
    }
    RCLCPP_INFO(this->get_logger(),"load config file '%s' success",config_file_path.c_str());
    

    tf2_buffer_=std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_=std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    posed_image_publisher_=this->create_publisher<sensor_msgs::msg::Image>("colored_image",10);
    
    image_subscriber=std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this,image_topic,10);
    point_cloud_subscriber_=std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this,point_cloud_topic,10);
    synchronizer_=std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *image_subscriber, *point_cloud_subscriber_);
    synchronizer_->registerCallback(
        [this](const sensor_msgs::msg::Image::SharedPtr& image_msg, const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud_msg){
            this->image_point_cloud_callback(image_msg,point_cloud_msg);
        }
    );

    bool init_detectors_success=init_detectors();
    if(!init_detectors_success){
        RCLCPP_FATAL(this->get_logger(),"init detectors error");
        return;
    }
    RCLCPP_INFO(this->get_logger(),"init detectors success");

}

void DetectorManager::image_point_cloud_callback(const sensor_msgs::msg::Image::SharedPtr& image_msg, 
        const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud_msg){

    // 转换图像

    cv::Mat image;
    try{
        cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(image);
    }
    catch (cv_bridge::Exception& e){
    // 转换失败
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 转换点云

    geometry_msgs::msg::TransformStamped transform;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> transformed_cloud;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> transformed_cloud_pcl;
    
    try{
        transform=tf2_buffer_->lookupTransform(
            image_frame,
            point_cloud_frame,
            this->now(),
            rclcpp::Duration::from_seconds(1.0)
        );
    }
    catch (tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(),"[image_point_cloud_callback]: %s",ex.what());
        return;
    }


    tf2::doTransform(*point_cloud_msg, *transformed_cloud, transform);
    pcl::fromROSMsg(*transformed_cloud, *transformed_cloud_pcl);

    // 包装数据
    InputData::SharedPtr input_data=std::make_shared<InputData>(this->now(),image,transformed_cloud_pcl);
    
    {
        std::lock_guard<std::mutex> lock(input_data_mutex);
        this->input_data=input_data;
    }

    RCLCPP_INFO(this->get_logger(),"[image_point_cloud_callback] input data update");

}
        
bool DetectorManager::init_detectors(){
    if(!(config["detectors"].IsDefined()&&config["detectors"].IsSequence())){
        RCLCPP_FATAL(this->get_logger(),"[init_detectors] get detectors list config error");
        throw std::runtime_error("[init_detectors] get detectors list config error");
        return false;
    }

    try{
    int detector_num=config["detectors"].size();
    for(int i=0;i<detector_num;i++){
        std::string detector_type=config["detectors"][i][0].as<std::string>();
        std::string detector_name=config["detectors"][i][1].as<std::string>();
        YAML::Node config_node= (config["detector_config"][detector_name].IsMap() ? config["detector_config"][detector_name] : YAML::Node());

        if(detector_type=="arrow_detector"){
            std::shared_ptr<ArrowDetector> detector=std::make_shared<ArrowDetector>(
                config_node,
                detector_name);
            detectors.push_back(detector);
        }
        else if(detector_type=="arrow_detector_pcl"){
            std::shared_ptr<ArrowDetectorPCL> detector=std::make_shared<ArrowDetectorPCL>(
                config_node,
                detector_name);
            detectors.push_back(detector);
        }
        else if(detector_type=="rectangle_detector"){
            std::shared_ptr<RectangleDetector> detector=std::make_shared<RectangleDetector>(
                config_node,
                detector_name);
            detectors.push_back(detector);
        }
        else{
            RCLCPP_ERROR(this->get_logger(),"[init_detectors] get detector type error, get %s. skip",detector_type.c_str());
        }
    }

    if(detectors.size()!=detector_num){
        RCLCPP_FATAL(this->get_logger(),"[init_detectors] init detectors error, not all detectors init success, init %d, get %d",detectors.size(),detector_num);
        return false;
    }
    }
    catch(const std::exception& e){
        RCLCPP_FATAL(this->get_logger(),"[init_detectors] init arrow detector error:%s",e.what());
        throw std::runtime_error("[init_detectors] init arrow detector error");
        return false;
    }

    RCLCPP_INFO(this->get_logger(),"[init_detectors] init detectors success, init %d",detectors.size());
    return true;

}

void DetectorManager::start_detect(){
    auto detect_function = [this](const std::shared_ptr<BaseDetector> detector){
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr detect_result_publisher=
        this->create_publisher<std_msgs::msg::Int32>(detector->get_detector_name()+"_time_use",10);
        while(true){
            if(!rclcpp::ok()){
                break;
            }
            std::shared_ptr<InputData> input_data;
            {
                std::lock_guard<std::mutex> lock(input_data_mutex);
                input_data=this->input_data;
            }
            if(input_data==nullptr){
                continue;
            }
            if(input_data->update_time-this->now() > this->input_data_time_out){
                RCLCPP_WARN(this->get_logger(),"[%s] input data time out",detector->get_detector_name().c_str());
                continue;
            }
            auto start_time = std::chrono::high_resolution_clock::now();
            DetectorOutput output;
            bool detect_success=detector->detect(*input_data,output);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            std_msgs::msg::Int32 time_use_msg;
            time_use_msg.data=duration.count();
            detect_result_publisher->publish(time_use_msg);
            handle_detect_result(output);
        }
    };

    for(auto & detector: detectors){
        detect_threads.push_back(std::thread(detect_function,detector));
    }

    RCLCPP_INFO(this->get_logger(),"[start_detect] start detect %d",detectors.size());
}

}
