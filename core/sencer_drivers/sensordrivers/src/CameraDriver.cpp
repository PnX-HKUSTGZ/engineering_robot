#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "SensorDrivers/MvCameraControl.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/utils.hpp>
#include <yaml-cpp/yaml.h>

#define DEBUG

using namespace std::chrono;
using namespace std::placeholders;

namespace Engineering_robot_Pnx{

class CameraDriver : public rclcpp::Node{
    public:
    CameraDriver(rclcpp::NodeOptions options=rclcpp::NodeOptions()):
        rclcpp::Node("camera_driver", options){

        std::string package_name="sensordrivers";
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
        RCLCPP_INFO(this->get_logger(),"config file path: %s",config_file_path.c_str());

        get_image_thread=std::make_shared<std::thread>([this](){
            while(1){
                this->get_image();
            }
        });

        tf_broadcaster_=std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        current_image=nullptr;
        // Image_service = this->create_service<interfaces::srv::Imagerequest>("/sensor/camera/images",
        //     std::bind(&CameraDriver::Image_service_callback,this,_1,_2));

        RCLCPP_INFO(this->get_logger(),"tf_broadcaster_ init ok!");

        try{
            cameraMatrix=config["camera"]["camera_matrix"].as<std::vector<double>>();
            distCoeffs=config["camera"]["dist_coeffs"].as<std::vector<double>>();
        }
        catch(YAML::Exception& e){
            RCLCPP_ERROR(this->get_logger(),"error reading config file while load cameraMatrix & distCoeffs: %s",e.what());
            rclcpp::shutdown();
        }
        cameraMatrixMat=cv::Mat(cv::Size(3,3),CV_64F);
        for(int i=0;i<9;i++){
            cameraMatrixMat.at<double>(i/3,i%3)=cameraMatrix[i];
        }
        RCLCPP_INFO(this->get_logger(),"reading config file while load cameraMatrix & distCoeffs OK!");


        geometry_msgs::msg::TransformStamped to_map;

        try{
            to_map.header.frame_id="robot_base_link";
            to_map.child_frame_id="sensor/camera";
            to_map.header.stamp=this->now();
            to_map.transform.rotation.w=config["object_pos"]["camera"]["rotate"]["w"].as<double>();
            to_map.transform.rotation.x=config["object_pos"]["camera"]["rotate"]["x"].as<double>();
            to_map.transform.rotation.y=config["object_pos"]["camera"]["rotate"]["y"].as<double>();
            to_map.transform.rotation.z=config["object_pos"]["camera"]["rotate"]["z"].as<double>();
            to_map.transform.translation.x=config["object_pos"]["camera"]["translation"]["x"].as<double>();
            to_map.transform.translation.y=config["object_pos"]["camera"]["translation"]["y"].as<double>();
            to_map.transform.translation.z=config["object_pos"]["camera"]["translation"]["z"].as<double>();
        }
        catch(YAML::Exception& e){
            RCLCPP_ERROR(this->get_logger(),"error reading config file while load tf2: %s",e.what());
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(),"reading config file while load tf2 OK!");
    
        tf_broadcaster_->sendTransform(to_map);

        RCLCPP_INFO(this->get_logger(),"pub camera tf2 ok!");

        debug_pub=this->create_publisher<sensor_msgs::msg::Image>("/sensor/camera/images",10);
        
    }
    private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    YAML::Node config;
    sensor_msgs::msg::Image::SharedPtr current_image;
    std::map<u_int32_t,bool> cline_id;
    // mtx for current_image
    std::mutex image_mutex;

    // thread that get_image
    std::shared_ptr<std::thread> get_image_thread;

    rclcpp::TimerBase::SharedPtr get_image_timer;

    std::vector<double> cameraMatrix;
    std::vector<double> distCoeffs;
    cv::Mat cameraMatrixMat;

    int nRet = MV_OK;
    
    #ifdef DEBUG
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> debug_pub;
    #endif

    private: //camera related

    void get_image(){

        if(!rclcpp::ok()){
            rclcpp::shutdown();
        }

        void* handle = NULL;
        nRet = MV_CC_Initialize();
    do{
        if (MV_OK != nRet){
            RCLCPP_INFO(this->get_logger(),"Initialize SDK fail! nRet [0x%x]", nRet);
            break;
        }
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_EnumDevices fail! nRet [%x]", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0){
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++){
                RCLCPP_INFO(this->get_logger(),"[device %d]:", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo){
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }
        } 
        else{
            RCLCPP_ERROR(this->get_logger(),"Find No Devices!");
            break;
        }
    
        RCLCPP_INFO(this->get_logger(),"Please Intput camera index: ");
        unsigned int nIndex = 0;
        // scanf("%d", &nIndex);
    
        if (nIndex >= stDeviceList.nDeviceNum){
            RCLCPP_ERROR(this->get_logger(),"Intput error!");
            break;
        }
    
        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_CreateHandle fail! nRet [%x]", nRet);
            break;
        }
        else RCLCPP_INFO(this->get_logger(),"MV_CC_CreateHandle correct");
    
        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_OpenDevice fail! nRet [%x]", nRet);
            break;
        }
        else RCLCPP_INFO(this->get_logger(),"MV_CC_OpenDevice successfully");
    
        // 设置触发模式为off
        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
            break;
        }
    
        GainAdjustment(handle);
        // 注册抓图回调
        // register image callback
        nRet = MV_CC_RegisterImageCallBackEx(handle, &CameraDriver::ImageCallBackEx, this);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_RegisterImageCallBackEx fail! nRet [%x]", nRet);
            break; 
        }
        else RCLCPP_INFO(this->get_logger(),"MV_CC_RegisterImageCallBackEx correct");
    
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_StartGrabbing fail! nRet [%x]", nRet);
            break;
        }
        else RCLCPP_INFO(this->get_logger(),"MV_CC_StartGrabbing correct");
    
        std::this_thread::sleep_for(100000ms);
    
        // 停止取流
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_StopGrabbing fail! nRet [%x]", nRet);
            break;
        }
        else{
            RCLCPP_INFO(this->get_logger(),"MV_CC_StopGrabbing correct");
        }
    
        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            RCLCPP_ERROR(this->get_logger(),"MV_CC_CloseDevice fail! nRet [%x]", nRet);
            break;
        }
    
        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_DestroyHandle fail! nRet [%x]", nRet);
            break;
        }
        handle = NULL;
    }while(0);
    
        if(handle!=NULL){
            MV_CC_DestroyHandle(handle);
            handle=NULL;
        }
        MV_CC_Finalize();
    }
    
    void GainAdjustment(void* handle){
        //load in
        int ExposureTimeLower;
        int ExposureTimeUpper;
        float GainValue;
    
        try{
            ExposureTimeLower=this->config["camera"]["ExposureTimeLower"].as<int>();
            ExposureTimeUpper=this->config["camera"]["ExposureTimeUpper"].as<int>();
            GainValue=this->config["camera"]["Gain"].as<double>();
        }
        catch(YAML::Exception& e){
            RCLCPP_ERROR(this->get_logger(),"error reading config file while load camera gain and exposure time: %s",e.what());
            rclcpp::shutdown();
        }

        nRet=MV_CC_SetFloatValue(handle, "Gain", GainValue);
        if(nRet!=MV_OK){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_SetGain fail! nRet [%x]",nRet);
        }
        else RCLCPP_INFO(this->get_logger(),"Set Gain : %f",GainValue);
        
    
        MVCC_FLOATVALUE exposurtime;
        nRet=MV_CC_GetExposureTime(handle,&exposurtime);
        if(nRet!=MV_OK){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_GetExposureTime fail! nRet [%x]",nRet);
        }
        else{
            RCLCPP_INFO(this->get_logger(),"FPS : %f",exposurtime.fCurValue);
            RCLCPP_INFO(this->get_logger(),"Upper exposure time : %f",exposurtime.fMax);
            RCLCPP_INFO(this->get_logger(),"Lower exposure time : %f",exposurtime.fMin);
        }
    
        nRet=MV_CC_SetAutoExposureTimeUpper(handle,ExposureTimeUpper);
        if(nRet!=MV_OK){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_SetAutoExposureTimeUpper fail! nRet [%x]",nRet);
        }
        else RCLCPP_INFO(this->get_logger(),"Set ExposureTimeUpper : %d",ExposureTimeUpper);
    
        nRet=MV_CC_SetAutoExposureTimeLower(handle,ExposureTimeLower);
        if(nRet!=MV_OK){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_SetAutoExposureTimeLower fail! nRet [%x]",nRet);
        }
        else RCLCPP_INFO(this->get_logger(),"Set ExposureTimeLower : %d",ExposureTimeLower);
    
    
        nRet=MV_CC_SetAutoExposureTimeUpper(handle,ExposureTimeUpper);
        if(nRet!=MV_OK){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_SetAutoExposureTimeUpper fail! nRet [%x]",nRet);
        }
        else RCLCPP_INFO(this->get_logger(),"Set ExposureTimeUpper : %d",ExposureTimeUpper);
    
        nRet=MV_CC_SetAutoExposureTimeLower(handle,ExposureTimeLower);
        if(nRet!=MV_OK){
            RCLCPP_ERROR(this->get_logger(),"MV_CC_SetAutoExposureTimeLower fail! nRet [%x]",nRet);
        }
        else RCLCPP_INFO(this->get_logger(),"Set ExposureTimeLower : %d",ExposureTimeLower);
    
    }

    bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            RCLCPP_INFO(this->get_logger(),"The Pointer of pstMVDevInfo is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
    
            // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
            RCLCPP_INFO(this->get_logger(),"Device Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
            RCLCPP_INFO(this->get_logger(),"CurrentIp: %d.%d.%d.%d" , nIp1, nIp2, nIp3, nIp4);
            RCLCPP_INFO(this->get_logger(),"UserDefinedName: %s" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            RCLCPP_INFO(this->get_logger(),"Device Model Name: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
            RCLCPP_INFO(this->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
        {
            RCLCPP_INFO(this->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
        {
            RCLCPP_INFO(this->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
        {
            RCLCPP_INFO(this->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
        {
            RCLCPP_INFO(this->get_logger(),"UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
            RCLCPP_INFO(this->get_logger(),"Serial Number: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
            RCLCPP_INFO(this->get_logger(),"Model Name: %s", pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),"Not support.");
        }
    
        return true;
    }

    static void __stdcall ImageCallBackEx(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser){
        if(!rclcpp::ok()){
            rclcpp::shutdown();
        }
        if (!pFrameInfo) return;

        CameraDriver* node= static_cast<CameraDriver*> (pUser);
    
        cv::Mat OriginalImage(pFrameInfo->nExtendHeight, pFrameInfo->nExtendWidth,CV_8UC1,pData);
        cv::Mat imageRGB;
        cv::cvtColor(OriginalImage, imageRGB, cv::COLOR_BayerBG2BGR);
        cv::undistort(imageRGB,OriginalImage,node->cameraMatrixMat,node->distCoeffs);
    
        auto image_ptr=cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",imageRGB).toImageMsg();

        image_ptr->header.frame_id="/sensor/camera";
        image_ptr->header.stamp=node->get_clock()->now();
    
        node->debug_pub->publish(*image_ptr);

        node->image_mutex.lock();
        node->current_image=image_ptr;
        node->current_image->header.stamp=node->now();
        node->current_image->header.frame_id="/sensor/camera";
        node->cline_id.clear();
        node->image_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }

};//CameraDriver

}//Engineering_robot_Pnx

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
    Engineering_robot_Pnx::CameraDriver);