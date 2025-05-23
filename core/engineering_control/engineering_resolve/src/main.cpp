#include "engineering_resolve/engineering_resolve.hpp"

int main (int argc,char** argv){
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Engineering_robot_Pnx::Engineering_robot_Controller>();

    if(!node->MoveitInit()){
        RCLCPP_ERROR(node->get_logger(),"MoveitInit fail!");
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    
}