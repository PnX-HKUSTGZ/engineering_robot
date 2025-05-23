#include "engineering_resolve/engineering_resolve.hpp"


namespace Engineering_robot_Pnx{

Resolving_Node::Resolving_Node(
    std::string name,
    const std::string & ARM_CONTROL_GROUP,
    const std::string & END_EFFECTOR_CONTROL_GROUP,
    const std::string & end_link,
    rclcpp::NodeOptions node_options):
    Node(name,node_options),
    ARM_CONTROL_GROUP(ARM_CONTROL_GROUP),
    END_EFFECTOR_CONTROL_GROUP(END_EFFECTOR_CONTROL_GROUP),
    end_link(end_link){
        RCLCPP_INFO_STREAM(this->get_logger(),name<<" init ok!");
    }

bool Resolving_Node::MoveitInit(){
    std::thread([this](){
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(this->shared_from_this());
        executor.spin();
    }).detach();
    RCLCPP_INFO_STREAM(this->get_logger(),"spin node "<<this->get_name());
    try{
        move_group_=std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(),ARM_CONTROL_GROUP);
        RCLCPP_INFO(this->get_logger(),"move_group_ init ok! with group name: %s",ARM_CONTROL_GROUP.c_str());
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"MoveitInit fail with %s",e.what());
        return 0;
    }


    if(move_group_->setEndEffectorLink(end_link)){
        RCLCPP_INFO(this->get_logger(),"set the endeffector link : %s",end_link.c_str());
    }
    else{
        RCLCPP_ERROR(this->get_logger(),"fail to set the endeffector link : %s",end_link.c_str());
        return 0;
    }
    return 1;
}

void Resolving_Node::clear_constraints_state(){
    move_group_->clearPathConstraints();
    move_group_->clearPoseTargets();
    move_group_->clearTrajectoryConstraints();
}

bool Resolving_Node::setPoseTarget(const geometry_msgs::msg::Pose & targetpose){
    return move_group_->setPoseTarget(targetpose);
}

void Resolving_Node::setGoalOrientationTolerance(double tolerance){
    move_group_->setGoalOrientationTolerance(tolerance);
}

void Resolving_Node::setGoalPositionTolerance(double tolerance){
    move_group_->setGoalPositionTolerance(tolerance);
}

void Resolving_Node::setMaxVelocityScalingFactor(double factor){
    move_group_->setMaxVelocityScalingFactor(factor);
}

void Resolving_Node::setMaxAccelerationScalingFactor(double factor){
    move_group_->setMaxAccelerationScalingFactor(factor);
}

void Resolving_Node::setPlanningTime(double time){
    move_group_->setPlanningTime(time);
}

void Resolving_Node::setReplanAttempts(int times){
    move_group_->setReplanAttempts(times);
}

bool Resolving_Node::setEndEffectorLink(const std::string & end_link){
    return move_group_->setEndEffectorLink(end_link);
}

moveit::core::MoveItErrorCode Resolving_Node::plan(moveit::planning_interface::MoveGroupInterface::Plan &plan){
    return move_group_->plan(plan);
}

void Engineering_robot_Controller::Multiclear_constraints_state(){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->clear_constraints_state();
    }
}

bool Engineering_robot_Controller::MultisetPoseTarget(const geometry_msgs::msg::Pose & targetpose){
    bool ok=1;
    for(int i=0;i<MultithreadNum;i++){
        ok=ok&&resolving_nodes_[i]->setPoseTarget(targetpose);
    }
    return ok;
}

void Engineering_robot_Controller::MultisetGoalOrientationTolerance(double tolerance){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->setGoalOrientationTolerance(tolerance);
    }
}

void Engineering_robot_Controller::MultisetGoalPositionTolerance(double tolerance){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->setGoalPositionTolerance(tolerance);
    }
}

void Engineering_robot_Controller::MultisetMaxVelocityScalingFactor(double factor){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->setMaxVelocityScalingFactor(factor);
    }
}

void Engineering_robot_Controller::MultisetMaxAccelerationScalingFactor(double factor){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->setMaxAccelerationScalingFactor(factor);
    }
}

void Engineering_robot_Controller::MultisetPlanningTime(double time){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->setPlanningTime(time);
    }
}

void Engineering_robot_Controller::MultisetReplanAttempts(int times){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->setReplanAttempts(times);
    }
}

bool Engineering_robot_Controller::MultisetEndEffectorLink(const std::string & end_link){
    for(int i=0;i<MultithreadNum;i++){
        resolving_nodes_[i]->setEndEffectorLink(end_link);
    }
}

}