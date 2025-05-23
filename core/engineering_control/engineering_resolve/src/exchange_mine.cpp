#include "engineering_resolve/engineering_resolve.hpp"

namespace Engineering_robot_Pnx{

bool Engineering_robot_Controller::robot_go_pose(const std::string & name){
    clear_constraints_state();
    auto names=move_group_->getNamedTargets();
    if(std::find(names.begin(),names.end(),name)==names.end()){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose fail! no such target!");
        return 0;
    }

    move_group_->setNamedTarget(name);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success=0;
    for(int i=0;i<AllowPlanAttempt;i++){
        RCLCPP_INFO(this->get_logger(),"robot_go_pose try %d",i);
        success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            RCLCPP_INFO(this->get_logger(),"robot_go_pose plan success!");
            break;
        }
        else{
            RCLCPP_INFO(this->get_logger(),"robot_go_pose plan fail!");
        }
    }

    if(!success){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose fail!");
        return 0;
    }

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        RCLCPP_INFO(this->get_logger(),"robot_go_pose move failed! with %s",e.what());
        return 0;
    }

    return 1;
}

void Engineering_robot_Controller::unfix_RedeemBox_pos(){
    if(RedeemBox_pos_pub_timer!=nullptr){
        RCLCPP_INFO(this->get_logger(),"unfixed RedeemBox pos!");
        RedeemBox_pos_pub_timer->cancel();
        RedeemBox_pos_pub_timer=nullptr;
    }
}

geometry_msgs::msg::TransformStamped Engineering_robot_Controller::fix_RedeemBox_pos(){
    if(RedeemBox_pos_pub_timer!=nullptr){
        RCLCPP_WARN(this->get_logger(),"RedeemBox pos already fixed!, will unfixed and fix again");
        unfix_RedeemBox_pos();
    }

    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id="";

    bool get_tranform=0;
    int try_count=0;
    std::this_thread::sleep_for(10ms);
    while(!get_tranform&&try_count<50){
        try{
            msg=tf2_buffer_->lookupTransform(
                robot_base,
                "object/box",
                this->now(),
                20ns
            );
            get_tranform=1;
        }
        catch(const std::exception & e){
            RCLCPP_ERROR_STREAM(this->get_logger(),"look transform of RedeemBox fail with"<<e.what()<<", try again");
            get_tranform=0;
        }
        try_count++;
    }
    if(!get_tranform){
        RCLCPP_ERROR(this->get_logger(),"look transform of RedeemBox fail!");
        return msg;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),"get transform of RedeemBox and "<<robot_base);

    RedeemBox_pos_pub_timer=this->create_wall_timer(10ms,[this,msg](){
        geometry_msgs::msg::TransformStamped msg_=msg;
        msg_.child_frame_id="object/fixedbox";
        msg_.header.stamp=this->now();
        this->tf2_pub_->sendTransform(msg_);
    });
    RCLCPP_INFO(this->get_logger(),"create RedeemBox_pos_pub_timer!");

    return msg;

}

bool Engineering_robot_Controller::AutoExchangeMine(){

    try_times++;

    auto start_time = std::chrono::steady_clock::now();

    clearPlanScene();

    if(!robot_go_pose("get_mine")){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose home fail!");
        return 0;
    }

    // wait for robot get mine

    auto msg=fix_RedeemBox_pos();
    if(msg.header.frame_id==""){
        RCLCPP_ERROR(this->get_logger(),"fix_RedeemBox_pos fail!");
        return 0;
    }
    bool LoadAttachMinecheck=LoadAttachMine();
    if(!LoadAttachMinecheck){
        RCLCPP_ERROR(this->get_logger(),"LoadAttachMine fail!");
        return 0;
    }
    bool LoadRedeemBoxcheck=LoadRedeemBox(msg);
    if(!LoadRedeemBoxcheck){
        RCLCPP_ERROR(this->get_logger(),"LoadRedeemBox fail!");
        return 0;
    }

    RCLCPP_INFO(this->get_logger(),"LoadAttachMine and LoadRedeemBox success!");

    if(!robot_go_pose("out_mine")){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose out_mine fail!");
    }

    geometry_msgs::msg::Pose TargetPose;
    geometry_msgs::msg::PoseStamped TransformedTargetPose;
    TransformedTargetPose.header.frame_id=robot_base;
    TransformedTargetPose.header.stamp=this->now();
    TargetPose.position.x=0;
    TargetPose.position.y=0;
    TargetPose.position.z=-0.1;
    TargetPose.orientation.x=0;
    TargetPose.orientation.y=0;
    TargetPose.orientation.z=-0.7071068;
    TargetPose.orientation.w=0.7071068;
    doPoseTransform(TargetPose,TransformedTargetPose.pose,msg);

    clear_constraints_state();
    move_group_->setPlannerId(planner);
    move_group_->setPoseTarget(TransformedTargetPose.pose);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
    move_group_->setPlanningTime(minPlanTime);
    move_group_->setNumPlanningAttempts(NumPlanningAttempts);
    move_group_->setReplanAttempts(AllowPlanAttempt);
    // move_group_->setPlanningPipelineId("ompl");
    move_group_->setGoalOrientationTolerance(minOrientationTolerance);
    move_group_->setGoalPositionTolerance(minPositionTolerance);

    bool success=0;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    for(int i=0;i<AllowPlanAttempt;i++){
        // move_group_->setPlannerId(planner);
        // move_group_->setPoseTarget(TransformedTargetPose.pose);
        // move_group_->setMaxVelocityScalingFactor(1);
        // move_group_->setMaxAccelerationScalingFactor(1);
        // move_group_->setPlanningTime(minPlanTime);
        // move_group_->setNumPlanningAttempts(NumPlanningAttempts);
        // move_group_->setReplanAttempts(4);
        // // move_group_->setPlanningPipelineId("ompl");
        // move_group_->setGoalOrientationTolerance(minOrientationTolerance);
        // move_group_->setGoalPositionTolerance(minPositionTolerance);
        RCLCPP_INFO(this->get_logger(),"AutoExchangeMine try %d",i);
        moveit::core::MoveItErrorCode error=move_group_->plan(plan);
        
        success=(error==moveit::core::MoveItErrorCode::SUCCESS);
        if(!success){
            RCLCPP_ERROR(this->get_logger(),"AutoExchangeMine plan fail! with %s",moveit::core::error_code_to_string(error).c_str());
        }
        if(success){
            RCLCPP_INFO(this->get_logger(),"AutoExchangeMine plan success!");
            break;
        }
    }


    if(!success){
        RCLCPP_ERROR(this->get_logger(),"plan fail!");
    }
    else{
        RCLCPP_INFO(this->get_logger(),"plan success!");
        success_times++;        
    }

    std_msgs::msg::Float32 success_rate;
    success_rate.data=success_times/(double)try_times;
    success_rate_pub_->publish(success_rate);

    if(!success){
        return 0;
    }

    auto end_time = std::chrono::steady_clock::now();

    std::chrono::duration<double>  duration = (end_time - start_time);
    RCLCPP_INFO(this->get_logger(),"AutoExchangeMine plan time: %ld ms", duration.count());
    std_msgs::msg::Float32 plan_time;
    plan_time.data=duration.count();
    plan_time_pub_->publish(plan_time);

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(this->get_logger(),"move failed! with %s",e.what());
        return 0;
    }

    RCLCPP_INFO(this->get_logger(),"move finish !");

    return 1;
}

void Engineering_robot_Controller::commmand_executor(){
    while(rclcpp::ok()){
        std::this_thread::sleep_for(33ms);
        auto command=get_player_command();
        if(!command.is_started){
            continue;
        }
        if(command.breakout){
            continue;
        }
        RCLCPP_INFO(this->get_logger(),"commmand_executor get");

        auto computer_state=get_computer_state();

        if(computer_state.recognition==REC_FAIL){
            RCLCPP_ERROR(this->get_logger(),"recognition fail! ignore command");
            continue;
        }
        
        set_computer_state(STATE_ONE,PLANNING);

        RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe start!");

        std::thread mine_exchange_pipe_thread([this](){
            mine_exchange_pipe_state=PIPE_THREAD_RUNNING;
            clearPlanScene();
            this->mine_exchange_pipe();
            if(get_computer_state().current_state==STATE_ERROR){
                RCLCPP_ERROR(this->get_logger(),"mine_exchange_pipe fail!");
                mine_exchange_pipe_state=PIPE_THREAD_ERROR;
                return;
            }
            clearPlanScene();
            mine_exchange_pipe_state=PIPE_THREAD_OK;
        });

        while(1){
            std::this_thread::sleep_for(33ms);
            auto now_command=get_player_command();
            if(now_command.breakout){
                RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe stop by player");
                RCLCPP_INFO(this->get_logger(),"try to stop mine_exchange_pipe");
                pthread_cancel(mine_exchange_pipe_thread.native_handle());
                clearPlanScene();
                mine_exchange_pipe_thread.join();
                RCLCPP_INFO(this->get_logger(),"stop mine_exchange_pipe!");
                break;
            }
            if(mine_exchange_pipe_state==PIPE_THREAD_RUNNING){
                continue;
            }
            if(mine_exchange_pipe_state==PIPE_THREAD_OK){
                mine_exchange_pipe_thread.join();
                RCLCPP_INFO(this->get_logger(),"mine_exchange_pipe finish!");
                break;
            }
            if(mine_exchange_pipe_state==PIPE_THREAD_ERROR){
                mine_exchange_pipe_thread.join();
                RCLCPP_ERROR(this->get_logger(),"mine_exchange_pipe fail!");
                break;
            }
        }
        mine_exchange_pipe_state=PIPE_THREAD_NOLAUNCH;
        set_computer_state(STATE_WAIT,STATE_WAIT);

    }

}

void Engineering_robot_Controller::mine_exchange_pipe(){

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto msg=fix_RedeemBox_pos();
    bool LoadRedeemBoxcheck=LoadRedeemBox();
    if(!LoadRedeemBoxcheck){
        RCLCPP_ERROR(this->get_logger(),"LoadRedeemBox fail! pipe end!");
        return;
    }
    else {
        RCLCPP_INFO(this->get_logger(),"LoadRedeemBox success!");
    }

{// state 1

    set_computer_state(STATE_ONE,PLANNING);

    move_group_->setNamedTarget("get_mine");
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);

    bool success=0;
    for(int i=0;i<AllowPlanAttempt;i++){
        RCLCPP_INFO(this->get_logger(),"state 1 plan try %d",i);
        success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            RCLCPP_INFO(this->get_logger(),"state 1 plan plan success!");
            break;
        }
        else{
            RCLCPP_INFO(this->get_logger(),"state 1 plan plan fail!");
        }
    }

    if(!success){
        set_computer_state(STATE_ERROR,0);
        RCLCPP_ERROR(this->get_logger(),"state 1 plan fail!");
        return;
    }
    else{
        set_computer_state(STATE_ONE,MOVING);
        RCLCPP_INFO(this->get_logger(),"state 1 plan success!");
    }

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        set_computer_state(STATE_ERROR,0);
        RCLCPP_INFO(this->get_logger(),"state 1 move failed! with %s",e.what());
        return;
    }
    RCLCPP_INFO(this->get_logger(),"state 1 move success!");
    set_computer_state(STATE_ONE,FINISH);

}

{// state 1 waiting for player
    RCLCPP_INFO(this->get_logger(),"waiting player attach ok command....");
    while(1){
        auto player_command=get_player_command();
        if(player_command.is_attach){
            RCLCPP_INFO(this->get_logger(),"get player command , attach ok!");
            break;
        }
        std::this_thread::sleep_for(20ms);
    }

    set_computer_state(STATE_TWO,PLANNING);

    bool LoadAttachMineCheck=LoadAttachMine();
    if(!LoadAttachMineCheck){
        RCLCPP_ERROR(this->get_logger(),"LoadAttachMine failed! pipe end!");
        return;
    }
    else {
        RCLCPP_INFO(this->get_logger(),"LoadAttachMine success!");
    }

}

{// state 2
    set_computer_state(STATE_TWO,PLANNING);

    if(!robot_go_pose("out_mine")){
        RCLCPP_ERROR(this->get_logger(),"robot_go_pose out_mine fail!");
    }

    geometry_msgs::msg::Pose TargetPose;
    geometry_msgs::msg::Pose TransformedTargetPose;
    TargetPose.position.x=0;
    TargetPose.position.y=0;
    TargetPose.position.z=-0.1;
    TargetPose.orientation.x=0;
    TargetPose.orientation.y=0;
    TargetPose.orientation.z=-0.7071068;
    TargetPose.orientation.w=0.7071068;
    doPoseTransform(TargetPose,TransformedTargetPose,msg);

    bool success=0;

    clear_constraints_state();

    move_group_->setPlannerId(planner);
    move_group_->setPoseTarget(TransformedTargetPose);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
    move_group_->setPlanningTime(minPlanTime);
    move_group_->setNumPlanningAttempts(NumPlanningAttempts);
    move_group_->setReplanAttempts(AllowPlanAttempt);
    // move_group_->setPlanningPipelineId("ompl");
    move_group_->setGoalOrientationTolerance(minOrientationTolerance);
    move_group_->setGoalPositionTolerance(minPositionTolerance);

    for(int i=0;i<AllowPlanAttempt;i++){
        RCLCPP_INFO(this->get_logger(),"AutoExchangeMine try %d",i);
        moveit::core::MoveItErrorCode error=move_group_->plan(plan);
        
        success=(error==moveit::core::MoveItErrorCode::SUCCESS);
        if(!success){
            RCLCPP_ERROR(this->get_logger(),"AutoExchangeMine plan fail! with %s",moveit::core::error_code_to_string(error).c_str());
        }
        if(success){
            RCLCPP_INFO(this->get_logger(),"AutoExchangeMine plan success!");
            break;
        }
    }

    if(!success){
        set_computer_state(STATE_ERROR,0);
        RCLCPP_ERROR(this->get_logger(),"state 2 plan fail!");
        return;
    }
    else{
        set_computer_state(STATE_TWO,MOVING);
        RCLCPP_INFO(this->get_logger(),"state 2 plan success!");
    }

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        set_computer_state(STATE_ERROR,0);
        RCLCPP_INFO(this->get_logger(),"state 2 move failed! with %s",e.what());
        return ;
    }
    RCLCPP_INFO(this->get_logger(),"state 2 move success!");
    set_computer_state(STATE_TWO,FINISH);

}

// wait command
{
    RCLCPP_INFO(this->get_logger(),"waiting player release ok command....");
    while(1){
        auto player_command=get_player_command();
        if(player_command.is_finish){
            RCLCPP_INFO(this->get_logger(),"get player command , release ok!");
            break;
        }
        std::this_thread::sleep_for(20ms);
    }
    set_computer_state(STATE_THREE,PLANNING);
}

{// state 3
    set_computer_state(STATE_THREE,PLANNING);
    try{
        move_group_->detachObject("Mine");
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(this->get_logger(),"Detach Mine error with %s",e.what());
    }
    RemoveObject("Mine");

    move_group_->setNamedTarget("get_mine");
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);

    bool success=0;
    for(int i=0;i<AllowPlanAttempt;i++){
        RCLCPP_INFO(this->get_logger(),"state 3 plan try %d",i);
        success=(move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);
        if(success){
            RCLCPP_INFO(this->get_logger(),"state 3 plan plan success!");
            break;
        }
        else{
            RCLCPP_INFO(this->get_logger(),"state 3 plan plan fail!");
        }
    }

    if(!success){
        set_computer_state(STATE_ERROR,0);
        RCLCPP_ERROR(this->get_logger(),"state 3 plan fail!");
        return;
    }
    else{
        set_computer_state(STATE_THREE,MOVING);
        RCLCPP_INFO(this->get_logger(),"state 3 plan success!");
    }

    try{
        move_group_->execute(plan);
    }
    catch(const std::exception & e){
        set_computer_state(STATE_ERROR,0);
        RCLCPP_INFO(this->get_logger(),"state 3 move failed! with %s",e.what());
        return;
    }
    RCLCPP_INFO(this->get_logger(),"state 3 move success!");
    set_computer_state(STATE_THREE,FINISH);

}

    std::this_thread::sleep_for(8ms); // 休息一下

}

}// namespace Engineering_robot_Pnx