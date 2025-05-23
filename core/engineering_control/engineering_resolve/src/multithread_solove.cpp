#include <engineering_resolve/engineering_resolve.hpp>

namespace Engineering_robot_Pnx{

// bool Engineering_robot_Controller::MultithreadedPlanne(
//     const planning_interface::MotionPlanRequest& req, 
//     planning_interface::MotionPlanResponse &res,
//     int threadnum){

//     std::vector<std::thread> threads;
//     // 线程状态 
//     /*
//     0 未开始 
//     1 正在运行 
//     2 运行结束 
//     3 运行出错
//     */
//     std::vector<std::atomic<MultithreadState> > thread_status(threadnum);
//     for(int i=0;i<threadnum;i++){
//         thread_status[i]=MultithreadState::PLANE_THREAD_NOLAUNCH;
//     }
//     std::vector<planning_interface::MotionPlanResponse> thread_res(threadnum);

//     for(int i=0;i<threadnum;i++){
//         threads.push_back(std::thread([i,&thread_res,&thread_status,req,this](){
//             thread_status[i]=MultithreadState::PLANE_THREAD_RUNNING;
//             bool ok=0;
//             {// LockedPlanningSceneRO
//             planning_scene_monitor::LockedPlanningSceneRO lscene(planning_scene_monitors_[i]);

//             // moveit::core::RobotModelPtr robot_model = robot_model_loaders_[i]->getModel();
//             planning_pipeline::PlanningPipelinePtr planning_pipeline_(
//                 new planning_pipeline::PlanningPipeline(lscene->getRobotModel(), this->shared_from_this(), "ompl"));

//             try{
//                 ok=planning_pipeline_->generatePlan(lscene, req, thread_res[i]);
//             }
//             catch(const std::exception& e){
//                 RCLCPP_ERROR(this->get_logger(), "thread NO.%d Planning failed: %s", i,e.what());
//                 thread_status[i]=MultithreadState::PLANE_THREAD_ERROR;
//                 return;
//             }
//             }// LockedPlanningSceneRO
//             if(ok&&thread_res[i].error_code_.val==thread_res[i].error_code_.SUCCESS){
//                 thread_status[i]=MultithreadState::PLANE_THREAD_OK;
//                 RCLCPP_INFO(this->get_logger(), "thread NO.%d Planning succeeded!", i);
//                 return;
//             }
//             else{
//                 thread_status[i]=MultithreadState::PLANE_THREAD_ERROR;
//                 RCLCPP_INFO(this->get_logger(), "thread NO.%d Planning failed!", i);
//                 return;
//             }
//         }));            
//     }

//     RCLCPP_INFO(this->get_logger(), "MultithreadedPlanne started!");

//     // 等待有线程算出正确答案，或者所有线程出错
//     bool get_ans=0;
//     while(!get_ans){
//         std::this_thread::sleep_for(20ms);
//         int failcount=0;
//         for(int i=0;i<threadnum;i++){
//             if(thread_status[i]==MultithreadState::PLANE_THREAD_OK){
//                 res=thread_res[i];
//                 get_ans=true;
//             }
//             if(thread_status[i]==MultithreadState::PLANE_THREAD_ERROR){
//                 failcount++;
//             }
//         }
//         if(failcount==threadnum){
//             break;
//         }
//     }


//     for(int i=0;i<threadnum;i++){
//         threads[i].join();
//     }

//     if(get_ans){
//         RCLCPP_INFO(this->get_logger(), "MultithreadedPlanne succeeded!");
//     }
//     else{
//         RCLCPP_INFO(this->get_logger(), "MultithreadedPlanne failed!");
//     }


//     return get_ans;

// }

bool Engineering_robot_Controller::MultithreadedPlanne(
    moveit::planning_interface::MoveGroupInterface::Plan& plan){

    std::vector<std::thread> threads;
    std::vector<std::atomic<MultithreadState> > thread_status(MultithreadNum);
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> thread_plans(MultithreadNum);

    for(int i=0;i<MultithreadNum;i++){
        thread_status[i]=MultithreadState::PLANE_THREAD_NOLAUNCH;
    }
    for(int i=0;i<MultithreadNum;i++){
        threads.push_back(std::thread([i,&thread_plans,&thread_status,this](){
            thread_status[i]=MultithreadState::PLANE_THREAD_RUNNING;
            moveit::core::MoveItErrorCode state=resolving_nodes_[i]->plan(thread_plans[i]);
            bool ok=(state==moveit::core::MoveItErrorCode::SUCCESS);
            if(ok){
                thread_status[i]=MultithreadState::PLANE_THREAD_OK;
                RCLCPP_INFO(this->get_logger(), "thread NO.%d Planning succeeded!", i);
                return;
            }
            else{
                thread_status[i]=MultithreadState::PLANE_THREAD_ERROR;
                RCLCPP_INFO(this->get_logger(), "thread NO.%d Planning failed!", i);
                return;
            }
        }));
    }

    // 等待有线程算出正确答案，或者所有线程出错
    bool get_ans=0;
    while(!get_ans&&rclcpp::ok()){
        std::this_thread::sleep_for(20ms);
        int failcount=0;
        for(int i=0;i<MultithreadNum;i++){
            if(thread_status[i]==MultithreadState::PLANE_THREAD_OK){
                plan=thread_plans[i];
                get_ans=true;
                break;
            }
            if(thread_status[i]==MultithreadState::PLANE_THREAD_ERROR){
                failcount++;
            }
        }
        if(failcount==MultithreadNum){
            break;
        }
        if(!rclcpp::ok()){
            rclcpp::shutdown();
        }
    }

    for(int i=0;i<MultithreadNum;i++){
        threads[i].join();
    }

    if(get_ans){
        RCLCPP_INFO(this->get_logger(), "MultithreadedPlanne succeeded!");
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "MultithreadedPlanne failed!");
    }
    return get_ans;
}


bool Engineering_robot_Controller::DemonRun(){
    
}




}