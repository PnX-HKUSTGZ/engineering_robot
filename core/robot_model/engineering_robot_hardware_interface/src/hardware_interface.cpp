#include "engineering_robot_hardware_interface/hardware_interface.hpp"

namespace Engineering_robot_Pnx{

void ERHardwareInterface::computer_state_sub_callback(command_interfaces::msg::ComputerState::ConstSharedPtr msg){
    ComputerState state;
    state.current_state=msg->current_state;
    state.pos1_state=msg->pos1_state;
    state.pos2_state=msg->pos2_state;
    state.pos3_state=msg->pos3_state;
    state.recognition=msg->recognition;
    set_computer_state(state);
}

PlayerCommandContent ERHardwareInterface::get_player_command(){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    return player_command;
}

ComputerState ERHardwareInterface::get_computer_state(){
    std::lock_guard<std::mutex> ul(computer_state_mutex);
    return computer_state;
}

void ERHardwareInterface::set_player_command(const PlayerCommandContent & input_command){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    player_command=input_command;
}

void ERHardwareInterface::set_computer_state(const ComputerState & input_state){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    computer_state=input_state;
}

CallbackReturn ERHardwareInterface::on_init(const HardwareInfo & hardware_info){
    RCLCPP_INFO_STREAM(logger,"on_init called!\n");
    device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
    try{
        owned_ctx_=std::make_unique<drivers::common::IoContext>(2);
        serial_driver_=std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
        serial_driver_->init_port(device_name_,*device_config_);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(logger, "Load drivers::serial_driver::SerialDriver failed with "<<e.what()<<"\n");
        return CallbackReturn::ERROR;
    }

    open();

    std::map<std::string,bool> joiny_interface;
    for(auto & i : hardware_info.joints){
        joiny_interface[i.name]=1;
    }
    for(auto & i : joint_name){
        if(!joiny_interface[i]) return CallbackReturn::ERROR;
    }

    auto computer_state=get_computer_state();
    computer_state.current_state=0;
    computer_state.pos1_state=0;
    computer_state.pos2_state=0;
    computer_state.pos3_state=0;
    computer_state.recognition=0;
    set_computer_state(computer_state);

    node_=std::make_shared<rclcpp::Node>("hardware_interface");

    single_executor_=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    single_executor_->add_node(node_);

    single_executor_thread_=std::make_shared<std::thread>([this](){
        single_executor_->spin();
    });
    single_executor_thread_->detach();

    computer_state_sub_=node_->create_subscription<command_interfaces::msg::ComputerState>("/computer_state",1,[this](command_interfaces::msg::ComputerState::ConstSharedPtr msg){
        computer_state_sub_callback(msg);
    });

    player_command_pub_=node_->create_publisher<command_interfaces::msg::PlayerCommand>("/player_command",1);

    RCLCPP_INFO_STREAM(logger,"on_init called OK!\n");
    return CallbackReturn::SUCCESS;
}

void ERHardwareInterface::on_configure(){
    RCLCPP_INFO_STREAM(logger,"on_configure called!\n");
}

void ERHardwareInterface::on_cleanup(){
    RCLCPP_INFO_STREAM(logger,"on_cleanup  called!\n");
}

std::vector<StateInterface> ERHardwareInterface::export_state_interfaces(){
    std::vector<StateInterface> state;

    for(int i=0;i<6;i++){
        state.push_back(StateInterface(joint_name[i],"position",&state_data[i]));
        state.push_back(StateInterface(joint_name[i],"velocity",&v[i]));
        // RCLCPP_INFO_STREAM(logger,"add state:"<<joint_name[i]);
    }

    return state;
}

std::vector<CommandInterface> ERHardwareInterface::export_command_interfaces(){
    std::vector<CommandInterface> inter;
    for(int i=0;i<6;i++){
        inter.push_back(CommandInterface(joint_name[i],"position",&WID_p[i]));
        inter.push_back(CommandInterface(joint_name[i],"velocity",&WID_v[i]));
        // RCLCPP_INFO_STREAM(logger,"add command:"<<joint_name[i]);
    }
    return inter;
}


return_type ERHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){

    // RCLCPP_INFO_STREAM(logger,"read call read at"<<time.seconds()<<","<<time.nanoseconds());
    try{
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data;
        NowPosition packet;
        data.reserve(sizeof(NowPosition));
        bool get_message=0;
        while(!get_message){
            try {
                serial_driver_->port()->receive(header);
                if (header[0] == 0xA5) {
                    // RCLCPP_INFO_STREAM(logger,data.size());
                    data.resize(sizeof(NowPosition) - 1);
                    // RCLCPP_INFO_STREAM(logger,"===================================="<<data.size());
                    serial_driver_->port()->receive(data);
    
                    data.insert(data.begin(), header[0]);
                    // RCLCPP_INFO_STREAM(logger,"===================================="<<data.size());
                    // for(auto i : data){
                        // RCLCPP_INFO_STREAM(logger,""<<int(i));
                    // }
                    // RCLCPP_INFO_STREAM(logger,"=====================================");
                    fromVector(data,packet);
    
                    int expectcrc=Get_CRC16_Check_Sum(reinterpret_cast<const uint8_t*>(&packet),52,0xFFFF);
                    bool crc_ok= (expectcrc==packet.crc16);
                    // RCLCPP_INFO_STREAM(logger,"get_crc= "<<packet.crc16<<" expect_crc= "<<expectcrc);
                    if (!crc_ok&&packet.crc16!=0) { // crc 0 表示虚拟串口
                        RCLCPP_ERROR_STREAM(logger, "crc check fail! crc:");
                        // throw "crc check fail!";
                    }
                    get_message=1;
                }
                else{
                    RCLCPP_WARN_STREAM(logger,"read fail with the bad head: "<<int(header[0]));
                    
                }
            }
            catch(std::exception & e){
                RCLCPP_ERROR_STREAM(logger, "read_from_serial fialed with "<<e.what());
                reopen();
                get_message=0;
                return return_type::OK;
                // throw e;
            }
        }
    
        for(int i=0;i<6;i++){
            state_data[i]=packet.posv[i][0];
            v[i]=packet.posv[i][1];
        }
        state_data[0]=-state_data[0];
        v[0]=-v[0];
        // state_data[4]+=1.5707963267948966192313215;

        PlayerCommandContent nowcommand;
        nowcommand.breakout=packet.breakout;
        nowcommand.is_finish=packet.is_finish;
        nowcommand.is_started=packet.is_started;
        nowcommand.is_attach=packet.is_attach;
        nowcommand.command_time=node_->now();

        command_interfaces::msg::PlayerCommand msg;
        msg.breakout=nowcommand.breakout;
        msg.is_finish=nowcommand.is_finish;
        msg.is_started=nowcommand.is_started;
        msg.is_attach=nowcommand.is_attach;
        msg.header.frame_id="/playercommand";
        msg.header.stamp=node_->now();

        player_command_pub_->publish(msg);

        // RCLCPP_INFO_STREAM(logger,"read_from_serial ok!");
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(logger, "read_from_serial failed with "<<e.what());
        reopen();
        return return_type::OK;
    }
    return return_type::OK;
}

return_type ERHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
    SendDate packet;
    auto current_state=get_computer_state();
    packet.current_state=current_state.current_state;
    packet.pos1_state=current_state.pos1_state;
    packet.pos2_state=current_state.pos2_state;
    packet.pos3_state=current_state.pos3_state;
    packet.recognition=current_state.recognition;
    // RCLCPP_INFO_STREAM(logger,"write called!");
    for(int i=0;i<6;i++){
        packet.posv[i][0]=WID_p[i];
        packet.posv[i][1]=WID_v[i];
        // RCLCPP_INFO_STREAM(logger,"add "<<WID_p[i]<<","<<WID_v[i]);
    }
    // packet.posv[4][0]-=1.57079632679496619;
    packet.posv[0][0]=-packet.posv[0][0];
    packet.posv[0][1]=-packet.posv[0][1];
    uint16_t crc16 = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), 53, 0xFFFF);
    packet.crc16=crc16;
    // RCLCPP_INFO_STREAM(logger,"crc "<<crc16);
    std::vector<uint8_t> data = toVector(packet);

    try{
        serial_driver_->port()->send(data);
    }
    catch(std::exception & e){
        RCLCPP_ERROR_STREAM(logger, "serial_driver_->port()->send() failed with "<<e.what());
        reopen();
    }

    // RCLCPP_INFO_STREAM(logger,"write finish!");
    return return_type::OK;

}

void ERHardwareInterface::on_activate(){

}

void ERHardwareInterface::on_deactivate(){

}

void ERHardwareInterface::on_shutdown(){

}

void ERHardwareInterface::open(){
    bool noerror=0;
    while(!noerror){
        try{
            if (!serial_driver_->port()->is_open()){
                serial_driver_->port()->open();
            }
            noerror=1;
        }
        catch (const std::exception &ex){
            RCLCPP_ERROR_STREAM(logger, "open Serial failed with "<<ex.what()<<", wait 200 ms.\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            noerror=0;
        }
    }
}

void ERHardwareInterface::reopen(){
    if(serial_driver_->port()->is_open()){
        serial_driver_->port()->close();
    }
    
    bool noerror=0;
    while(!noerror){
        try{
            if (!serial_driver_->port()->is_open()){
                serial_driver_->port()->open();
            }
            noerror=1;
        }
        catch (const std::exception &ex){
            RCLCPP_ERROR_STREAM(logger, "open Serial failed with "<<ex.what()<<", wait 200 ms.\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            noerror=0;
        }
    }
}


void ERHardwareInterface::on_error(){
    RCLCPP_INFO_STREAM(logger,"on_error called!");
    if(serial_driver_->port()->is_open()){
        serial_driver_->port()->close();
    }
    if(owned_ctx_){
        owned_ctx_->waitForExit();
    }
    try{
        owned_ctx_=std::make_unique<drivers::common::IoContext>(2);
        serial_driver_=std::make_unique<drivers::serial_driver::SerialDriver>(*owned_ctx_);
    }
    catch(const std::exception & e){
        RCLCPP_ERROR_STREAM(logger, "Load drivers::serial_driver::SerialDriver failed with "<<e.what()<<"\n");
    }

    reopen();

}


}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(Engineering_robot_Pnx::ERHardwareInterface, hardware_interface::SystemInterface)