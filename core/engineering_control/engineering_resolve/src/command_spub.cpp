#include "engineering_resolve/engineering_resolve.hpp"
namespace Engineering_robot_Pnx{

void Engineering_robot_Controller::set_player_command(const PlayerCommandContent & input_command){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    player_command=input_command;
}

void Engineering_robot_Controller::set_computer_state(const ComputerState & input_state){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    computer_state=input_state;
}

void Engineering_robot_Controller::player_command_sub_callback(const command_interfaces::msg::PlayerCommand::ConstSharedPtr & msg){
    PlayerCommandContent input_command;
    input_command.command_time=msg->header.stamp;
    input_command.breakout=msg->breakout;
    input_command.is_finish=msg->is_finish;
    input_command.is_attach=msg->is_attach;
    input_command.is_started=msg->is_started;
    set_player_command(input_command);
}

void Engineering_robot_Controller::computer_state_pub_callback(){
    command_interfaces::msg::ComputerState msg;
    auto current_state=get_computer_state();
    msg.header.frame_id="/computer";
    msg.header.stamp=this->now();
    msg.current_state=current_state.current_state;
    msg.pos1_state=current_state.pos1_state;
    msg.pos2_state=current_state.pos2_state;
    msg.pos3_state=current_state.pos3_state;
    msg.recognition=current_state.recognition;
    computer_state_pub_->publish(msg);
}

PlayerCommandContent Engineering_robot_Controller::get_player_command(){
    std::lock_guard<std::mutex> ul(player_command_mutex);
    return player_command;
}

ComputerState Engineering_robot_Controller::get_computer_state(){
    std::lock_guard<std::mutex> ul(computer_state_mutex);
    return computer_state;
}

void Engineering_robot_Controller::set_regonition_state(int statenum,int state){
    if(state!=REC_SUCCESS&&state!=REC_FAIL){
        RCLCPP_WARN(this->get_logger(),"set_regonition_state error! with state :[%d]",state);
        return;
    }
    std::lock_guard<std::mutex> ul(computer_state_mutex);
    computer_state.recognition=state;
}

void Engineering_robot_Controller::set_computer_state(int statenum,int state){
    if(state<0||state>2){
        RCLCPP_WARN(this->get_logger(),"set_computer_state error! with state :[%d]",state);
        return;
    }
    std::lock_guard<std::mutex> ul(computer_state_mutex);
    switch (statenum){
    case STATE_ONE:
        computer_state.current_state=STATE_ONE;
        computer_state.pos1_state=state;
        computer_state.pos2_state=0;
        computer_state.pos3_state=0;
        break;
    case STATE_TWO:
        computer_state.current_state=STATE_TWO;
        computer_state.pos2_state=state;
        computer_state.pos1_state=0;
        computer_state.pos3_state=0;
        break;
    case STATE_THREE:
        computer_state.current_state=STATE_THREE;
        computer_state.pos3_state=state;
        computer_state.pos1_state=0;
        computer_state.pos2_state=0;
        break;
    case STATE_WAIT:
        computer_state.current_state=STATE_WAIT;
        computer_state.pos1_state=0;
        computer_state.pos2_state=0;
        computer_state.pos3_state=0;
        break;
    case STATE_ERROR:
        computer_state.current_state=STATE_ERROR;
        computer_state.pos1_state=0;
        computer_state.pos2_state=0;
        computer_state.pos3_state=0;
        break;
    default:
        RCLCPP_WARN(this->get_logger(),"set_computer_state error! with statenum :[%d]",statenum);
        break;
    }
}

}// namespace Engineering_robot_Pnx