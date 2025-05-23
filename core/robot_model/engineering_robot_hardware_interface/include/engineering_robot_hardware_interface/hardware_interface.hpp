#include "crc.hpp"
#include "packet.hpp"
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <serial_driver/serial_driver.hpp>
#include <iostream>
#include <thread>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <command_interfaces/msg/computer_state.hpp>
#include <command_interfaces/msg/player_command.hpp>

#ifndef engineering_robot_hardware_interface
#define engineering_robot_hardware_interface

namespace Engineering_robot_Pnx{

using hardware_interface::HardwareInfo;
using CallbackReturn=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;
using  hardware_interface::return_type;
using FlowControl = drivers::serial_driver::FlowControl;
using Parity = drivers::serial_driver::Parity;
using StopBits = drivers::serial_driver::StopBits;

struct PlayerCommandContent{
    rclcpp::Time command_time=rclcpp::Time(0,0);
    bool is_started=0;
    bool is_finish=0;
    bool is_attach=0;
    bool breakout=0;
};

struct ComputerState{
    uint8_t current_state;
    uint8_t recognition:2;
    uint8_t pos1_state:2;
    uint8_t pos2_state:2;
    uint8_t pos3_state:2;
};

class ERHardwareInterface : public hardware_interface::SystemInterface{

public:

// no content
void on_configure();
// no content
void on_cleanup();
// try reload
void on_shutdown();

// no content
void on_activate();
// no content
void on_deactivate();
// try reload
void on_error();

void reopen();
void open();

CallbackReturn on_init(const HardwareInfo & hardware_info) override;
std::vector<StateInterface> export_state_interfaces() override;
std::vector<CommandInterface> export_command_interfaces() override;
return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

rclcpp::Logger logger=rclcpp::get_logger("RHardwareInterface");
std::unique_ptr<IoContext> owned_ctx_;
std::string device_name_="/dev/ttyACM0";
// 顺序和串口包的顺序一样
std::vector<std::string> joint_name={"J1J","J2J","J3J","J4J","J5_xJ","J5_zJ"};
std::vector<std::string> interface_name={"position","velocity"};
std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

// get from read
double state_data[6];
// get from read
double v[6];
// 位置
double WID_p[6];
// 速度
double WID_v[6];

uint32_t baud_rate=115200;
FlowControl fc = FlowControl::NONE;
Parity pt = Parity::NONE;
StopBits sb = StopBits::ONE;

private:

std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> single_executor_;
std::shared_ptr<std::thread> single_executor_thread_;
std::shared_ptr<rclcpp::Node> node_;
rclcpp::Subscription<command_interfaces::msg::ComputerState>::SharedPtr computer_state_sub_;
rclcpp::Publisher<command_interfaces::msg::PlayerCommand>::SharedPtr player_command_pub_;

PlayerCommandContent player_command;
std::mutex player_command_mutex;

ComputerState computer_state;
std::mutex computer_state_mutex;

void computer_state_sub_callback(command_interfaces::msg::ComputerState::ConstSharedPtr);
PlayerCommandContent get_player_command();
ComputerState get_computer_state();
void set_player_command(const PlayerCommandContent & input_command);
void set_computer_state(const ComputerState & input_state);

};// ERHardwareInterface

}//Engineering_robot_Pnx

#endif  //engineering_robot_hardware_interface