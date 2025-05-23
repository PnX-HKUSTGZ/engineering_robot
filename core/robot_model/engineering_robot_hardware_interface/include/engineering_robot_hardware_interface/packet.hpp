#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
namespace Engineering_robot_Pnx{

#pragma pack(push, 1)
struct NowPosition{
    uint8_t Header = 0xA5;
    uint8_t is_started: 1; // 是否开始任务
    uint8_t is_attach: 1; // 
    uint8_t is_finish: 1; // 是否完成矿石放入矿仓并松开气泵
    uint8_t breakout: 1; // 是否中断
    uint8_t reserved: 4;
    /*
    从0到5依次是
    抬升机构
    J1
    J2
    J3
    J4
    J5
    单位是m或者角度
    */
    float posv[6][2];
    uint16_t crc16 = 0;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SendDate{
    uint8_t Header = 0xA5;
    uint8_t current_state; // 上位机当前状态
    uint8_t recognition:2; // 11: 成功, 01: 识别中, 00: 失败
    uint8_t pos1_state:2; // 11: 运动完成, 01: 运动中, 00: 失败
    uint8_t pos2_state:2;
    uint8_t pos3_state:2;
    /*
    从0到5依次是
    抬升机构
    J1
    J2
    J3
    J4
    J5
    */
    float posv[6][2];
    //对应的速度
    //单位是m或者角速度
    uint16_t crc16 = 0;
};
#pragma pack(pop)

inline void fromVector(const std::vector<uint8_t> & data,NowPosition & packet){
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return;
}

inline std::vector<uint8_t> toVector(const SendDate & data){
  std::vector<uint8_t> packet(sizeof(SendDate));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendDate), packet.begin());
  return packet;
}


#endif  // RM_SERIAL_DRIVER__PACKET_HPP_

}  // namespace Engineering_robot_Pnx