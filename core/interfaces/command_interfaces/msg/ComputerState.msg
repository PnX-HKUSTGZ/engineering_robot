std_msgs/Header header

uint8 current_state # 上位机当前状态
uint8 recognition # 11 成功，01 识别中，00 失败
uint8 pos1_state # 11 运动完成，01 运动中，00 失败
uint8 pos2_state
uint8 pos3_state