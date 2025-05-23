# engineering_interfaces

这个仓库用于接口定义，其应该被首先编译，然后由其他项目引用。

## msgs

### ComputerState

用于向下位机传递上位机的状态信息。

定义在 `command_interfaces/msgs/ComputerState.msg` 中。

```
std_msgs/Header header

uint8 current_state # 上位机当前状态
uint8 recognition # 11 成功，01 识别中，00 失败
uint8 pos1_state # 11 运动完成，01 运动中，00 失败
uint8 pos2_state
uint8 pos3_state
```

### PlayerCommand

用于向上位机传递玩家的指令。
定义在 `command_interfaces/msgs/PlayerCommand.msg` 中。
```
std_msgs/Header header

uint8 is_started # 是否开始任务
uint8 is_attach # 是否完成attach
uint8 is_finish # 是否完成兑矿并且松开气泵
uint8 breakout # 是否中断任务

```
