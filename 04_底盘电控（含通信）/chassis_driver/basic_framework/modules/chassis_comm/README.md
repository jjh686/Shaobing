# chassis_comm 模块

本模块在底盘任务中处理导航 CAN 命令，输出原底盘驱动使用的控制结构，并生成带会话/序号的实测速度反馈。

## 文件职责

- `chassis_comm.c/.h`：平台无关的状态、组包接收、安全门和反馈；每个 `ChassisComm` 实例独立，不直接使用 HAL、RTOS 或 `robot_def.h`。
- `chassis_comm_port.c/.h`：C 板 CAN2、USART3 接收机、中断队列、HAL tick、原驱动结构和单位换算。
- `chassis_comm_config.h`：CAN/UART、轴符号、反馈周期、INS 和运动锁配置。
- `protocol/`：复用双板工程的字节协议与 CAN HAL 邮箱发送辅助；与 `communication_reference/common` 对应文件保持一致。

## 当前工程中的三个接入点

```c
// ChassisInit()，仅调用一次
ChassisCommPortInit();
// ChassisTask() 开始，四轮均有反馈且 daemon 在线时传 1
ChassisCommPortCommand(&chassis_cmd_recv, comm_motors_online);
// ChassisTask() 完成速度估计后
ChassisCommPortFeedback(&chassis_feedback_data, comm_motors_online);
```

port 初始化非重入，只初始化一次。接收回调只复制 CAN 帧；核心解析、安全门和输出由同一底盘任务顺序调用，不要跨多个任务并发操作同一实例。ISR 队列有 16 个槽、可用 15 个；满队列/超龄命令进入 RX 故障，等待新鲜有效 STOP 恢复。

## 迁移到其他驱动

可以保留 `protocol/`、`chassis_comm.c/.h`，替换 port：

1. 实例调用 `ChassisCommInit`。
2. 接收侧先过滤标准 CAN 数据帧，再调用 `ChassisCommReceive`；保留硬件接收时间，不把积压数据标记成当前新数据。
3. 每个控制周期通过 `ChassisCommSetSafety` 更新遥控在线/导航档/电机在线，并调用 `ChassisCommGetCommand`。只有 `enabled=true` 才输出速度，否则停止。
4. 用真实轮速的 SI 速度调用 `ChassisCommMakeFeedback`，按顺序发送生成的三帧。
5. 发生 RX 溢出时调用 `ChassisCommRxFault`，不可丢失安全状态。

`vx/vy` 单位 m/s，`wz` rad/s；+X 前、+Y 左、+yaw 逆时针。核心不检查指针为空，调用方须提供有效对象和缓冲区。纯核心返回的是安全门后的 SI 指令，原驱动 aps 单位换算只在 port。

不要同时编译本模块的协议 `.c` 与旧 `nav_relay` / `nav_conversion` 同名实现，否则会出现重复符号。本底盘 CMake 已递归纳入新模块，无需额外添加源文件。

## 默认值与协议

运动和轴标定锁均为 0；反馈周期 20 ms；CAN2；route=2；遥控器 USART3、CH7+CH8 同时 MID。
命令 ID 0x330/331/332，反馈 0x338/339/33A，1 Mbps，标准数据帧 DLC=8。字段与验收指令见工程根目录 `docs/`。

三帧命令按到达顺序组包，三个接收过滤器必须设为同一个硬件 FIFO。当前 port 使用 `.rx_fifo_policy = CAN_RX_FIFO_POLICY_FIFO0`；云台反馈接收同样固定 FIFO0。不能退回交替 FIFO 的 AUTO 策略，否则延迟处理中断时可能跨 FIFO 乱序。BSP 新增策略字段，默认 0 保持其他注册端点的旧 AUTO 行为。

关闭导航档或遥控失联只会停车，不会自动恢复旧摇杆驱动；后续如需手动驾驶，须单独设计清楚的命令仲裁。
