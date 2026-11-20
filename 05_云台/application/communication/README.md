# 云台 C 板通信总入口

应用层通信集中在 `robot_comm.c`，对外接口在 `robot_comm.h`。
控制模块通过接口读命令、设置回传模式，不直接注册 USB 回调或组织通信帧。
板间新通信已独立封装在 `modules/chassis_link/`；模块实现、配置和任务均在该目录。
详见 `modules/chassis_link/README.md`。

## 当前调用关系

```text
RobotCMDInit
  └─ RobotCommInit：初始化唯一的 USB 接收入口并调用 ChassisLinkInit

OSTaskInit
  └─ ChassisLinkStartTask：由板间通信模块自行创建普通优先级1 ms任务

USB 接收回调
  └─ nav_usb_mux 按完整帧分流
      ├─ SP 29字节 → SP解析 → 命令快照、序号、接收时间、会话代数
      └─ 导航34字节 → ChassisLinkReceiveUsb → 模块私有命令队列

ChassisLinkTask → ChassisLinkUpdate（独立普通优先级任务，1 ms 调度）
  ├─ 导航队列 → CAN1 0x330～0x332 → 底盘CAN2
  ├─ UART3 SBUS快照 → CAN1 0x340～0x342（20 ms 周期）
  └─ 底盘CAN反馈 0x338～0x33A → USB导航反馈

RobotCMDTask / AutoAimBootUpdate
  ├─ RobotCommGetVisionCommand：读取一致的命令快照
  ├─ RobotCommUsbIsConfigured：查询 USB 配置状态
  └─ RobotCommSetVisionMode：更新回传模式

INS 任务：INS_Task → GimbalUpdateFeedback → RobotCommUpdate
  └─ 按 VISION_TX_PERIOD_MS 节流 → 姿态快照 → SP 打包 → USB 发送
```

`RobotCommGetVisionCommand` 返回是否收过有效命令；即使命令过期，仍可读取快照。
调用者继续使用接收时间、会话和控制状态执行原有的超时与接管判断。
`RobotCommVisionIsOnline` 同时检查 USB 配置状态和接收超时。
USB会话事件通过 `ChassisLinkResetUsbSession` 通知板间通信模块。
`nav_gateway_port.h` 仅保留旧API兼容转接，实际实现已迁入 `modules/chassis_link/`。

## 本次整理的兼容约定

- SP 指令仍为 29 字节，状态仍为 43 字节，字段和 CRC 算法不变。
- 角度、速度、加速度仍分别使用 rad、rad/s、rad/s²。
- SP 命令经分流后进入原流式解析器；USB会话事件清除残帧、命令有效位及导航会话。
- 姿态发送保留在 INS 更新后，仍按原配置的 5 ms 周期节流。
- `master_process.c/h` 保留原 `Vision…` API 和数据类型，兼容其他代码。
- `sp_protocol.c/h` 继续负责协议编解码；`bsp_usb` 和 `usbd_cdc_if` 负责硬件传输。
- 云台控制参数、限位、遥控/自瞄选择保持原有实现。

## 与04的CAN兼容性（2026-11-20）

基准为 `04_DiPan_Board/communication_reference/gimbal_firmware`，
底盘配套端为 `04_DiPan_Board/chassis_driver/basic_framework`。
板间使用1 Mbps、标准数据帧、每帧8字节、三帧顺序组包；ID、字段、
单位、CRC和会话门控均与04一致。发送时限已从20 ms对齐为8 ms，
接收组帧窗口仍为10 ms；三帧反馈统一进入FIFO0。

CAN1自动bus-off恢复已开启。CAN1还连接yaw电机，因此保留05原有自动重发；
导航/遥控组由8 ms超时主动取消。CAN2的pitch/发射电机总线配置保持原值。
自动重发策略与04有意不同，不改变线协议，实际总线故障恢复仍需硬件验收。

邮箱取消保持硬件所有权直到完成或复用；BSP每次重新分配邮箱时通知网关，
防止导航超时误取消已复用邮箱中的yaw指令。保留05的诊断计数器、
USB未连接时的CAN接收排空，以及独立于USB的遥控转发。
详细对比与测试见 `docs/04_CAN协议对齐与回归验证-20261120.md`。

## 验证

```sh
sh tests/run_all_host_tests.sh
make -j8 all
```

通信测试对旧 `Vision…` 兼容接口和新 `RobotComm…` 接口运行同一组场景：
拆包、噪声/CRC 错帧/粘包、会话变化、超时、姿态字段与回传周期、
无效 IMU、USB BUSY/FAIL、兼容模式/弹速设置和毫秒计时回绕。
新增测试验证SP/导航混合字节流、USB会话重置、与04协议源码的一致性、
真实CAN BSP的电机队列及异常帧拒收、异步邮箱取消和两种配置各10,000次模拟往返。
跨板兼容测试默认读取同级 `04_DiPan_Board`，也可单独执行
`python3 tests/run_can_compat_tests.py --reference /path/to/04_DiPan_Board`。
另有模块任务测试，验证普通优先级、原任务栈、1 ms调度、创建失败可重试及重复启动不会创建第二个任务。
