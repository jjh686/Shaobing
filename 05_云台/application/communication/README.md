# 云台 C 板通信总入口

应用层通信集中在 `robot_comm.c`，对外接口在 `robot_comm.h`。
控制模块通过接口读命令、设置回传模式，不直接注册 USB 回调或组织通信帧。

## 当前调用关系

```text
RobotCMDInit
  └─ RobotCommInit：初始化唯一的 USB 接收入口

USB 接收回调
  └─ SP 解析 → 命令快照、序号、接收时间、会话代数

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

## 本次整理的兼容约定

- SP 指令仍为 29 字节，状态仍为 43 字节，字段和 CRC 算法不变。
- 角度、速度、加速度仍分别使用 rad、rad/s、rad/s²。
- USB 接收仍直接进入原流式解析器；断线重新枚举时清除残帧和命令有效位。
- 姿态发送保留在 INS 更新后，仍按原配置的 5 ms 周期节流；未新增任务。
- `master_process.c/h` 保留原 `Vision…` API 和数据类型，兼容其他代码。
- `sp_protocol.c/h` 继续负责协议编解码；`bsp_usb` 和 `usbd_cdc_if` 负责硬件传输。
- CAN 驱动、总线配置、云台控制参数、限位、遥控/自瞄选择不因本次整理改变。

## 后续合并导航与底盘代码

先约定导航物理接口、消息字段/单位/坐标系、CAN ID、发送周期和超时行为，
再从通信总模块接入专用导航链路、底盘链路和路由处理。
当前只提供统一的现有通信入口，尚未实现导航或底盘中转。

USB 当前只有一个接收回调和固定 SP 协议。如果导航共用 USB，需由统一入口
分发消息，并同步约定上位机协议，不能再次注册接收回调或直接拼接导航数据。

导航和 CAN 中转应使用合适的任务调度；不要把额外转发、阻塞等待或日志处理
加入当前高优先级 INS 回传路径。需要转发的状态应携带接收时间和序号，任务间
通过一致快照或队列传递，过期运动指令不能靠重复转发维持“在线”。

## 验证

```sh
sh tests/run_all_host_tests.sh
make -j8 all
```

通信测试对旧 `Vision…` 兼容接口和新 `RobotComm…` 接口运行同一组场景：
拆包、噪声/CRC 错帧/粘包、会话变化、超时、姿态字段与回传周期、
无效 IMU、USB BUSY/FAIL、兼容模式/弹速设置和毫秒计时回绕。
