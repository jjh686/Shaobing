# 当前通信修复（2026-11-20）

现用发布为 `release/通信稳定性修复-05整合-20261120/`。
已合入时间戳竞争、SBUS 流式接收、UART 重挂、USB 队列仲裁和分类诊断；
模块初始化时启动共用 SBUS 接收服务，使上电自瞄模式也能转发底盘遥控许可。
云台控制及 CAN 邮箱保护保留。当前构建入口为 `bash tools/build.sh`，验证入口为
`python3 tools/test_host.py`；配套固件与操作见
[工程归属说明](../../docs/通信稳定性修复与工程归属-20261120.md)。
下文保留之前的模块封装记录，其中“本次”指封装阶段。

---

# 云台板间通信模块

本模块已集成到05云台工程，处理与04配套底盘之间的通信：USB导航指令转CAN1、
UART3 SBUS快照转CAN1、CAN底盘反馈转USB。主函数维持原有入口，通信业务及其任务均封装在模块内。

## 文件分工

| 文件 | 职责 |
|---|---|
| `chassis_link.h` | 对外初始化、任务启动、USB投递、USB会话重置及周期处理接口 |
| `chassis_link.c` | 模块私有队列、导航转发、遥控转发、反馈解码、发送恢复、邮箱所有权管理 |
| `chassis_link_task.c` | 模块私有任务句柄、任务创建和1 ms循环 |
| `chassis_link_config.h` | CAN句柄、队列容量、发送周期、任务周期及栈配置 |
| `chassis_link_debug.h` | 原nav_*诊断符号，保留SWD观察接口 |
| `nav_joint_config.h` | 原有关节未标定配置，保持无效标志 |

线协议继续复用 `modules/nav_conversion/`：CAN ID、分帧、CRC、单位、会话及TTL门控集中在协议层。
原 `application/communication/nav_gateway_port.c` 已迁入本模块；
同目录的 `nav_gateway_port.h` 仅保留旧API到新接口的兼容转接，便于既有调用及04测试继续使用。

## 工程接入

```text
RobotCMDInit
  └─ RobotCommInit
      ├─ ChassisLinkInit                 初始化板间通信、注册CAN反馈
      └─ USBInit                        唯一USB接收入口

USB接收 → RobotComm分流
  ├─ SP帧 → 原自瞄解析与快照
  └─ 导航帧 → ChassisLinkReceiveUsb      只投递，不在中断里转发

USB会话通知 → ChassisLinkResetUsbSession

OSTaskInit
  └─ ChassisLinkStartTask               外层只调用这一处
      └─ 模块内ChassisLinkTask
          └─ ChassisLinkUpdate → osDelay(1)

INS任务 → 原RobotCommUpdate             保持5 ms的SP姿态回传节流
```

初始化顺序保持原样：RobotCMD初始化通信，随后初始化云台，再创建任务。
任务仍使用普通优先级、原512栈参数、1 ms调度；重复调用启动接口不会创建第二个通信任务。
USB只有RobotComm注册的一个回调，SP自瞄仍按原路径处理。

## 保留的运行约定

- 云台CAN1 PD0/PD1 ↔ 底盘CAN2 PB5/PB6，1 Mbps；CAN1同时连接yaw，保留原电机自动重发设置。
- 导航命令0x330～0x332、底盘反馈0x338～0x33A、遥控0x340～0x342；标准数据帧，每帧8字节，三帧组包。
- 发送时限8 ms、组帧窗口10 ms、遥控20 ms周期及100 ms新鲜度；保留源年龄、故障代数、CH10急停和重新拨档要求。
- USB断开时仍转发遥控并排空CAN反馈；USB会话重置清除导航队列及许可状态。
- 保留BSP邮箱复用通知，防止导航取消误伤已复用邮箱中的yaw指令。
- 云台控制、PID、限位、电机驱动、SP协议、USB底层和04源码均未因本次封装更改。

## 构建与验证

从05工程目录执行：

```sh
sh tests/run_all_host_tests.sh
make -j4 all
```

Make已经加入模块两个C文件和头文件路径；CMake按现有规则自动扫描新模块。
本机旧版ARM binutils不支持原有的 `--no-warn-rwx-segments` 选项，已移除该警告屏蔽选项以完成CMake链接；
该调整不改变云台控制或通信配置。

主机回归通过：原云台控制、DM4310、SP通信、混合USB分流、CAN协议及异常处理、
真实BSP电机队列、模块任务生命周期，以及真实05模块与04底盘端口在两种配置下各10,000次模拟往返。
88个受保护文件的SHA-256与封装前一致，覆盖主函数、FreeRTOS入口、云台控制、
电机驱动、SP编解码、USB底层、CAN BSP及已对齐的协议层。

Make默认构建产物已更新到 `05_云台/build/basic_framework.elf` 及同名BIN/HEX。
CMake构建在 `/tmp/gimbal-chassis-link-cmake/` 验证通过。新固件尚未烧录，实物运行仍需验证。

本次发布快照位于仓库的 `05_云台/release/云台通信模块版-20261120/`，
固件放在 `01_云台C板_USB_CAN1/` 中，包含 `Gimbal_ChassisLink.elf/.hex/.bin`；
版本根目录保存SHA-256校验、构建/测试记录和烧录说明。
配套底盘固件位于 `04_底盘电控（含通信）/chassis_driver/basic_framework/release/通信稳定性修复-20261120-185644/`。

后续实物通信仍使用04原来的 `tools/zero_test.sh` 和 `tools/motor_test.sh`，
云台板选择本次05构建产物，底盘继续使用04配套固件。
