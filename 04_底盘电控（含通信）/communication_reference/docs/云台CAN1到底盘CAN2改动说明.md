# 云台 CAN1 接底盘 CAN2：改动与重烧说明

日期：2026-11-11。当前交付目录：`/home/wangjiaming/桌面/c_board_nav_gateway_20261111`。

## 结论

可以使用 **小电脑 USB → 云台 C 板 CAN1 ↔ 底盘 C 板 CAN2**。CAN1/CAN2 是本地控制器编号，不是线上协议编号，两端编号无需相同。两端均使用经典 CAN、1 Mbps、标准数据帧，DLC=8。

无需重写或重新安装 STM32 HAL 固件库，也无需改电脑 USB 协议。需要把云台导航收发及其邮箱管理从 CAN2 改到 CAN1，重新编译和烧录云台。底盘接 CAN2 的同版 v1 协议继续兼容；如果底盘仍是原哨兵旧协议或此前单板测试固件，需要单独烧入本目录底盘固件。

## 实物接线

| 云台 C 板 | 底盘 C 板 |
|---|---|
| CAN1-H | CAN2-H |
| CAN1-L | CAN2-L |
| GND | GND |

使用板载收发器后的 CANH/CANL 插口，不将 MCU RX/TX 直接互接。按丝印/原理图确认针序；断电后接线，避免把插口中的电源脚误接信号。

两个物理末端各有一只 120Ω 终端；整条总线断电后测 H/L 间约 60Ω 是两只 120Ω 并联的典型情况。两块板正确独立供电，不能通过 CAN 信号线供电。

本工程 MCU 配置为云台 CAN1 PD0/PD1、底盘 CAN2 PB5/PB6（RX/TX）。这只是源码映射，实物仍使用 H/L 端子。此前说明中 PB12/PB13 与本工程不一致，已更正。

## 固件和协议变化

| 内容 | 当前实现 |
|---|---|
| 云台导航注册/收发 | CAN1，接收 0x338～0x33A，发送 0x330～0x332 |
| 底盘导航注册/收发 | CAN2，接收 0x330～0x332，发送 0x338～0x33A |
| 命令内容 | 原 v1 的 session、sequence、TTL、vx/vy/wz、flags 和 CRC |
| 反馈内容 | 原 v1 的 ACK、实测速度、故障位、uptime 和 CRC |
| USB | 原 34 字节协议，route=2，115200 配置不变 |
| route=2 的含义 | 经云台转发，不等于必须使用 CAN2 |
| 云台电机实例 | 改到云台 CAN2，仍禁止执行输出；本次通信测试不用接云台电机 |
| 底盘电机 | 仍使用底盘 CAN1，原四电机 ID 不变 |

将云台 CAN1 专用于板间通信，可避免原云台电机发送队列与导航邮箱共用。之后若恢复云台电机，应核对其 CAN2 接线、ID、关节零位与安装参数；不能沿用“云台电机接 CAN1”的旧接线说明直接启动。

## 具体代码改动

- `gimbal_firmware/application/communication/nav_gateway_port.c`：注册使用 hcan1，导航发送状态绑定 hcan1。
- `common/nav_can_hal.h`：发送状态显式保存 CAN 控制器指针，查询邮箱、发送和超时中止均使用所属控制器，避免只改接收注册却继续从 CAN2 发送。共享代码已同步到两份固件。
- `chassis_firmware/modules/nav_relay/nav_can_adapter.c`：发送状态显式绑定 hcan2，线上协议和接收接口保持兼容。
- `gimbal_firmware/Src/can.c`：CAN1 保持 1 Mbps，关闭自动重传、开启 bus-off 自动恢复。
- 云台 CAN1 SCE 中断入口、NVIC 及 `.ioc` 配置同步更新；没有运行 CubeMX 重新生成工程。
- `gimbal_firmware/application/gimbal/gimbal.c`：保留的两台云台电机实例改用 hcan2，原执行锁仍为关闭状态。
- `tests/test_can_hal.c`：验证 CAN1/CAN2 邮箱独立、过期仅中止所属总线。
- `tests/test_chain.c`：对实际两板适配代码校验云台 CAN1/底盘 CAN2 的注册和发送；保持 STOP、TTL、失联、溢出和默认锁定检查。
- 接线、烧录步骤、协议、README、改动记录、通信 TXT 和构建配置已更新。

## 编译与验证

- `bash tools/test.sh` 通过，日志 `tests-can1-can2.log`。
- 默认锁定和仅供主机测试的放行变体各完成 10,000 次实际适配代码的软件往返。
- 独立 CAN1/CAN2 邮箱超期中止、USB/CRC/序列/TTL 和云台回归测试通过。
- `bash tools/build.sh` 通过，日志 `firmware-sizes-can1-can2.log`、`build-gimbal.log`、`build-chassis.log`。
- 六份 HEX/BIN/ELF 通过 `artifacts/SHA256SUMS` 校验。
- 新 ELF SHA-256：
  - 云台：`abffb6a7dd2e1d47ec5e8eab8cbd39c43f154514d47a91fa9a661d437a4b8bed`。
  - 底盘：`c92d2ce3b71cadf65505825188af7a66d7106c2b716a9267bf9949cf8966c551`。

交付仍为默认关闭运动：`NAV_RELAY_ALLOW_MOTION=0`、`NAV_AXES_VERIFIED=0`、`GIMBAL_INSTALLATION_CONFIRMED=0`、`NAV_JOINT_CALIBRATED=0`。

本次没有改动 USB 编解码器或新导航运行包，因此无需重装电脑软件，也未重复与该接口调整无关的 ROS/PTY 检查。

## 应烧录的文件与当前硬件状态

| 板子 | 文件 | 是否需要重烧 |
|---|---|---|
| 云台 C 板 | `artifacts/gimbal_nav_converter.elf`（或同名 HEX，二选一） | 必须，之前实际烧入的是 CAN2 版 |
| 底盘 C 板 | `artifacts/chassis_nav_can.elf`（或同名 HEX，二选一） | 若已有本项目兼容的 CAN2/v1 固件可继续；旧原程序或未知版本需更新 |

旧 CAN2↔CAN2 固件及此前云台烧录/STOP 日志已保存在 `artifacts/history/gimbal-CAN2-to-chassis-CAN2/`，不应用于新接线。

本次只重新生成固件，**没有把新 CAN1 版写入任何板**。尝试通过 EmberProbe 官方芯片查询脚本确认目标时，返回 `OPENOCD_START_FAILED / CMSIS-DAP command CMD_INFO failed`，没有读到当前芯片 UID，未执行擦除/写入。工作区当前配置显示底盘 ELF 且记录的目标 UID 为 `0032003C3533470536363032`，与此前云台 UID `002A003B3034470F31353233` 不同；这只是软件保存的身份，不能替代本次现场芯片识别。

需要先恢复下载器/无线远端供电与配对，并确认 SWD 所接的是云台板。不要在目标身份未确认时点击下载云台固件。烧录前保持电机主电源断开，并使用现有新版 OpenOCD、MuseLab CMSIS-DAP、STM32F4x、SWD 100 kHz。

新固件烧录后运行 `docs/烧录成功后通信测试指令.txt` 内的 STOP 测试。当前新拓扑的实物双向 ACK 尚未验收；软件测试通过不等于 CAN 接线和实车导航已通过。
