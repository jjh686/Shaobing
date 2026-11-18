# 现用底盘工程（2026-11-18 迁移）

本目录已由桌面 DiPan_Board 完整替换，并在新路径重新构建验证。当前入口见 [迁移与运行说明](docs/迁移与运行说明-20261118.md)。本次未烧录、未驱动实物。

底盘源码、方向修正、遥控 CAN 转发、保护逻辑与 10 秒测试全部保留。配套云台是本目录 communication_reference/gimbal_firmware；03_云台电控（含通信）是另一个独立工程，不能据此认定兼容。原对话中间歇 fault=2 尚未解决。下文保留来源工程历史记录，其中旧绝对路径与旧 ELF 哈希不作为迁移后的构建入口。

---

# DiPan_Board 当前版本（2026-11-17）

**本目录已经同步方向修正和10秒测试脚本，并重新编译、测试通过；本次未烧录、未进行实物转动。**

- 底盘配置：VX_SIGN=+1、VY_SIGN=-1、WZ_SIGN=-1；命令与速度反馈同步转换。
- 当前分板固件：release/方向修正版-20261117/。底盘选择02_底盘C板_CAN2_方向修正/DiPan_Board.elf；云台选择01_云台C板_USB_CAN1/Gimbal_USB_CAN1.elf。
- 这次需更新底盘；云台已是配套版本时无需因方向修正重烧。
- 电脑测试脚本已支持0.2～10秒，默认1秒。首次新方向验证先架空、逐方向1秒，再做10秒。
- 完整说明：当前烧录说明.md，或docs/方向修正同步与烧录说明-20261117.md。
- 两份ARM编译、7个主机C测试、CAN FIFO回归、4个USB PTY、7个电机PTY测试均通过；两份BIN与此前修正版一致。
- 本次未确认实物方向，也未修复此前间歇遥控fault=2；现有保护保留。

## 以下为历史记录

以下内容保留追溯，其中“最新”“当前”等措辞只代表当时状态。当前源码、固件和说明以上方2026-11-17记录为准。

---

# DiPan_Board 底盘通信模块封装

## 最新部署：底盘电机测试版（2026-11-14）

底盘两个运动宏已按用户要求改为1，已烧入编号末尾31353233的底盘C板并自动校验通过。5秒真实STOP测试取得250帧新鲜底盘ACK；遥控CAN转发在线，尚需退出导航档再进入完成许可。四电机断电状态下未测试轮子运动，本轮未烧录云台。

烧录固件保存在 `release/底盘电机测试版-20261114/`；完整操作和转动测试命令见 `docs/底盘电机架空测试与烧录记录-20261114.md` 及同名TXT。先架空、低速、逐方向验证。本段为当前状态，下方“未烧录/运动锁保留”等内容描述历史版本。

## 最新源码：遥控器 CAN 转发版（2026-11-14，未烧录）

本轮新增云台 UART3 SBUS → 云台 CAN1 → 底盘 CAN2 的10路遥控状态转发、CH10下档软件停止、接收机失控及100ms新鲜度门控。底盘不再初始化本地 UART3 遥控器；恢复后须退出导航档再进入。底盘/云台运动锁均保留。

- 最新配套固件：`release/遥控CAN转发版-20261114/`（两块板都要更新，尚未硬件验证）。
- 协议、接线、档位、测试与未确认事项：`docs/遥控CAN转发与接线说明-20261114.md`。
- 下方“实物通信已通过”仅指此前保存的旧配套映像，不能代替本次遥控功能实测。`artifacts/`已更新为本轮构建。

## 2026-11-14 实物通信已通过

已确认此前底盘板内程序与配套底盘 ELF 不一致。重新烧录底盘并自动校验后，USB→云台 CAN1→底盘 CAN2→USB 的 10 秒零速度实测收到 500 帧新鲜底盘 STOP ACK；CAN 错误寄存器镜像为 0。底盘运动锁保留，本次未测试轮子运动或雷达导航。

- 实测配套固件：`release/实物通信通过版-20261114/`，分别按板子子目录使用。
- 当前部署、问题原因、代码改动及日志：`docs/双C板通信实测与修改记录-20261114.md`。
- 实物通过时使用的两板 ELF 均已单独保存在上述实测配套目录；当前 artifacts 是新版待验证固件。

以下为 11 月 12 日的历史接入说明；其中“未烧录/实物待验”的状态以本节和 11 月 14 日记录为准。

更新日期：2026-11-12。全部新内容位于 `/home/wangjiaming/桌面/DiPan_Board`。

## 本次通信修正版（CAN 接收 FIFO 修复）

已在两块板的副本中修复导航三帧组被分配到不同接收 FIFO、延迟处理中断时可能乱序丢组的问题。三帧导航命令和三帧反馈各自统一进入 FIFO0；USB/CAN 线缆协议和 ID 不变。此次修复需要两块板分别更新，不能只更新一块板。

- 分板固件与使用说明：`release/CAN_FIFO修正版-20261112/`。
- 修复原因、实测边界及待确认项：`docs/通信故障修复与修正版说明-20261112.md`。
- 源码仍在下面两份独立副本内；原工程未修改。本次只生成文件，没有烧录。

已将导航通信封装为 `chassis_comm` 模块，接入用户底盘驱动的独立副本。链路为：

```text
小电脑 USB → 云台 C 板 CAN1 ↔ 底盘 C 板 CAN2
                               底盘 C 板 CAN1 ↔ 四个 C620 / M3508
```

## 两份独立副本

| 副本 | 复制来源 | 用途 |
| --- | --- | --- |
| `chassis_driver/` | `/home/wangjiaming/桌面/RM_2026_9_19-sentry` | 本次实际接入通信模块的底盘驱动 |
| `communication_reference/` | `/home/wangjiaming/桌面/c_board_nav_gateway_20261111` | 已有双板通信工程完整副本，提供协议、云台 USB/CAN 中转与电脑测试工具 |

采用本地完整文件复制，无硬链接；来源没有 Git 元数据，因此不是重新从远程下载。两份原工程未改动，逐文件校验见 `logs/source-integrity.json`。电脑 ROS 导航目录也未改动、未复制入本目录。

```text
chassis_driver/basic_framework/modules/chassis_comm/  通信核心、C 板适配、配置、协议
communication_reference/gimbal_firmware/             配套云台工程
communication_reference/chassis_firmware/            旧底盘通信参考，非本次新底盘工程
tools/                                               新的编译与测试入口
tests/                                               实际模块的软件测试
build/chassis/  build/gimbal/                         本次独立构建目录
artifacts/                                           本次固件与 SHA256
docs/  logs/                                         改动记录、指令、检查结果
```

## 编译与测试

```bash
cd /home/wangjiaming/桌面/DiPan_Board
bash tools/build_chassis.sh
bash tools/build_gimbal.sh
bash tools/test.sh
python3 tools/verify_sources.py
```

本机两份 ARM 工程均编译成功，6 项主机测试通过。其中锁定版和仅在主机启用运动的模拟版各完成 10,000 次软件 USB/CAN 往返。结果见 `logs/host-test-results.json`；这些结果不代表实物 CAN、USB、轮子和导航已经验证。

副本自带的 `chassis_driver/build`、`communication_reference/build` 是复制前的旧缓存。使用上面的新入口，不以旧缓存中的 ELF 判断本次结果。`communication_reference/tools/build.sh` 构建的是旧参考工程，不是本次模块化底盘驱动。

## 固件对应板子

| 实物板 | 本次输出 | 说明 |
| --- | --- | --- |
| 底盘 C 板 | `artifacts/DiPan_Board.elf`，同名 `.hex` / `.bin` | 接电机 CAN1、板间 CAN2；部署新模块需要单独烧录到底盘板 |
| 云台 C 板 | `artifacts/Gimbal_USB_CAN1.elf`，同名 `.hex` / `.bin` | 小电脑 USB、板间 CAN1；由复制的兼容云台工程生成 |

两块板分别烧录；BIN 的应用起始地址为 `0x08000000`。本次没有执行烧录。虽然线缆协议兼容，但本次 FIFO 修复同时涉及云台和底盘接收代码，两块板都需要更新为本次修正版。根目录 VS Code/EmberProbe 配置默认指向底盘 ELF；F5 对应底盘目标，不能接着云台板直接运行。

## 当前运行限制

`chassis_comm_config.h` 中 `CHASSIS_COMM_ALLOW_MOTION=0`、`CHASSIS_COMM_AXES_VERIFIED=0`，因此当前底盘固件用于通信验证，默认不使能轮子。遥控器/电机离线会报告故障，仍可验证 STOP/ACK。当前安全门只识别遥控器导航档和失联，不提供摇杆手动驾驶接管。

上车前仍需确认轮径、减速比、轮序、正负方向、接收机格式和导航拨杆。当前仅传输导航速度及状态反馈，没有新增障碍物距离/语义标签载荷；不替代完整云台/发射机构及裁判数据协议。

- 改动、模块接口、协议、待确认项：`docs/底盘通信模块接入与改动说明-20261112.md`
- 编译、分板烧录说明和 STOP 测试命令：`docs/编译烧录与通信测试指令.txt`
- 初始复制清单：`docs/source_copies_manifest.json`
- 最终副本差异清单：`docs/integration_changes.json`
- 模块复用接口：`chassis_driver/basic_framework/modules/chassis_comm/README.md`
