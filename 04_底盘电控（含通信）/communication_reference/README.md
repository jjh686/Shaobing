# 新导航 USB → 云台 C 板 → CAN → 底盘 C 板

已按此前配置的新导航包 `rm_nav_source_20261110(zbh)` 接入。此目录包含电脑端通信代码快照、两块 C 板的独立源码、编译固件、测试和说明。原来的三个输入工程未被覆盖。

当前接线：**小电脑 USB → 云台 C 板 CAN1 ↔ 底盘 C 板 CAN2**，两端 CAN 均为 1 Mbps。

云台板会解析 USB 导航帧中的速度、会话、序列号和有效期，换算后生成 CAN 消息；底盘回传确认、实测速度和故障信息。导航速度采用 ROS 车体系：X 向前、Y 向左、逆时针为正。

**交付固件默认关闭底盘和云台电机运动输出，用于先验证通信。两块板需分别烧录；实际烧录与接口调整状态见本次 docs 记录，尚未驱动实车。当前选择底盘固定雷达模式，云台板只做中转；这个模式无需云台关节标定。**

本车当前入口与改动：[底盘固定雷达导航部署](../rm_nav_source_20261110(zbh)/底盘固定雷达导航部署与待确认-20261112.md)、[启动指令 TXT](../rm_nav_source_20261110(zbh)/底盘导航启动与联调指令-20261112.txt)。

2026-11-11 23:34 已收到250帧真实新鲜底盘 STOP ACK（fault=15，仍禁止运动）；此前0ACK是历史结果。本轮适配不执行烧录，未读回确认板内镜像哈希。

先读以下文档：

- [云台固件识别修复与独立烧录包](docs/云台ELF识别修复-20261111.md)
- [接线、烧录与启动操作](docs/接线烧录与测试.md)
- [两块 C 板分别烧录的逐步操作](docs/两块C板分别烧录操作.md)
- [烧录成功后的通信测试指令（TXT）](docs/烧录成功后通信测试指令.txt)
- [云台 CAN1 ↔ 底盘 CAN2 接口调整](docs/云台CAN1到底盘CAN2改动说明.md)
- [USB 与 CAN 协议](docs/通信协议.md)
- [修改记录、测试结果和待确认项](docs/改动与待确认.md)
- [EmberProbe CMD_INFO 烧录报错排查](docs/烧录报错CMD_INFO排查.md)

## 固件

| 烧入对象 | HEX 固件 | 源码入口 |
|---|---|---|
| 云台 C 板 | `artifacts/gimbal_nav_converter.hex` | `gimbal_firmware/application/communication/nav_gateway_port.c` |
| 底盘 C 板 | `artifacts/chassis_nav_can.hex` | `chassis_firmware/modules/nav_relay/nav_can_adapter.c` |

`artifacts/` 同时提供 BIN、ELF 和 `SHA256SUMS`。两块板不能互换固件。

只打开 `gimbal_firmware/` 时，EmberProbe 选择其内部的 `firmware_release/gimbal_nav_converter.elf`。这是从 `artifacts/` 同步的云台固件，避免访问工作区外路径。`tools/build.sh` 会自动刷新该目录及 `SHA256SUMS`。

## 电脑启动

查看板载 USB 设备的稳定路径：

```bash
ls -l /dev/serial/by-id/
```

烧录两块板、完成接线后，将下列 `实际云台板设备名` 替换成上一步的真实设备名。零速度检测会核对 USB 产品名，并且只发送 STOP：

```bash
bash /home/wangjiaming/桌面/c_board_nav_gateway_20261111/pc/run.sh zero /dev/serial/by-id/实际云台板设备名
```

零速度检测退出后，再启动 ROS 通信：

```bash
bash /home/wangjiaming/桌面/c_board_nav_gateway_20261111/pc/run.sh comm /dev/serial/by-id/实际云台板设备名
```

不连接硬件，检查电脑端节点能否启动：

```bash
bash /home/wangjiaming/桌面/c_board_nav_gateway_20261111/pc/run.sh preview
```

当前底盘固定雷达入口：`bash pc/run.sh chassis_navigation /dev/serial/by-id/实际云台板设备名 use_rviz:=true`。仍默认禁止使能；原 `pc/run.sh navigation` 是完整云台方案。

## 重新编译与测试

```bash
cd /home/wangjiaming/桌面/c_board_nav_gateway_20261111
bash tools/test.sh
bash tools/build.sh
```

`common/` 是 MCU 共享协议代码的维护入口。构建脚本会把它同步到两份固件，再编译并更新固件校验文件。
