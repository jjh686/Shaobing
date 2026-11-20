# 05云台通信模块版固件

版本：2026-11-20。源码来自05_云台，已集成独立chassis_link模块并对齐04板间协议。

## 烧录对象与文件

`01_云台C板_USB_CAN1/` 中固件烧入云台C板（STM32F407）；历史记录中云台板UID末尾为36363032，实际烧录前核对目标板。

| 文件 | 用途 |
|---|---|
| 01_云台C板_USB_CAN1/Gimbal_ChassisLink.elf | 推荐烧录文件，包含地址和调试信息 |
| 01_云台C板_USB_CAN1/Gimbal_ChassisLink.hex | 同一程序的HEX格式，包含烧录地址 |
| 01_云台C板_USB_CAN1/Gimbal_ChassisLink.bin | 同一程序的BIN格式，起始地址0x08000000 |

三种格式任选其一；板卡子目录中的文件逐字节复制自05_云台/build，并已从ELF重新导出BIN/HEX核对一致。
本次只整理固件，尚未烧录或进行实物通信测试。

配套底盘固件：
[DiPan_Board.elf](../../../04_DiPan_Board/release/底盘配套版-20261120/02_底盘C板_CAN2_方向修正/DiPan_Board.elf)。
该底盘包保留11月17日方向修正版原始内容，已核对与当前底盘源码一致。
底盘已是配套版本时，本次只需更新云台固件。两块板分别选择各自文件。

## 本版内容

- 板间通信封装在05_云台/modules/chassis_link/，独立普通优先级1 ms任务。
- CAN1↔底盘CAN2，1 Mbps；导航0x330～0x332，反馈0x338～0x33A，遥控0x340～0x342。
- 发送超时8 ms，组帧窗口10 ms；保留遥控源年龄、失控/急停、USB会话和STOP门控。
- 保留原云台控制、PID、限位、SP自瞄及电机自动重发；修复邮箱复用时可能误取消yaw报文的问题。

## 校验与验证

在本目录执行：

```sh
sha256sum -c SHA256SUMS
```

原云台控制、DM4310、SP通信和7项CAN兼容程序通过；真实05通信模块与04底盘端口在两种配置下各完成10,000次模拟往返。
Make和CMake构建均通过；本目录选用05默认Make构建产物。
软件测试与编译结果不代替实物CAN拥塞、故障恢复和云台动态响应验收。

详细记录：构建记录.json、源文件校验.json、编译日志.log、软件测试.log。
源码包含本次尚未提交的修改；构建记录中的Git提交号需结合源文件校验理解。

## 通信测试

接线照旧：电脑USB→云台，云台CAN1↔底盘CAN2，H-H、L-L及共地。
烧录后继续使用04原tools/zero_test.sh；停止其他占用同一USB的程序后执行：

```sh
cd '/home/wangjiaming/文档/GitHub/RM_11_13/04_DiPan_Board'
PORT='/dev/serial/by-id/实际云台应用USB设备名'
bash tools/zero_test.sh "$PORT" --duration 10
```

预期输出新鲜底盘STOP ACK的PASS。后续轮子测试仍按原架空、遥控许可和电机在线流程进行。
