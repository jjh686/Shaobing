# 遥控 CAN 转发、停止与失控保护（2026-11-14）

## 当前状态

已修改独立副本 DiPan_Board 内两端源码，生成两份配套固件，未烧录、未做本轮实物接收机/遥控CAN测试。此前成功的500帧STOP ACK属于旧固件，不能证明本轮新增遥控链路已通过。
底盘 CHASSIS_COMM_ALLOW_MOTION=0、CHASSIS_COMM_AXES_VERIFIED=0；云台 GIMBAL_INSTALLATION_CONFIRMED=0 均保留。因此接电机、收到导航速度也不使能运动。本轮不启用云台/发射机构。

两份固件放在 release/遥控CAN转发版-20261114，必须分别烧录：

| 对象 | 文件 |
|---|---|
| 云台 C 板：USB中转、UART3 SBUS接收、CAN1发送 | 云台C板/Gimbal_USB_CAN1.elf |
| 底盘 C 板：CAN2接收遥控与导航，CAN1电机 | 底盘C板/DiPan_Board.elf |

同时提供HEX/BIN和SHA256SUMS；BIN应用地址0x08000000。不使用 communication_reference/chassis_firmware 中的旧参考固件。旧实物通信通过版保留，便于回退；本轮不要与旧版混用。

## 实物接线

断电、断开电机主电源后调整接线。

接收机 → 云台 C 板：

| 接收机引脚功能 | 云台接口 |
|---|---|
| − / GND | DBUS A8（GND） |
| ＋ / 正电源 | DBUS B8（5V） |
| S.BUS 输出信号 | DBUS C8（信号） |

根据本地《RoboMaster 开发板 C 型用户手册》纸面第13页：DBUS经板载反相电路接UART3_RX，A8地、B8 5V、C8信号。接口和PWM共用长排针，但必须插DBUS端的三针，不能任意插PWM通道。
按用户照片中下方C板的方向（USB在下、DBUS在右侧靠下），DBUS这一排从照片左到右是信号、5V、GND；仍以接口位置和手册编号复核。不要接到旁边单独的5V两针接口。
接收机为照片中的WFLY R206S，外壳有PWM/PPM/S.BUS标记。必须使用S.BUS模式下的复用输出，而非任意普通PWM通道；照片不能可靠确定接收机端三根杜邦线各自功能，按接收机壳体“−/＋/信号”标记或型号手册核对，不按线色推断。若接收机已另外供电，不再并接C板5V输出；仍共地并连接信号。
用户称此前已使用成功，本轮保留原S.BUS设置；具体遥控器型号、输出通道和开关对应关系仍未确认。

板间与上位机：

- 小电脑USB数据线 → 云台C板自身Micro-USB。
- 云台CAN1 H → 底盘CAN2 H；云台CAN1 L → 底盘CAN2 L；两板共地。
- 底盘CAN1 → C620电调总线，仍按ID1/2/4/3配置。
- 接收机只需接云台；新版底盘不初始化本地UART3遥控接收，避免两个遥控来源互相覆盖。
- CAN2接口中带的电源针不是CAN信号，勿凭线色或照片左右位置接线；24V不得接入DBUS、5V或CAN信号针。

## 档位和恢复行为

本轮沿用已有CH7/CH8导航定义，并增加原云台代码使用的CH10下档停止：

- CH7与CH8均中档：请求导航。
- CH10下档：软件停止，底盘零力；释放CH10不会自动恢复导航。
- 遥控器关闭、SBUS failsafe/frame-lost/无效帧、无有效源数据超过100ms、CAN转发超时、队列溢出或相关帧格式错误：底盘不允许导航输出。
- 上电、失控、停止、协议错误或中转重启恢复后：先把CH7或CH8移出中档，收到有效非导航样本，再把CH7和CH8都拨回中档。
- PC导航仍要求独立STOP会话建立、后续ENABLE、新鲜TTL、四电机反馈在线；遥控许可不会覆盖这些门控。
- 已传输六个模拟通道和四个开关通道，但没有新增摇杆手动驾驶混控。退出导航为零力，不是自动进入手动驾驶。
- CH10是软件停止，不是独立硬件断电急停，不代替电机主电源开关。

R206S外壳“6CH”不能证明S.BUS数据中CH7、CH8、CH10可由遥控器操纵。必须读取真实通道再确认。若不支持，需按遥控器型号另定导航与停止通道；本次不会为了通过门控而伪造额外通道或取消停止。

## 线缆协议

CAN1→CAN2，1Mbps，标准数据帧，DLC8，ID依次0x340、0x341、0x342。每20ms尝试发送一组三帧，三帧组窗口10ms，硬件发送超时8ms；共24字节，小端。与原导航0x330~332、反馈0x338~33A以及电容0x301/302无冲突。

| 组内字节 | 含义 |
|---|---|
| 0 | 协议版本1 |
| 1 | bit0源在线；bit1停止/失控；其余必须0 |
| 2~3 | 16位CAN组序号，允许自然回绕 |
| 4~5 | SBUS故障代数低16位；即使短故障发生在两次转发间，也使接收端解除导航许可 |
| 6~7 | 源最后有效SBUS帧年龄，毫秒，饱和65535；不使用CAN轮询时间伪造源心跳 |
| 8~15 | CH1右水平、CH2右竖直、CH3左竖直、CH4左水平，int16，992中位、死区15、限幅±660 |
| 16~19 | CH5/CH6原始11位SBUS通道0~2047（还原原解析器的-240偏移） |
| 20 | CH7~10各2bit：0上、1中、2下；值3无效 |
| 21 | 保留，必须0 |
| 22~23 | 字节0~21的CRC16，复用nc_crc，初值0xffff、多项式反射0x8408 |

接收端拒绝损坏、乱序、缺帧、超时、非法通道/版本/保留位；重复或后退序号不能刷新有效数据。转发中断超过100ms后可重新建立序号基线，但仍须退出导航档再进入。CRC用于检错，不是防恶意注入认证。
目标端将源年龄加上本地排队/组包耗时，后续按本地时间递增；超过100ms失效，不会因持续转发旧SBUS快照而一直保持在线。任务周期/调度会带来约一个底盘任务周期的额外停止延迟，尚无实车时间测量。
云台遥控转发不依赖USB是否配置；PC断开由原导航TTL与会话机制停止底盘，同时遥控状态仍可继续传输。

## 修改位置

- communication_reference/common/remote_can.h：遥控线缆编解码、序号、新鲜度、恢复门控的公共实现；两板各有同内容副本，主机测试检查三份一致。
- communication_reference/gimbal_firmware/modules/nav_conversion/remote_can.h：云台协议副本。
- chassis_driver/basic_framework/modules/chassis_comm/protocol/remote_can.h：底盘协议副本。
- 云台 application/communication/nav_gateway_port.c：独立遥控发送器，20ms周期、源年龄、故障代数、在线/停止标志；即使USB未连接仍运行。
- 云台 modules/remote/remote_control.h/.c：一致快照增加真实接收时间；沿用SBUS故障代数。
- 云台 application/cmd/robot_cmd.c：上一轮已改为始终初始化UART3 SBUS。
- 云台 Src/usart.c及对应.ioc：标准SBUS 100000、8个有效数据位、偶校验、2停止位；HAL字长9B含校验。DBUS口使用板载反相，不再外加反相器。
- 底盘 modules/chassis_comm/chassis_comm_port.c：新增0x340~342 FIFO0接收注册，改为CAN遥控门控，故障/恢复逐次应用，溢出同时撤销遥控许可。
- 底盘 chassis_comm_config.h：明确遥控来源为CAN，删除本模块本地UART3配置。
- tests/test_remote_can.c：编解码、CRC/时序/序号/源年龄、停止与重新允许的测试。
- tests/test_chain.c和遥控桩：通过实际两端适配代码验证整链路；接收机数据/外设由主机模拟。
- tools/test_host.py：新增协议测试和三份头文件一致性检查。

修改前备份：logs/before-remote-can-20261114.tar.gz；上一轮SBUS初始化前备份另在logs/remote-before-20261114/。

## 编译、测试及实物验证顺序

```bash
cd /home/wangjiaming/桌面/DiPan_Board
bash tools/test.sh
bash tools/build_gimbal.sh
bash tools/build_chassis.sh
```

软件测试通过：7个主机可执行测试；运动锁定/仅主机启用运动的整链路模拟各10000次；双FIFO历史乱序故障复现与修正对照；4个Python USB诊断测试。主机允许运动宏不用于ARM固件。
两份ARM固件编译通过，仍保留运动锁。本轮未访问硬件、未烧录、未发送运动命令。

实物验证：

1. 电机主电源断开，按板分别烧入本目录两份新固件并校验，复位运行。
2. 保持接收机原S.BUS模式，核对供电与三针连接并完成对频。
3. 先观察云台nav_rc_online、nav_rc_age_ms、nav_rc_estop以及底盘chassis_rc_channels，确认CH1~10真实变化，核对导航和停止档位；未经检查不解除运动锁。
4. CH7或CH8退出中档，再全部拨回中档，检查底盘chassis_rc_navigation是否按预期变化。CH10下档必须清零许可，释放后必须重新切档才能恢复。
5. 关遥控器、拔接收机信号、断开板间CAN各测试一次：底盘遥控许可应撤销，恢复连接不能自动许可。核对断线过程中的真实延迟，不以灯亮判断数据有效。
6. 电脑做禁用STOP通信测试（不需要雷达）：

```bash
bash tools/zero_test.sh /dev/serial/by-id/usb-YueLuEmbedded_RM_Nav_Convert_SP_3668306E3533-if00 --duration 10
```

如果USB序列号变化，用实际/dev/serial/by-id/路径。未接电机、运动锁或遥控未许可会报告故障；STOP ACK用于证明链路，不代表允许运动。

可观察的SRAM变量（需与当前烧录ELF一致）：

- 云台：nav_rc_groups、nav_rc_online、nav_rc_estop、nav_rc_age_ms。
- 底盘：chassis_rc_accepted、chassis_rc_rejected、chassis_rc_online、chassis_rc_navigation、chassis_rc_estop、chassis_rc_channels[10]。
- USB反馈仍使用原NC_FAULT_RC表示遥控/导航许可问题，本轮没有扩展USB遥控原始通道消息。

## 尚待确认

- 遥控器型号及R206S在S.BUS模式下实际可用通道、开关映射和失控输出；此前使用成功不等于当前10通道/停止通道映射已确认。
- 接收机端物理S.BUS输出口和各针功能不能仅凭照片中线色认定；需壳体标记/型号手册或测量确认。
- UART从原1停止位改为标准2停止位，真实帧接收仍待验证；如实物输出不是标准反相SBUS，应据手册/波形适配而非任意交换电源线。
- 这套固件不含图中完整云台DM电机/发射任务，Yaw仍在云台CAN2；本轮只实现遥控状态和导航中转。
- 当前无人工摇杆底盘驾驶；轮序、方向、几何、运动控制与雷达导航仍未实车验证。
