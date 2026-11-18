# DiPan_Board 当前代码、方向修正和烧录说明

更新日期：2026-11-17。本次直接修改目标工程：/home/wangjiaming/桌面/DiPan_Board。

## 当前状态

已把此前在C_Board_Flash_20261116中完成的方向修正、10秒测试脚本和对应回归测试同步到本工程，并在本工程路径重新编译底盘及云台固件，软件测试全部通过。两份BIN分别与此前验证版逐字节一致。

本次没有烧录、没有访问实物USB/CAN或运行电机；新方向需要烧入底盘C板后现场确认。历史版本和修改前备份均保留。

## 问题和修正位置

协议统一：X向前、Y向左、正yaw为俯视逆时针。测试脚本left/right分别表示左移/右移，ccw/cw分别表示原地逆时针/顺时针，不是同一种运动。

用户反馈left/right和ccw/cw相反。电脑脚本与USB/CAN协议符号一致，云台中转不翻转速度字段；原底盘适配三轴全部设为+1，与当前实物横移和旋转方向不匹配。本次在底盘适配层修正，前后轴保持原值。

配置文件：
/home/wangjiaming/桌面/DiPan_Board/chassis_driver/basic_framework/modules/chassis_comm/chassis_comm_config.h

~~~c
#define CHASSIS_COMM_VX_SIGN 1.0f
#define CHASSIS_COMM_VY_SIGN (-1.0f)
#define CHASSIS_COMM_WZ_SIGN (-1.0f)
~~~

同目录chassis_comm_port.c的命令转换和速度反馈已经共用这组符号，修改配置会同步影响下发轮速参考与上传电脑的实测速度，避免只在测试脚本互换按钮而使导航和里程计不一致。本次没有改轮序、电机ID、PID、CAN协议ID或云台方向逻辑。

此判断是协议与当前驱动/安装方向的适配，不证明电机接线或机械参数一定正确。更换轮位、ID或安装方向后仍需重新验证。

本次同步的4个文件：

- chassis_driver/basic_framework/modules/chassis_comm/chassis_comm_config.h：VY/WZ符号改为-1。
- tests/test_chain.c：六方向命令及反馈符号回归。
- communication_reference/pc/motor_test.py：运动时长支持0.2～10秒，默认仍为1秒。
- tests/test_motor_test.py：完整10秒流程和运行中遥控故障中止测试。

底盘两个运动宏仍为1，遥控许可、四电机在线、会话/序号、TTL、ACK和STOP保护保留。宏置1不代表机械方向已验证；遥控fault=2此前出现过，不能因本次方向修改就认定已修复。

## 两套代码的目录关系

可以放在同一个总文件夹、同一个VS Code工作区，当前已经如此。两块MCU各自独立运行，需要分别编译、分别烧录不同固件；不要混合两套main/启动文件，也不要把同一ELF烧给两块板。

~~~text
/home/wangjiaming/桌面/DiPan_Board/
  chassis_driver/basic_framework/             底盘板工程
  communication_reference/gimbal_firmware/   云台板工程
  communication_reference/pc/                电脑通信和测试脚本
  tools/build_chassis.sh                      底盘编译入口
  tools/build_gimbal.sh                       云台编译入口
  tools/test.sh                               软件测试入口
  artifacts/                                  最新编译产物
  release/方向修正版-20261117/                 本次分板固定快照
  docs/方向修正同步与烧录说明-20261117.md      本文
~~~

communication_reference/chassis_firmware是旧通信参考，不是当前实际驱动；不要把其中旧ELF当成最新底盘程序。旧release子目录是历史版本。

链路：小电脑USB→云台CAN1→底盘CAN2；底盘CAN1→四个C620/M3508。本工程未实现USB/CAN跨板升级引导程序，USB接云台不能自动烧录底盘。

## 本次烧录对应

| 实物板 | 应选文件 | 本次操作 |
|---|---|---|
| 底盘C板，UID末尾31353233 | release/方向修正版-20261117/02_底盘C板_CAN2_方向修正/DiPan_Board.elf | 需要更新方向修正固件 |
| 云台C板，UID末尾36363032 | release/方向修正版-20261117/01_云台C板_USB_CAN1/Gimbal_USB_CAN1.elf | 已是现有配套版时不必重烧 |

最新底盘ELF：
/home/wangjiaming/桌面/DiPan_Board/release/方向修正版-20261117/02_底盘C板_CAN2_方向修正/DiPan_Board.elf

SHA256：9388dafe0ece71279d1898c48af4f43cae73935bef21cf5fa5c27f6360494ad5

底盘BIN SHA256：836afb0bba3b26ecdb8bf9be9435a3619fd0c2cb36b5c613fd7587880e6d752e

配套云台ELF SHA256：cd5f30b41f19a99bc635cd15a90e44fb91da904ed5deaaac6c1d51f46e39da32

ELF、HEX、BIN为同一程序的不同格式，选择一种即可。BIN应用起始地址0x08000000。源码路径影响ELF的调试信息，因此本目录重新编译的底盘ELF与此前C_Board_Flash目录的ELF哈希不同；两份BIN完全一致，程序内容一致。

烧录前断开电机主电源，保留目标板和下载器供电，核对SWD接到的实物板及UID，再选择对应ELF。执行烧写、校验、复位后才算更新；本次只生成文件，未烧录。根目录调试配置如默认指向底盘，不可在SWD接云台时直接按默认任务烧录。

## 编译和验证结果

~~~bash
cd '/home/wangjiaming/桌面/DiPan_Board'
bash tools/build_chassis.sh
bash tools/build_gimbal.sh
bash tools/test.sh
~~~

已在此路径执行并通过：

- 底盘及云台ARM固件编译。
- 7个主机C测试程序，包含锁定/允许运动两种配置、实际通信端口源码和10000次模拟USB/CAN往返。
- 六个方向命令和速度反馈符号检查。
- CAN FIFO回归、4项USB PTY测试、7项电机脚本PTY测试（包含10秒模拟和途中遥控故障中止）。

815个相关C/H/链接/汇编源文件与此前修正版一致，两份BIN与此前验证版一致。日志：/home/wangjiaming/桌面/DiPan_Board/logs/方向修正版编译与软件测试-20261117.log。

测试使用软件模拟，不能替代实物方向、速度比例或导航效果验收。后续重新编译会更新artifacts，不会自动更新本次release快照。

## 烧录后测试指令

先停止其他占用云台通信USB的程序，不需要启动雷达或ROS。四轮架空，确认两板和CAN按原方案连接。

~~~bash
cd '/home/wangjiaming/桌面/DiPan_Board'
ls -l /dev/serial/by-id/
PORT='/dev/serial/by-id/usb-YueLuEmbedded_RM_Nav_Convert_SP_3668306E3533-if00'
# 若应用USB名称不同，按实际列表修改PORT，不选下载器
bash tools/zero_test.sh "$PORT" --duration 10
~~~

STOP测试PASS只说明应答链路通过，不代表运动许可成立。转动前四电机在线、遥控许可稳定、fault持续为0、CH10不在下档。故障恢复后按原规则让CH7或CH8先离开中档，再将二者都置中档。

方向刚修改，先逐条做1秒检查，每次结束后确认轮子响应和停止情况，再进行下一条：

~~~bash
bash tools/motor_test.sh "$PORT" --wheels-raised --direction forward --speed 0.05 --duration 1
bash tools/motor_test.sh "$PORT" --wheels-raised --direction back --speed 0.05 --duration 1
bash tools/motor_test.sh "$PORT" --wheels-raised --direction left --speed 0.05 --duration 1
bash tools/motor_test.sh "$PORT" --wheels-raised --direction right --speed 0.05 --duration 1
bash tools/motor_test.sh "$PORT" --wheels-raised --direction ccw --speed 0.1 --duration 1
bash tools/motor_test.sh "$PORT" --wheels-raised --direction cw --speed 0.1 --duration 1
~~~

确认后，将对应命令改成--duration 10，例如：

~~~bash
bash tools/motor_test.sh "$PORT" --wheels-raised --direction left --speed 0.05 --duration 10
bash tools/motor_test.sh "$PORT" --wheels-raised --direction ccw --speed 0.1 --duration 10
~~~

10秒指运动阶段，另有约1秒STOP预检及0.6秒STOP收尾。Ctrl+C会尝试STOP；若STOP未确认或动作异常，断开电机主电源。STOP应答不证明轮子惯性已完全消失。

## 备份与未完成事项

修改前4个源码/测试文件、README及原artifacts文件备份在：
/home/wangjiaming/桌面/DiPan_Board/logs/同步方向修正前备份-20261117-183016

新增源码与测试修改的diff保存在该备份目录和本次release目录。历史release原样保留。

仍需现场完成：烧入新底盘固件，验证六方向；核对车头、俯视方向、轮位/电机ID；验证轮径/减速比/轴距/轮距等比例、速度反馈、雷达外参与导航TF；继续观察遥控fault=2。以上不由软件编译通过自动证明。
