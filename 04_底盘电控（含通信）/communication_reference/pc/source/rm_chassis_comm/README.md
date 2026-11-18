# 哨兵实车通信与云台坐标适配

当前实车入口：`sentry_reality.launch.py`。整体架构、文件职责、标定与验收见 [SENTRY_ARCHITECTURE.md](SENTRY_ARCHITECTURE.md)。

本包已增加46B关节反馈、ROS动态TF及带时间戳的云台系→底盘系速度转换。旧两个括号命名入口用于直接车体系通信测试，默认订阅`/chassis/manual_body_cmd_vel`，不能直接连接北极熊的云台系`cmd_vel`。

日期：2026-11-09。版本：0.2.0。状态：新协议、ROS2上位机与可移植电控封装；构建和测试记录见文末。文档由AI协助整理，电控标定与硬件接入待确认。

用户给出的 `standard_robot_pp_ros2.zip` 属于北极熊 SMBU-PolarBear-Robotics-Team，并非浮舟湿地；与同目录已解压版本的23个非.git文件完全一致。本包参考其 5A 分帧、CRC8/CRC16，重写长度校验、流式解析、超时和收发生命周期。载荷、ID和会话机制为本次新增，不能直接接原固件。

## 两种方案

- [底盘通信（USB直连底盘板）](方案/底盘通信（USB直连底盘板）/接入说明.md)：PC ⇄ USB ⇄ 独立底盘板；提供 `direct_usb.c/.h`。
- [底盘通信（USB经云台CAN转发）](方案/底盘通信（USB经云台CAN转发）/接入说明.md)：PC ⇄ 云台USB网关 ⇄ CAN ⇄ 底盘板；提供 `gimbal_can.c/.h`，同时定义网关角色和底盘角色。

两套共用 ROS 上位机和协议，电控端路径分别实现；安装时用两个括号命名的 launch 入口选择。原 ZIP、云台/底盘固件、自瞄程序、导航和2027决策包均未被自动改写或烧录。

## 依据本队代码需要解决的问题

1. 当前 `电控/云台_USB_SP_2026-10-28/.../basic_framework` 为云台联调版：`GIMBAL_ONLY_BUILD=1`，底盘任务未运行，SP29B仅含云台目标，没有底盘速度。
2. `电控/basic_framework-master/basic_framework-master/basic_framework-master` 有底盘任务与内部 `Chassis_Ctrl_Cmd_s`，但不存在已经接通的导航速度协议。其 NO_FOLLOW 会清wz、FOLLOW会重算wz、ROTATE会生成自旋，须增加保持导航三轴速度的模式。
3. 旧底盘注释x前/y右，并进行云台offset旋转；旧轮速/电机rpm换算也未完整闭合。不能把ROS m/s和rad/s原样塞给旧电机参考值。
4. 旧Seasky和SP均不能直接接新包。旧Seasky解码缺少目标长度保护，不能仅换cmd_id发送三个float。

新包统一使用ROS车体系 x前/y左/wz逆时针，线协议为SI单位。`basic_framework_port.h/.c` 提供默认未验证的单位/符号映射：未标定、配置非法、越界或失能时输出零且不启用导航。比例由电控实测确定，源码中没有猜测的默认“乘1000即可上车”。

## 文件与数据流

```text
Nav2最终cmd_vel / 人工测试Twist
       ↓ 新鲜度、平面约束、限速、显式使能
chassis_node → 34B命令 → 直连底盘 / 云台CAN网关
       ↑                          ↓
诊断、实测速度 ← 34B真实反馈 ← 底盘安全仲裁与TTL
```

- `src/chassis_node.cpp`：ROS2/Linux单线程串口所有者，支持半帧、重连、ACK检查和可选SP代理。
- `src/protocol.c`：显式字节编解码与固定大小解析器，详见 [PROTOCOL.md](PROTOCOL.md)。
- `firmware/mcu_link.c`：电控侧独立序号/会话、控制权和TTL看门狗。
- `firmware/can_transport.c`：经典CAN有界分片与重组。
- `方案/…`：两套MCU接入封装，回调式接入HAL/任务，不依赖ROS。
- `firmware/basic_framework_port.c`：标定后的SI→旧内部单位转换，不包含电机控制。
- `test/`：编解码、MCU状态、CAN/SP分流、单位映射与ROS PTY联调测试。

建议阅读顺序：本说明 → `PROTOCOL.md` → 所选方案的`接入说明.md` → `src/chassis_node.cpp` → 对应方案C文件 → `firmware/mcu_link.c`。云台转发方案再阅读`firmware/can_transport.c`。

## ROS接口

默认命名空间为 `/chassis`：

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| cmd_vel | geometry_msgs/Twist | 车体系三轴速度，输入前须完成控制权仲裁 |
| tx_preview | std_msgs/UInt8MultiArray | 检查本周期底盘命令字节；默认都是STOP |
| measured_velocity | geometry_msgs/TwistStamped | 仅真实反馈MEASURED有效时发布；不发布虚构位置/TF |
| diagnostics | diagnostic_msgs/DiagnosticArray | 连接、使能、route、session、反馈年龄及故障 |
| sp_proxy | std_msgs/String，transient_local | 可选SP虚拟串口路径 |
| set_enabled | std_srvs/SetBool | READY/STOP ACK后启用；启用后还需新cmd_vel |

实车使用`sentry_reality.launch.py`接收北极熊新增的`cmd_vel_gimbal_stamped`，经过`sentry_frame_bridge`转换后才进入串口。旧传输入口默认仅接`/chassis/manual_body_cmd_vel`；浮舟湿地等已有车体系输出也必须先核对坐标，避免重复旋转。只运行一个速度来源/仲裁出口和一个物理串口所有者。

底盘输入限制默认线速度模长1m/s、角速度3.5rad/s；正π的旋转指令可通过，程序不另加旋转。linear.z/angular.x/angular.y非零或任意非有限数会拒绝并清缓存。带时间戳TwistStamped需先转换/校验，不能直接接本Twist接口。

## Ubuntu构建

ROS2 Humble；上位机仅使用标准ROS消息、POSIX串口和PTY，无需北极熊机器人模型或视觉消息依赖。

```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/zbh26/Desktop/code/robomaster/导航
colcon --log-base ~/rm_chassis_ws/log build \
  --base-paths rm_chassis_comm \
  --build-base ~/rm_chassis_ws/build \
  --install-base ~/rm_chassis_ws/install \
  --cmake-args -DBUILD_TESTING=ON
ctest --test-dir ~/rm_chassis_ws/build/rm_chassis_comm --output-on-failure
source ~/rm_chassis_ws/install/local_setup.bash
```

## 启动与核对

两个入口默认 `connect:=false`，只做STOP预览，不打开真实串口：

```bash
ros2 launch rm_chassis_comm '底盘通信（USB直连底盘板）.launch.py'
# 或
ros2 launch rm_chassis_comm '底盘通信（USB经云台CAN转发）.launch.py'
ros2 topic echo /chassis/tx_preview
```

电控已实现本协议、单位和方向完成受控标定后，选择其中一个入口，添加 `connect:=true serial_port:=/dev/实际设备`。不要把旧SP设备当成已升级的底盘接收端。

```bash
ros2 topic echo /chassis/diagnostics
ros2 service call /chassis/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /chassis/set_enabled std_srvs/srv/SetBool '{data: false}'
```

启用仅允许后续新Twist参与发送，先前缓存不会恢复。断线/反馈过期、故障或人工接管会禁用；重新连接后重新STOP握手和显式启用。PC停发、MCU任务独立TTL及电控本地急停共同工作；失能回调必须落实为本车实际停止/制动行为。

## 方案二共享自瞄SP

添加 `sp_proxy_enabled:=true` 后，通过 `/chassis/sp_proxy` 获取 `/dev/pts/N`。把现有SP视觉程序的串口配置为这个PTY，物理云台USB只交给本节点。节点校验29B SP命令与43B反馈CRC，整帧调度；SP反馈拥堵可丢帧并依赖原SP超时恢复，不保证无损或硬实时延迟。

电控侧需用网关的SP完整帧回调接入现有云台处理，并让SP遥测与底盘反馈共用一个有界CDC发送所有者。旧独占USB接收回调/发送任务必须按方案说明改线；本包提供可编译封装，不会自动修改现有固件。SP自身模式、安全许可和超时仍由原云台业务控制。

## 电控实际接入

底盘的固定控制任务持续调用 direct 或 CAN 版本 `poll`，即使没有新数据也要运行；IRQ仅有界入队，避免并发操作同一协议对象。`set_safety` 的ready来自真实导航模式、标定和本地许可，不能由收包成功代替。

建议在底盘工程新增 `CHASSIS_NAVIGATION`：在模式switch中保留接收wz，设置offset_angle=0或直接绕过云台系旋转。最终命令仲裁放在 `RobotCMD` 的 `EmergencyHandler()` 之前，使急停仍最后覆盖；不能被其后的遥控赋值或旧CAN命令再次覆盖。单板使用唯一chassis_cmd发布者；双板也只能由底盘本地仲裁后采用导航输出。

本轮不更改原电控模式/轮速算法，不刷板。电控需完成实际CDC/CAN队列、任务调用、导航模式、轮速/反馈标定和控制权接入。

## 验证记录

已在 Ubuntu-22.04 / ROS2 Humble 完成 Release 构建，5项 CTest 全部通过：协议编解码、MCU会话与超时、CAN/两套封装、单位映射、USB/SP分流。断言在Release构建中保持启用。

ROS PTY端到端联调尚未通过：route=1/2 都能持续发出可解码的STOP帧，但测试端没有收到ROS反馈或诊断。独立的双进程ROS发布/订阅探针同样收不到消息，临时回环及Discovery Server测试也未恢复通信。当前存在跨进程DDS发现/通信障碍；此证据不能替代上位机反馈、使能、限速和SP代理的端到端验收。详见 [VALIDATION.md](VALIDATION.md)。

没有连接真实串口、CAN或电机，没有刷写MCU。CAN实体线、CDC硬件、轮子方向、车辆停止与长时间运行仍需实测。

备份：`C:/Users/zbh26/Desktop/code/robomaster/_backups/版本018_20261108-092016`。本次新增独立 `导航/rm_chassis_comm`，两种方案目录按用户要求用括号区分。
