# GM6020 Yaw + DM4310 Pitch 适配说明

基线：用户提供的《整理好6020电机.zip》，内部工程 `basic_framework_tracking_v1`。本版仅替换 pitch 执行器及其接入代码。适配日期：2026-11-15。

## 保留的控制与通信行为

- yaw 继续使用原 GM6020 驱动；两轴位置环、速度环继续使用 IMU 的 rad、rad/s 反馈。
- 保留安装旋转矩阵、yaw/pitch 正方向、连续 yaw、视觉绝对目标、启动相对机械限位及原方向宏。pitch 机械包络仍为 ±20°。
- 保留原包实际生效的两轴位置环 Kp=200 覆盖。配置中的 8/10 宏不是该包启动后实际 Kp，本次未清除这段原有实验设置。
- 保留原速度 PI、积分限幅、TRACKING/HOLD 切换时 pitch 保留负载积分的行为。未加入未经机构测量的重力补偿。
- 保留 29 字节 SP 指令、43 字节状态回传、小端 float32、`wxyz` 四元数和 CRC16。通信源码、姿态转换源码与基线相同。
- 保留原包自主启动流程：`GIMBAL_AUTO_AIM_ON_BOOT=1`、`GIMBAL_INSTALLATION_CONFIRMED=1`，健康硬件及连续有效指令满足条件后才进入控制；视觉无目标时保持最后实测姿态。

## 达妙参数与接线

默认 pitch 仍接 C 板 CAN1；yaw ID1 保持原接线。达妙使用独立 MIT 报文，不能再接回原 pitch 6020 并期待本固件控制它。

| 项目 | 本版配置 | 对应位置 |
| --- | --- | --- |
| 控制模式 | MIT，Kp=0、Kd=0，仅发送 t_ff | 专用 `dm4310_pitch.c` |
| Motor CAN ID | `0x002` | `GIMBAL_PITCH_DM_CAN_ID` |
| Master ID / 反馈 CAN ID | `0x000` | `GIMBAL_PITCH_DM_MASTER_ID` |
| CAN 波特率 | 1 Mbps，原 CAN 配置不变 | `Src/can.c` |
| P_MAX | 3.14159 rad | `GIMBAL_PITCH_DM_P_MAX` |
| V_MAX | 30 rad/s | `GIMBAL_PITCH_DM_V_MAX` |
| T_MAX | 10 N·m，协议编码范围 | `GIMBAL_PITCH_DM_T_MAX` |
| 软件力矩限幅 | ±3 N·m | `GIMBAL_PITCH_DM_TORQUE_LIMIT_NM` |

ID、量程依据本次用户提供的调试助手截图；这些参数必须与电机保存的参数一致。代码不会自动写电机参数、重设编码器零点或自动清除硬件故障。电机端 CAN Timeout 图示为 4000；如该固件单位为 50 µs，则对应 200 ms。此值不是 C 板的软件反馈超时：本版仍保留电机反馈 50 ms、IMU 20 ms、上层目标 30 ms 的检查。

达妙反馈位置、速度按输出轴 rad、rad/s 解析，不额外除以内部减速比 10。仅机械包络监视将位置转换成度，仍锚定上电后第一帧真实位置；控制反馈不切换成电机编码器。本版按电机输出轴与 pitch 轴直接 1:1 连接处理，存在额外传动比时需另行标定机械包络映射。

电机调试助手中的电机本体参数、编码器校准和 `KT_OUT` 应按实物型号及固件正确配置、读回确认；截图中的 `KT_OUT=0` 不作为本版的力矩标定依据。

## 力矩换算与调参边界

原 GM6020 的速度 PI 输出为 DJI 命令单位，不能直接作为 N·m 发给 4310。本版保留原 PI 数值体系，在驱动输出末端转换：

```c
torque_nm = controller_output * GIMBAL_PITCH_DM_NM_PER_COMMAND;
```

初始比例为 `3.0f / 20000.0f`，即原输出上限 20000 对应 3 N·m，原 pitch 积分上限 5000 对应 0.75 N·m。原输出方向宏在末端应用一次，限幅后才进行 MIT 编码。12 位力矩编码有量化误差，零命令落在零附近的一个量化格内。

这个比例是新执行器的初始增益，不是测得的 6020 电压命令与 DM 力矩等效关系。更换电机、减速器、负载后，同样的软件环路不能单凭编译测试保证相同的带宽和阻尼。装机时先确认小角度正负响应，再单独调整该比例；必要时再调整 pitch PI，yaw 参数无需随之修改。比例过大会增加振荡和冲击，过小会出现响应慢、负载支撑不足。初测可临时降低软件力矩限幅，确认方向后再逐步恢复。

软件 3 N·m 限幅与 T_MAX=10 的含义不同。厂商 DM-J4310-2EC V1.1 产品页列额定 3 N·m、峰值 7 N·m，协议量程 10 不代表允许连续输出 10 N·m；实际热负荷仍取决于安装、散热及工作周期。[厂商产品页](https://www.mdmbot.com/index.php?c=show&id=84)

## 驱动与停止行为

专用模块为 `modules/motor/DMmotor/dm4310_pitch.c/.h`，在原 1 ms 电机任务中执行。原通用 `dmmotor.c` 不注册本 pitch 电机，也不会接管它。

初始化不发送使能。停止时发送 Kp=Kd=0 的零力矩报文，同时获取真实反馈；收到合法指令并被应用层允许后，才请求达妙使能。只有反馈状态为使能时才发送非零力矩。反馈解析核对 8 字节长度和报文内部的电机 ID，故障、反馈过期、目标过期或非有限数值会停止并锁存。

停止表示零主动力矩，不是机械抱闸；重载 pitch 必须有相应机械支撑。硬件故障先排除原因并在达妙工具中处理，恢复健康后再按原工程恢复流程处理软件锁存。自主模式保留原逻辑：检修后复位 C 板重新进入启动流程。

## 视觉联调

本地 `sp_vision_25/io/gimbal/gimbal.hpp` 的收发结构与本固件匹配。更换电机无需修改 Solver、PnP 或目标角符号。

运行视觉的 YAML 中 `R_gimbal2imubody` 必须与 `GIMBAL_TO_IMU_ROTATION` 一致。本固件保持基线单位阵；本机 `standard3.yaml` 为单位阵，但 `windows_auto_aim_live.yaml` 曾配置为 `diag(-1,-1,1)`。应核对实际启动所用配置，不要因为换电机而套用另一个安装矩阵。

联调检查：电脑 `board mode=1`；有效跟踪时电脑 `TX mode=1`；C 板 `recv_data.mode=1`、`sequence` 增加，目标角随目标变化。DM 驱动中同时观察 `pitch_motor->measure.state`、`feedback_seen`、`last_feedback_tick`、`last_torque_nm` 和 `fault_latched`。接收角度变化只能证明通信，实际运动和误差仍需装机测量。

## 构建与测试

Ubuntu 22.04 工程根目录执行：

```bash
make -j4 all
sh tests/run_all_host_tests.sh
```

`make all` 只编译。既有 `make download_dap` 会烧录并复位运行，使用前应先检查安装、当前限幅和运动空间。本次交付不执行烧录。

本版附当前源码编译出的 BIN/HEX/ELF；生成环境、测试结果、源文件校验及固件 SHA256 记录于 `docs/dm4310_verification.json`。`docs/dm4310_changes.patch` 是相对用户原包的本次代码差异，原有 tracking 文档和测试记录保留作历史资料。

上机验收应记录同一组小角度阶跃、往返目标、无目标保持、断开视觉后的停机，以及运行温度和超调；核对实际角度方向、pitch ±20°包络及能否重复恢复。当前离线验证不包含实机响应、温升、供电和比赛规则检查。

参考协议：[达妙 DM-J4310-2EC V1.1 使用说明书，MIT 模式章节](https://damiao.enactic.ai/assets/files/dm-j4310-2ec-v1.1-zh-5d089e0b013d8e2fc203c5b670008a66.pdf)。
