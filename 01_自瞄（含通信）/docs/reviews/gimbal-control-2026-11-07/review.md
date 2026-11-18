# 云台视觉追踪控制移植前对照（2026-11-07）

结论：参考工程确实有可借鉴的视觉追踪实现，但当前工程已经具备相同的“角度环→速度环”结构。移植的有效范围应是视觉参数管理、状态切换和控制细节；不能用整份参考任务、PID 数字或通信驱动替换当前工程。当前源码和固件均未修改，本次未连接 MCU、复位或烧录。

## 1. 三份工程与证据范围

- 接收移植的电控：`/home/wangjiaming/下载/codex/新云台/basic_framework`，下文 T。
- 参考电控：`/home/wangjiaming/下载/rmcod2026_-sentry-dev`，下文 R。Pitch 在 `gimbal`，Yaw 在 `chassis/COD_H7_Template`。
- 当前视觉：`/home/wangjiaming/下载/sp_vision_25-main`，下文 V。

本文基于文件实际实现，不把注释、旧说明、代码中出现的结构体字段当作“功能已接通”。参考工程未做上板性能验证。当前参数表是源码基线，不能等同于本轮实机变量读取。

同目录的 [parameter_baseline.json](/home/wangjiaming/下载/sp_vision_25-main/docs/reviews/gimbal-control-2026-11-07/parameter_baseline.json) 单独保存参考原值与当前原值，标明“仅对照，不可加载为控制配置”；[source_manifest.json](/home/wangjiaming/下载/sp_vision_25-main/docs/reviews/gimbal-control-2026-11-07/source_manifest.json) 保存 33 个相关文件的 SHA-256，包括本地 ELF。任何源文件改变后应重新检查相应结论。

## 2. 输入、反馈、输出逐项对照

| 项目 | 当前 T | 参考 R | 移植决定 |
|---|---|---|---|
| 视觉通信 | SP 命令 29 字节、状态 43 字节，小端 float32，CRC | 以 0xFF 选择视觉帧，直接取字节恢复 float | 保留 SP 收发，不换旧协议 |
| 视觉角度单位 | yaw/pitch：rad | Pitch：度；Yaw：度 | 内部继续 rad，参考度值不能直接写入 |
| 目标含义 | 视觉世界坐标系中的绝对角目标 | 固件按其 IMU 姿态坐标中的绝对角目标解释 | 不把目标加到当前姿态上，不逐帧累加 |
| Yaw 角反馈 | 转换后的 IMU 连续角，rad | 云台板 IMU 角转发到底盘，度 | 保留当前连续化与安装转换 |
| Yaw 速度反馈 | IMU 转换得到的欧拉角速度，rad/s | 云台板 IMU Yaw_Gyro，度/s | 不能照搬速度增益 |
| Pitch 角反馈 | 转换后的 IMU pitch，rad | `-INS_Info.Roll_Angle`，度 | 不复制参考的 Roll 轴名和负号 |
| Pitch 速度反馈 | 转换后的 IMU 欧拉角速度，rad/s | 达妙 MIT 原始 Velocity，未作57.3转换 | 不切换反馈来源；原电机速度物理单位需手册确认 |
| 电机 | 两轴 GM6020 | Yaw GM6020；Pitch 达妙 MIT | 保留两轴 GM6020 驱动 |
| CAN 输出 | CAN1，共用 0x1FF，反馈 0x205/0x206 | Yaw 在另一板发 0x1FE；Pitch 在本板发 MIT torque 字段 | 不复制另一板路由、发送 ID 或 torque 输出 |
| 最终输出量纲 | CAN 有符号驱动命令数值 | Yaw CAN 原始命令；Pitch MIT torque 字段 | 未证明物理等效前，禁止比较或复用输出增益 |
| PID 接口参数顺序 | `PIDCalculate(pid, measure, ref)` | `PID_Calculate(pid, Target, Measure)` | 调用时必须重排，不能直接替换函数名 |
| 积分算法 | Ki × error × 实际秒数 dt，当前启用梯形积分 | 每周期 sum_error += error，再乘 Ki | Ki 必须按时间基准与量纲重新定义 |
| 积分限幅 | 直接限制 Iout，单位为驱动输出 | 限制误差累计值，之后再乘 Ki | 同名 LimitIntegral 不代表同一数值含义 |
| 视觉速度/加速度 | 收到 rad/s、rad/s²，但还未接入控制 | 接收包没有这四个字段 | 本次先不增加前馈，不把 aba 当作这些字段 |
| 机械限位 | 编码器启动相对 ±10°；每次误差先夹 ±5° | Pitch 只限手动目标；Yaw ±180/360 是环绕 | 保留当前机械包络并单独做实机标定 |
| 无目标 | SP mode0 进入 HOLD，锁存当前角度 | 手动视觉模式维持旧目标；Auto 可扫描 | 保留当前独立云台行为 |
| 超时/故障 | USB、会话、CAN、IMU、命令超时及故障锁存 | 检查到的视觉链没有消息年龄门控 | 保留当前保护链 |

证据入口：[T 协议][t-proto]、[T 姿态转换][t-math]、[T 云台反馈与电机配置][t-gimbal]、[T 目标选择][t-boot]、[T 电机驱动][t-driver]、[T PID API][t-pid]、[R Pitch 反馈][r-pitch-feedback]、[R Yaw 反馈][r-yaw-feedback]、[R 接收][r-rx]、[V 坐标变换][v-solver]。

参考 Pitch 的 MIT Velocity 常见协议单位为 rad/s，但仓库未找到其具体电机手册或明确单位注释。已证明它未乘57.3，不能把“常见协议约定”写成已经实测确认。

## 3. 数值基线：明确原值，不生成可直接烧录的移植参数

当前 T：

| 参数 | Yaw | Pitch | 量纲/含义 |
|---|---:|---:|---|
| 角度 Kp | 8 | 10 | s⁻¹，rad→rad/s |
| 角度 Ki、Kd | 0、0 | 0、0 | 当前未启用 |
| 角度死区 | 0.001745329 | 0 | rad，Yaw 为0.1° |
| 角度输出上限 | 8.726646 | 8.726646 | rad/s，即500°/s |
| 速度 Kp | 50 | 50 | 驱动命令/(rad/s) |
| 速度 Ki | 200 | 350 | 驱动命令/rad |
| 速度 Kd | 0 | 0 | 当前未启用 |
| 速度 Iout 上限 | 3000 | 2500 | 驱动命令 |
| 速度输出上限 | 20000 | 20000 | 驱动命令 |
| 瞬时角误差上限 | 0.08726646 | 0.08726646 | rad，即5° |

在当前 P 外环下，±5°误差上限使速度目标幅值通常不超过 Yaw 0.698132 rad/s、Pitch 0.872665 rad/s，即40°/s、50°/s，机械包络还可能进一步压低。单独提高8.726646的外环输出限幅，不能解除前面的5°误差限制。这不代表5°就是总行程。

参考 R 的视觉参数原值：

| 环 | Kp | Ki | Kd | 误差累计上限 | 输出上限 |
|---|---:|---:|---:|---:|---:|
| Yaw 角度 | -35 | 0 | 0 | 100 | 400 |
| Yaw 速度 | 205 | 0.2 | 2 | 1000 | 15000 |
| Pitch 角度 | -0.51 | 0 | 0 | 1 | 8 |
| Pitch 速度 | 0.305 | 0.001 | 0 | 1 | 8 |

来源：[R Yaw 参数][r-yaw-params]、[R Pitch 参数][r-pitch-params]、[T 当前配置][t-config]。

参考 Yaw 的 Iout 实际限幅为 0.2×1000=±200 个参考驱动命令单位；参考 Pitch 为0.001×1=±0.001个 MIT torque 字段单位。不能把1000、1直接填入 T 的 Iout 上限，也不能因为某个 Ki 数字小就认为其积分效果一定弱。

### 不能只按57.3换算

设 c=180/π：

- 若外环的输入从度变rad、输出也从度/s变rad/s，两个c抵消，纯比例外环的增益幅值可以不变。符号仍取决于误差定义和驱动方向。
- 参考 Yaw 先算 `e=target-measure`，再调用 `PID(0,e)`，PID 内部实际误差是 `-e`。因此其 Kp=-35 的整体效果是 +35×原角误差。T 若按正常 `PID(measure,target)` 调用，直接复制-35会改变反馈方向。
- 如果某个环只有输入单位改变、输出物理单位不变，才出现单边比例因子。例如参考 Pitch 的度角→MIT速度映射，不能按 Yaw 的同一规则处理。
- 在输出物理命令、误差符号、滤波结构都相同，且参考周期确实为 T 秒的理想条件下，离散逐拍速度 PID 转到显式时间 PID，才有幅值关系 `Kp_new=c*Kp_old`、`Ki_new=c*Ki_old/T`、`Kd_new=c*Kd_old*T`。这里是度/s输入转rad/s输入的例子。这些式子只在同为矩形积分、误差差分且未触发限幅等对应条件下严格匹配；当前采用梯形积分、测量微分和抗饱和，动态不能由这些尺度关系完整复现。当前输出协议、微分形式、实际T等条件尚不满足，不能把公式算出的数当作上板参数。
- 参考任务 `osDelay(1)` 请求延时1个系统tick，实际阻塞时间受tick相位及调度影响，控制周期还包含执行时间，不能视为精确1ms周期。

另一个历史纠正：当前工程旁的原始版本外环曾用度角、内环用原始陀螺rad/s，因此“旧版所有环都用度”不成立。统一为rad后保留外环8/10确实改变了原数值映射，但不能据此把所有PID同时乘57.3。原始代码不作为本次新参数来源。

## 4. 参考工程发现的实现问题

1. **Yaw 的 aba 修正实际为零。** [Yaw 控制][r-yaw-control] 先保存旧输出，在外环重新计算前就用输出减刚保存的值。不能把它当成已经有效的前馈。
2. **Pitch 的 aba 使用上一拍差分。** [Pitch 双环][r-pitch-control] 内环使用 aba 后才更新它。若要借鉴差分整形，需要重新定义离散式、采样周期和测试；本轮不纳入首个移植版本。
3. **视觉 Pitch 未经过手动目标限位。** [Pitch 目标更新][r-pitch-target] 第193行只夹 `Pitch_Angle`，真正视觉环用 `Vision_Pitch_Angle`。Yaw 的±180/360处理也不是机械限位。
4. **未使用的 PID 仍在每拍运算。** 两轴都先计算手动/视觉等环，再选择输出；Off和模式切换没有相应清理。复制这种方式会带入未选控制器的积分状态。
5. **视觉接收缺少可靠性检查。** [MiniPC 接收][r-rx] 未使用 Len 检查长度，未做CRC、有限数值、拆包和视觉超时；`Distance!=-1` 判断位于视觉帧分支之外，可能把旧值或初始化值认成有效目标。
6. **手动 Pitch 初始目标与反馈符号不一致。** 第188行首次手动目标取 +Roll，反馈第144行为 -Roll，非零姿态会引入初始误差。不能照抄初始化语句。
7. **整车耦合不适用于当前独立云台。** 参考 Yaw 输出受底盘轮组校准完成条件控制；Auto无目标可能按150°/s扫描。当前不引入底盘、导航或扫描依赖。

这些是对所查实现的判断，不等于该参考整车一定不能运行；部分问题可能在其固定工况下被掩盖，也可能没有触发对应分支。

## 5. 值得借鉴的内容及落实方式

| 内容 | 参考价值 | 当前落实方式 |
|---|---|---|
| 手控与视觉有独立参数 | 两种目标来源的动态特征不同，方便分别整定 | 将视觉参数按轴、按环明确命名；需要手控共存时再建立独立配置，使用同一套经过验证的计算逻辑 |
| 角度环接速度环 | 角度误差生成速度目标，速度误差生成驱动命令 | 当前已实现，保留结构，重点检查增益、积分和限制，不重复包一层PID |
| 角度误差走最短方向 | 避免±π附近走整圈 | 当前已有wrap和连续Yaw，保持rad实现，不复制±180/360数字 |
| 测量→目标→控制→发送职责分开 | 有助于逐段观测和定位 | 当前已有CMD/Gimbal/Motor分层，继续分别记录RX、目标、P/I、最终输出与实际反馈 |
| Pitch 重力补偿建模思路 | 对有重力负载的轴，能减少对积分补偿的依赖 | 参考当前该项实际被设为0；需本机质量、力臂、符号与驱动标定后另做，不抄其质量/力臂 |
| 状态明确区分手控、视觉、保持 | 有利于定义切换行为 | 保留当前TRACKING/HOLD/FAULT，在切换边界设计状态处理；不抄参考“所有PID一直算”的实现 |

参数独立不等于必须同时创建多组运行中的PID，更不代表单靠分组就能解决慢响应。当前默认上电自主路径不依赖遥控，首版可只针对视觉控制器做清晰的参数配置。

## 6. 当前工程仍需解决的控制问题

- **速度积分可能主导响应。** 本轮之前的只读数据出现正速度目标约+0.698rad/s、正P约+35，而I约-641至-425，合成输出仍负。若误差维持且其余条件不变，旧积分卸载可以花数秒。这是一个有数据支持的迟滞机制，但当时用户手动拨了Yaw，不能据此测出视觉目标运动后的总延迟，也不能直接断定物理电机反转。
- **STOP清PID已实现，TRACKING↔HOLD未必经过STOP。** [T 驱动][t-driver] 停止/故障分支会清PID，但目标换向或保持模式切换会沿用积分。
- **死区注释与实际行为不一致。** [T PID][t-pid] 的死区分支只清Output、ITerm，未清Iout。是否修改其策略应单独评估；不能在全局PID库里顺手改动而影响其他电机。
- **安装与坐标需要物理确认。** 两端矩阵都是identity仅证明配置一致，不能证明安装已标定。参考的-Roll轴映射不能用于替代确认。若相机与云台运动关系不满足固定外参，预测/坐标变换也会产生错误。
- **速度/加速度前馈尚未接通。** 后续若使用视觉yaw_vel/pitch_vel，必须明确它们是参考轨迹导数，并设计限幅/丢帧/切换；不能直接将yaw_acc数字加到CAN输出。

无证据把全部问题归因于USB单位。之前非零样本中RX yaw=-0.06141764298rad，CMD同值，PID目标约-0.06141763553rad，支持已观测角度链不存在度/弧度倍数错误。

原始数据与限制见 [恢复后采样记录](/home/wangjiaming/下载/sp_vision_25-main/logs/diagnostics/2026-11-07_2030/recovery_notes.md)。

## 7. 下一步实施边界与顺序

1. **固定基线。** 以本目录文件哈希和数值对照为起点；实际改动前另建隔离副本，保存完整原工程与构建产物。新参数不得覆盖“当前基线”或“参考原值”。
2. **保留接口和驱动。** SP协议、原始wxyz姿态回传、两端安装转换、CAN映射、所有超时/故障/机械保护保持现有定义。继续关闭发射。
3. **只改明确的控制适配点。** 首轮关注 `gimbal_config.h` 的视觉参数表达、`gimbal.c` 的控制器配置/模式交接；若需驱动支持参数切换，只在电机任务的周期边界应用，避免两个任务并发改PID。不换整份Control_Task，不一次性改全局PID库。
4. **确定切换和积分策略。** 定义TRACKING→HOLD、HOLD→TRACKING、STOP/FAULT的目标与状态交接。保持力矩可能需要积分，不能每次目标变化都清I；也不能让未激活控制器持续积累。以可观测的P/I/输出和切换样本选择清理、限幅或平滑接管方式。
5. **先验证基础闭环，再整定动态。** 在有效非零目标和安装关系明确的条件下，按轴验证方向、目标/反馈单位、输出模式，再分开调整速度环和角度环；每次只改一类因素，保留上一份可回退参数。不同时引入aba、重力补偿和视觉导数前馈。
6. **最后考虑扩展。** 基础角度跟随通过后，才单独评估速度前馈、负载补偿和视觉预测参数；用数据证明收益，再纳入正式配置。

应当先解决的是“控制量纲、参数语义、积分状态与实际驱动”的对应，而不是把参考工程数值全部搬进来后再猜哪个不对。

## 8. 验证和交付要求

本轮已做：静态追踪两轴完整调用链、对照PID实际公式、提取原始数值、核对本地ELF来源信息及保存文件哈希。未做新固件编译、烧录、目标Flash校验或运动试验。

实施后有意义的离线验证应覆盖：

- 相同绝对角不被当增量累加；跨±π误差走最短方向。
- PID参数顺序、角度/速度量纲、积分随真实时间增长的关系；积分限幅检查Iout而非误差累计量。
- 机械包络对所有目标来源生效；mode0保持目标而非归零。
- 模式交接、故障、超时、USB重连不沿用不应保留的旧目标/控制器状态。
- SP序列化和现有视觉兼容；不启用发射或底盘。

运动验证应在倒计时后分别进行单轴目标小阶跃、保持、反向与丢目标，记录视觉TX、板端RX、实际PID目标、角/速度反馈、P/I/输出、mode/state及各消息年龄。手动拨动云台的抗扰试验与移动视觉目标的跟随试验分开记录，不混算延迟。

本地ELF SHA-256为 `cefaaf64ba18a18038d4e2ebabacaa06eee3e43539fb330af20f5419bd551b71`，DWARF路径和文件时间未见明确来源错配证据；这不证明当前每字节源码及所有编译宏等于该ELF，也不替代板上Flash校验。

[t-proto]: /home/wangjiaming/下载/codex/新云台/basic_framework/modules/master_machine/sp_protocol.h:9
[t-math]: /home/wangjiaming/下载/codex/新云台/basic_framework/application/gimbal/gimbal_math.c:30
[t-gimbal]: /home/wangjiaming/下载/codex/新云台/basic_framework/application/gimbal/gimbal.c:70
[t-boot]: /home/wangjiaming/下载/codex/新云台/basic_framework/application/cmd/auto_aim_boot.c:80
[t-driver]: /home/wangjiaming/下载/codex/新云台/basic_framework/modules/motor/DJImotor/dji_motor.c:288
[t-pid]: /home/wangjiaming/下载/codex/新云台/basic_framework/modules/algorithm/controller.c:153
[t-config]: /home/wangjiaming/下载/codex/新云台/basic_framework/application/gimbal/gimbal_config.h:54
[r-pitch-feedback]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/gimbal/Task/Src/Control_Task.c:143
[r-yaw-feedback]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/chassis/COD_H7_Template/Task/Src/Control_Task.c:351
[r-rx]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/gimbal/Components/Device/Src/MiniPC.c:107
[v-solver]: /home/wangjiaming/下载/sp_vision_25-main/tasks/auto_aim/solver.cpp:48
[r-yaw-params]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/chassis/COD_H7_Template/Task/Src/Control_Task.c:74
[r-pitch-params]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/gimbal/Task/Src/Control_Task.c:49
[r-yaw-control]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/chassis/COD_H7_Template/Task/Src/Control_Task.c:668
[r-pitch-control]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/gimbal/Task/Src/Control_Task.c:203
[r-pitch-target]: /home/wangjiaming/下载/rmcod2026_-sentry-dev/gimbal/Task/Src/Control_Task.c:150

