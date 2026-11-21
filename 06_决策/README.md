# RMUL 3V3 哨兵团队决策

版本 0.3.0。默认用于状态和导航目标预览，实车输入与场地数据待验收。本文与代码由 AI 协助编写，待队伍复核。

详细的车辆约束、八种血量组合、补位条件和未解决情境见 [团队决策需求](团队决策需求.md)。本版替换了“只看自身低血，否则找第一个占点或默认巡逻”的旧策略。

## 来源与范围

采用浮舟湿地/COD 的 ROS 2 → 黑板 → BehaviorTree.CPP 4 → 输出适配结构。本包是独立新入口，原 `cod_behavior`、本地整合树、北极熊导航、串口驱动均未替换。

父工程基线为 `fzsd_xky` 提交 `6c65a992b0bb029f9cd83db6e6d6136f92af6e75`。本包借鉴其架构组织与持守、恢复设计。

规则及协议原件见 [race rule text](race%20rule%20text/)。本包依据 2027 联盟赛比赛规则 V1.0.0 和 2027 通信协议 V1.0.0。

- 官方串口使用 0xA5、两字节命令码；北极熊私有协议使用 0x5A、一字节 ID。两层须分别解码映射，私有 0x0D Buff 不等于官方 0x020D。
- 2027 的 HP、场地事件、伤害、RFID 与部分字段发生变化；0x020D 姿态反馈和 0x0120 指令在 2026 最终版已存在。协议中存在某功能不代表适用于 RMUL 哨兵。
- RMUL 3V3 哨兵全自动，不允许多机通信；初始允许发弹量 750，局内不能增加。
- 战亡自动读条复活；复活后虚弱，不能射击或激活控制区，检测己方补给区卡后解除。决策不主动请求复活、付费复活、购弹或 RMUC 姿态切换。
- 用户说明当前三车不能上高地或通过隧道：本版作为车辆能力硬约束，包括目标地形和途中穿越路线，不写成赛事统一禁令。

## 决策行为

| 顺序 | 条件 | 输出 |
| --- | --- | --- |
| 1 | 未使能/人工接管、不支持赛项、关键数据异常或非比赛 | Disabled / WaitConfig / WaitState / WaitGame |
| 2 | 战亡、规则状态未知、定位失效 | Dead / WaitState / LocalizationLost |
| 3 | 自身虚弱或血量恢复滞回有效 | 经验证的 Recovery 目标；无合格路线则 WaitPlan |
| 4 | 当前己方控制点确需持续维护，未完成交接 | Hold；跨帧保留持守责任 |
| 5 | 确认己方岗位空缺且没有实际队友维护 | Cover |
| 6 | 队友恢复需要与批准的地面撤退掩护目标成立 | Cover |
| 7 | 有合格中立/敌占点，无更优先任务 | Capture；队友未知/均不可用时更谨慎 |
| 8 | 有显式批准巡逻且无待解决持守责任 | Patrol |
| 9 | 没有新鲜、可达、风险可接受的目标 | WaitPlan，不盲发 home 或 patrol |

队友 HP 与位置分别计时，低血、恢复中、死亡、虚弱和未知分开。自身低血不因队友也低血而被禁止恢复；不默认补给排队，也不自动指挥人控队友。

默认血量策略为低于 30% 进入恢复、达到 80% 退出；按每辆车各自有效最大血量计算。队友初次出现在中间区间时保守看作恢复未完成。风险预算是可调工程分数，不代表死亡概率；未知风险不能当零风险。

本版仍使用明确任务优先级，没有实现全局最优团队调度，也未完整建模敌方火力、控制度濒危、剩余时间和各区独立占领进度。详见团队需求中的后续情境。

## 状态与地图契约

- `NavigationEvidence.msg`：定位有效性、路线和任务证据，以及可选的本车队友观察。
- `DecisionState.msg`：本车状态、两台队友观察、候选任务与分组时间；这是软件契约，不是裁判或电控原始包。
- `TeammateState.msg`：HP/最大 HP/规则状态及其时间；XY 位置及其独立时间。
- `TacticalObjective.msg`：任务 ID、用途、坐标、地形、路线验证、风险、控制归属、维护/空缺/交接证据。
- `DecisionIntent.msg`：模式、目标 ID、原因、射击许可、团队状态和本机补位提示。提示不等于已通知/命令队友。

所有 XY 数据须转换到 `map_frame`，默认 `map`。三节点共享策略配置，并验证批准点位文件的地图帧。桥接器只接受点位表中坐标、类型和路线证据一致的目标。

各分组时间是本机收到有效原始动态证据的 ROS 时间；重复汇总不得刷新。组合字段取所需动态证据中最旧的时间。静态地图和标定按版本校验，不靠周期重打时间戳伪装动态观察。

适配层必须保证：

1. 官方 game_type=4（RMUL 3V3），game_progress=4（比赛中）；不能直接套用模拟器的不同 stage 枚举。
2. `life_state_valid`/`weakened` 有可靠电控反馈或完整事件证据；HP>0 不能证明虚弱已解除，它们也不是声称官方 0x0201 新增了字段。
3. `shooter_enabled` 是硬件与规则许可的合取，不能只复制发射机构电源位。未知规则禁射时不提供正面许可。
4. 可达性来自车辆尺寸/通行能力和规划验证。平地目标若需要穿隧道/上高地仍拒绝；未知风险/未验证路线不作为可执行目标。
5. `vacancy_confirmed` 需要新鲜区域证据，不能由过期位置或没看到队友生成；`handover_confirmed` 还需新鲜、状态可用队友实际在位。
6. `self_present`、归属、维护与交接证据需要真实 TF/区域观测，不能由目标发布成功推导。

节点为 10 Hz wall timer、单线程执行，队列深度 1，最多 16 个候选任务，每个 ID 最多 64 字符。原始来源时间不推进时，steady 时间仍会使其过期；仿真暂停也不会永久保留旧许可。时钟后退清空缓存和策略状态。

这些措施限制本模块缓存和陈旧状态使用，不构成 Gazebo 图形进程内存泄漏的修复结论。

## 节点与执行链路

| 构成 | 职责 |
| --- | --- |
| `scripts/referee_rx.py`、`referee_state_adapter.py` | 校验和解码裁判帧，结合本车定位、路线和队友观察生成 `DecisionState`。配置 UART 后只读接收，断开清除旧状态并重试。 |
| `src/decision_node.cpp`、`trees/intents.xml` | 校验各来源时间，通过行为树读取状态、选择并发布 `DecisionIntent`。 |
| `include/rm2027_decision/policy.hpp` | 定义恢复、守点、补位、掩护、占点及巡逻的优先顺序。 |
| `scripts/intent_nav_bridge.py`、`config/approved_tasks.yaml` | 校验获批点位和最新路线，预览目标，或在显式启用后发送本车 Nav2 Action。失败按退避最多尝试三次，证据失效撤销目标，等待终态期间不发送下一目标。 |
| `02_导航（含通信）` 的 Nav2 与 MotionGate | Nav2 负责规划和速度指令。live 模式由 MotionGate 检查传感器、姿态、手动使能及任务心跳，再经车内底盘通信执行。 |

整合入口默认 `enabled:=false`、`execute_navigation:=false`，采用 `fixed_map`，Action 为 `/red_standard_robot1/navigate_to_pose`。当前 live 入口对应 `navigation_backend:=live` 和 `/navigate_to_pose`，需由 02 的启动入口选择 `goal_owner:=decision`。此模式禁用自带探索器，MotionGate 仅接收决策节点的活动心跳，保留原手动使能与底盘保护。测试中不启用实车执行。

赛场点位、禁行区、经核验的路线证据发布端、雷达外参和真实裁判数据接线仍需实车验收。官方协议的 `0x020B` 不能直接作为 RMUL 队友定位来源。本车可以基于可靠的本地观察补充，缺少证据时保留未知。

决策发送的是车内导航目标。裁判系统的 `0x0120` 属于另一条链路，本包的 UART 不发送上行帧。当前仓库底盘源码的周期上报与实车烧录版本需单独核对。

## Ubuntu 构建与查看

在 Ubuntu 上打开 `RM_11_13` 仓库根目录。构建依赖包括 ROS 2 Humble、ament、rosidl 和 BehaviorTree.CPP 4 软件包 `behaviortree_cpp`。Nav2 的 Python 接口使用 `nav2_msgs`。

```bash
source /opt/ros/humble/setup.bash
colcon --log-base ~/rm2027_decision_ws/log build \
  --base-paths '06_决策' \
  --build-base ~/rm2027_decision_ws/build \
  --install-base ~/rm2027_decision_ws/install \
  --cmake-args -DBUILD_TESTING=ON
ctest --test-dir ~/rm2027_decision_ws/build/rm2027_decision --output-on-failure
source ~/rm2027_decision_ws/install/local_setup.bash
ros2 launch rm2027_decision rm_11_13.launch.py
```

默认输出 `Disabled`。打开策略预览时保持执行禁用。

```bash
ros2 launch rm2027_decision rm_11_13.launch.py enabled:=true execute_navigation:=false
```

另一个加载相同环境的终端：

```bash
ros2 topic echo /decision2027/intent
ros2 topic echo /decision2027/navigation_status
ros2 interface show rm2027_decision/msg/DecisionState
```

没有有效裁判状态时应为 `WaitState`。缺少定位证据时为 `LocalizationLost`，有定位但无合格任务时为 `WaitPlan`。仿真预览使用 `use_sim_time:=true` 并共享 `/clock`。修改启动参数后需重启节点。

## 验证与待办

已编写断言：本车门控和 HP 滞回、八种团队低血组合、队友滞回/最大血量、死亡/虚弱/未知、位置防堆叠、持守与交接、岗位漏项/争夺中不离岗、地形/路线硬过滤、风险预算/非法数值、无图等待与全低血不排队。`freshness_test.cpp` 检查重复来源时间、暂停、未来/乱序时间及超时边界。

原始审查记录保存在 [docs](docs/) 和 [review](review/)。实际裁判串口、真实 Nav2、场地禁区、长时间联合运行和实车停车仍待验收。控制度、比赛剩余时间及双方占领进度尚未进入策略。
