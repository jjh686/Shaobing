# RMUL 3V3 哨兵团队决策

版本：0.2.0；日期：2026-11-01。状态：Ubuntu 构建、策略/时间断言与禁用态启动检查通过；真实状态联调与实车验收待完成。本文与代码由 AI 协助编写，待队伍复核。

详细的车辆约束、八种血量组合、补位条件和未解决情境见 [团队决策需求](团队决策需求.md)。本版替换了“只看自身低血，否则找第一个占点或默认巡逻”的旧策略。

## 来源与范围

采用浮舟湿地/COD 的 ROS 2 → 黑板 → BehaviorTree.CPP 4 → 输出适配结构。本包是独立新入口，原 `cod_behavior`、本地整合树、北极熊导航、串口驱动均未替换。

父工程基线为 `fzsd_xky` 提交 `6c65a992b0bb029f9cd83db6e6d6136f92af6e75`，已有本地整合修改；来源见 `../UPSTREAM_SOURCES.json`。复用了架构组织与持守/恢复设计思路，未宣称所有本地集成功能均为浮舟湿地原版。

协议依据为 [2027 通信协议 V1.0.0（20261029）](https://bbs.robomaster.com/wiki/20204847/1969151)，对照 [2026 最终 V2.0.0（20260626）](https://bbs.robomaster.com/wiki/20204847/811363)。赛事依据为本机《RoboMaster 2027 机甲大师高校联盟赛比赛规则手册 V1.0.0（20261015）》，位于 `C:/Users/zbh26/Desktop/大学/ROBOMASTER/01_规则与官方资料/`。

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

- `DecisionState.msg`：本车状态、两台队友观察、候选任务与分组时间；这是软件契约，不是裁判或电控原始包。
- `TeammateState.msg`：HP/最大 HP/规则状态及其时间；XY 位置及其独立时间。
- `TacticalObjective.msg`：任务 ID、用途、坐标、地形、路线验证、风险、控制归属、维护/空缺/交接证据。
- `DecisionIntent.msg`：模式、目标 ID、原因、射击许可、团队状态和本机补位提示。提示不等于已通知/命令队友。

所有 XY 数据须转换到 `map_frame`，默认 `map`；输入 `header.frame_id` 不一致则拒绝。目标 ID 以后由执行层映射到已批准点位，不由本包编造坐标。

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

## 当前接入状态

默认 `profile: rmul_3v3`、`enabled: false`。本包不打开串口，不创建 Nav2 Action client，不发布 cmd_vel 或实际射击命令。

真实分包适配、地图任务生成、队友观察来源、Nav2 执行仲裁及失败/取消/到达处理尚未接入。没有这些输入时，正常输出应为等待，不能用伪造可达性、时间戳或风险分数让它强行运行。

北极熊已有地面队友位置话题可作为待更新的适配来源；0x020B 是否在本赛项实际稳定下发仍需联调确认。视觉可作为本车观察来源，不能据此增加未经许可的队间通信。

## Ubuntu 构建与查看

本轮通过 WSL Ubuntu-22.04 完成隔离构建和短时禁用态节点检查，未启动 Gazebo、导航或车辆控制。以下为后续持久构建与查看命令。

前提：ROS 2 Humble、ament、rosidl 和 BehaviorTree.CPP 4 已准备。`behaviortree_cpp` 不等于 Nav2 使用的 `behaviortree_cpp_v3`。若既有 `~/fzsd_nav_ws` 已成功构建所需依赖，可加载其环境。

```bash
source /opt/ros/humble/setup.bash
# 仅在此工作区已成功构建时加载：
source ~/fzsd_nav_ws/install/local_setup.bash
cd /mnt/c/Users/zbh26/Desktop/code/robomaster/导航/fzsd_xky
colcon --log-base ~/rm2027_decision_ws/log build \
  --base-paths rm2027_decision \
  --build-base ~/rm2027_decision_ws/build \
  --install-base ~/rm2027_decision_ws/install \
  --cmake-args -DBUILD_TESTING=ON
ctest --test-dir ~/rm2027_decision_ws/build/rm2027_decision --output-on-failure
source ~/rm2027_decision_ws/install/local_setup.bash
ros2 launch rm2027_decision intents.launch.py
```

默认输出 Disabled。仅打开意图计算（仍不执行车辆控制）：

```bash
ros2 launch rm2027_decision intents.launch.py enabled:=true
```

另一个加载相同环境的终端：

```bash
ros2 topic echo /decision2027/intent
ros2 interface show rm2027_decision/msg/DecisionState
```

没有有效状态时应 WaitState；本车状态完整但缺少任务地图时应 WaitPlan。仿真输入才添加 `use_sim_time:=true` 并共享 /clock。参数启动后只读，改配置需重启本节点。消息结构从 0.1.x 扩展至 0.2.0，需重新生成接口并更新发布端，旧发布端不能直接混用。

## 验证与待办

已编写断言：本车门控和 HP 滞回、八种团队低血组合、队友滞回/最大血量、死亡/虚弱/未知、位置防堆叠、持守与交接、岗位漏项/争夺中不离岗、地形/路线硬过滤、风险预算/非法数值、无图等待与全低血不排队。`freshness_test.cpp` 检查重复来源时间、暂停、未来/乱序时间及超时边界。

2026-11-01 实测结果：

- Ubuntu-22.04 下纯 C++17 策略与新鲜度测试通过；使用 `-Wall -Wextra -Wpedantic -Werror` 编译通过。
- 加载 ROS 2 Humble 与既有 FZSD 依赖后，隔离 `colcon build` 成功，新增四种消息与节点共同编译通过。
- CTest 的 `rm2027_policy`、`rm2027_freshness` 两项均通过。Release 构建中测试显式保留 assert，未因 NDEBUG 跳过检查。
- 实际加载 `trees/intents.xml`，运行默认禁用节点约 3 秒；输出 Disabled，SIGINT 正常退出。
- 最后隔离构建临时路径为 `/tmp/rm2027-ros-final-Is0jlw`，该路径可能随 WSL 临时目录清理而消失，不作为部署安装目录。

尚未完成真实 DDS 输入回放、全场仿真、Nav2 执行仲裁、长期内存或车辆验收。上述测试证明的是已列场景下的程序行为，不证明战术最优或可直接上车。

待确认地图点位、队友状态来源、虚弱确认、风险分数和控制区维护证据；随后接入仲裁执行层，并补控制度/时间/敌情评分。

修改前快照：首版前为 `_backups/版本015_20261031-231430`；确认 3V3 前为 `版本016_20261031-233603`；本轮团队策略前为 `版本017_20261101-000930`。回退应只处理本次包和对应差异，避免整工作区恢复覆盖其他修改。
