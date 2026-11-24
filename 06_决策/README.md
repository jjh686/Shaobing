# RMUL 3V3 哨兵团队决策

版本 0.3.0。默认用于状态和导航目标预览，实车输入与场地数据待验收。本文与代码由 AI 协助编写，待队伍复核。

详细的车辆约束、八种血量组合、补位条件和未解决情境见 [团队决策需求](docs/团队决策需求.md)。本版替换了“只看自身低血，否则找第一个占点或默认巡逻”的旧策略。

## 目录与代码职责

目录组织参考北极熊的决策包。行为树、启动入口、节点源码和参数分别存放。裁判与视觉输入、导航桥接按职责分目录，规则原件和审查记录归入文档。[北极熊决策包目录](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior)

```text
06_决策/
├── behavior_trees/               行为树编排
│   └── intents.xml
├── launch/                       ROS 2 启动入口
│   ├── intents.launch.py         单独启动决策树
│   ├── rm_11_13.launch.py        接入已有导航的联调入口
│   └── sentry_bringup.launch.py   上位机联调总启动
├── src/                          C++ 节点实现
│   └── decision_node.cpp
├── include/rm2027_decision/       C++ 策略与时间检查
│   ├── policy.hpp
│   └── freshness.hpp
├── scripts/
│   ├── adapters/                 输入适配
│   │   ├── referee_rx.py
│   │   ├── referee_state_adapter.py
│   │   └── vision_feedback.py
│   ├── navigation/               导航证据与目标桥接
│   │   ├── navigation_evidence.py
│   │   └── intent_nav_bridge.py
│   └── local_env.sh              Ubuntu 构建与运行环境
├── msg/                          ROS 2 消息
│   ├── DecisionState.msg
│   ├── DecisionIntent.msg
│   ├── NavigationEvidence.msg
│   ├── TacticalObjective.msg
│   └── TeammateState.msg
├── config/                       策略参数与实车点位
│   ├── decision.yaml
│   └── approved_tasks.yaml
├── maps/                         场地 CAD 和离线地图
│   └── offline/
├── robot_description/             离线车模及其几何参数
├── test/                         单元测试和离线导航联调
├── docs/                         需求、规则原件与历史审查
│   ├── rules/
│   ├── reviews/20261121/
│   ├── 团队决策需求.md
│   └── 决策代码与规则审查-20261121.md
├── CMakeLists.txt                 编译、生成消息、安装和注册测试
└── package.xml                   ROS 2 包名、版本与依赖
```

### 启动入口

| 文件 | 作用 |
| --- | --- |
| [intents.launch.py](launch/intents.launch.py) | 只启动行为树节点。可回放已有的 `DecisionState`，查看同一批输入会得到什么结果。 |
| [rm_11_13.launch.py](launch/rm_11_13.launch.py) | 启动状态适配、决策树和导航桥。导航证据与 Nav2 由配套的导航进程提供。 |
| [sentry_bringup.launch.py](launch/sentry_bringup.launch.py) | 在以上节点之外启动导航证据节点。根据参数接入已有导航或启动本队实时导航，也可加载预览车模和启动录包。 |
| [local_env.sh](scripts/local_env.sh) | 加载 ROS Humble、BehaviorTree.CPP v4 以及本包已构建的安装环境。 |

构建后仍使用 `ros2 launch rm2027_decision` 加上对应启动文件名。源码的行为树路径为 `behavior_trees/intents.xml`，安装目录也保留此前的 `trees/intents.xml` 供已有的 `tree_file` 配置使用。

### 运行代码

| 文件 | 输入、处理与输出 |
| --- | --- |
| [referee_rx.py](scripts/adapters/referee_rx.py) | 接收原始字节，检查帧头、长度和 CRC，解码本包已支持的裁判字段，并维护本车生命状态。 |
| [vision_feedback.py](scripts/adapters/vision_feedback.py) | 接收本机视觉状态，解析跟踪状态和时间，过滤无效或过期的数据。 |
| [referee_state_adapter.py](scripts/adapters/referee_state_adapter.py) | 把裁判数据、导航证据和视觉状态汇总为 `DecisionState`，并保留各输入的时间和有效性。 |
| [decision_node.cpp](src/decision_node.cpp) | 订阅 `DecisionState`，注册并运行行为树节点，发布 `DecisionIntent`。状态变化时记录模式、目标和原因。 |
| [intents.xml](behavior_trees/intents.xml) | 按顺序读取状态、调用策略、发布结果。具体判断由 C++ 策略提供。 |
| [policy.hpp](include/rm2027_decision/policy.hpp) | 根据输入的有效性、当前模式和可用任务决定输出，保存需要跨轮比较的状态。 |
| [freshness.hpp](include/rm2027_decision/freshness.hpp) | 检查原始输入的时间。收到同一条旧数据时维持原始时间，供决策节点判断超时。 |
| [navigation_evidence.py](scripts/navigation/navigation_evidence.py) | 根据 TF 确认定位，通过 Nav2 规划服务和约束栅格检查路线，再发布可供决策使用的 `NavigationEvidence`。 |
| [intent_nav_bridge.py](scripts/navigation/intent_nav_bridge.py) | 根据点位表和最新证据核对 `DecisionIntent`，默认预览目标。启用执行后通过 Nav2 Action 发目标，负责状态回报和必要的撤销。 |

运行时的数据关系如下。视觉状态随合并消息提供给联调观察。导航闭环从路线证据开始，经过选点，再回到 Nav2 执行。

```text
裁判字节 → referee_rx.py ───────┐
视觉状态 → vision_feedback.py ─┼→ referee_state_adapter.py → DecisionState
TF + 规划 + 地图 → 导航证据 ────┘                           ↓
                         DecisionIntent ← 行为树 ← decision_node.cpp
                               ↓
                         intent_nav_bridge.py → Nav2
```

### 参数、资源与测试

| 文件或目录 | 作用 |
| --- | --- |
| [msg/](msg/) | 决策输入、输出及证据的五个 ROS 2 消息，字段职责见[状态与地图契约](#状态与地图契约)。 |
| [decision.yaml](config/decision.yaml) | 赛项、地图坐标系、输入超时和策略阈值。 |
| [approved_tasks.yaml](config/approved_tasks.yaml) | 经场地核对后填写的实车点位表。 |
| [maps/](maps/) | 场地 STEP 原件。离线的 `red_map.yaml` 定义栅格坐标与分辨率，`red_map.pgm` 是栅格图，`red_tasks.yaml` 是配套示例点位。 |
| [robot_description/](robot_description/) | `sentry_preview.urdf.xacro` 描述离线车模，`offline_geometry.yaml` 存放模型尺寸和安装参数。 |
| [policy_test.cpp](test/policy_test.cpp) · [freshness_test.cpp](test/freshness_test.cpp) | 分别验证策略输出和时间有效性。 |
| [referee_rx_test.py](test/referee_rx_test.py) · [vision_feedback_test.py](test/vision_feedback_test.py) | 分别验证裁判字段解析和视觉状态输入。 |
| [navigation_closed_loop.py](test/navigation_closed_loop.py) | 用离线 3V3 栅格、真实 Navfn 规划器和模拟运动服务验证目标收发及取消。 |
| [docs/rules/](docs/rules/) · [docs/reviews/](docs/reviews/) | 比赛手册与协议原件，以及 2026-11-21 审查的脚本、日志和结果。历史脚本对应当时的仓库与环境，当前回归入口见 [test/](test/)。 |

## 来源与范围

采用浮舟湿地/COD 的 ROS 2 → 黑板 → BehaviorTree.CPP 4 → 输出适配结构。本包是独立新入口，原 `cod_behavior`、本地整合树、北极熊导航、串口驱动均未替换。

父工程基线为 `fzsd_xky` 提交 `6c65a992b0bb029f9cd83db6e6d6136f92af6e75`。本包借鉴其架构组织与持守、恢复设计。

规则及协议原件见 [docs/rules](docs/rules/)。本包依据 2027 联盟赛比赛规则 V1.0.0 和 2027 通信协议 V1.0.0。

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
- `DecisionState.msg`：本车状态、两台队友观察、候选任务与分组时间。阶段剩余秒数来自 `0x0001`，双方控制度来自 `0x0003`，中央区双方激活标志来自 `0x0101`。控制度与激活各有独立时间戳和有效标志。更新接口后应重新构建引用该消息的节点。
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

## 执行与异常处理

裁判适配器配置 UART 后只读接收，断开时清除旧状态并重试。导航桥接检查目标与最新路线，失败时按退避间隔最多尝试三次。证据失效会撤销当前目标，等待 Nav2 返回终态后才会发新目标。

Nav2 负责规划和速度输出。live 模式由导航包的 MotionGate 检查传感器、姿态、手动使能和任务心跳，再经车内底盘通信执行。

整合入口默认 `enabled:=false`、`execute_navigation:=false`，采用 `fixed_map`，Action 为 `/red_standard_robot1/navigate_to_pose`。当前 live 入口对应 `navigation_backend:=live` 和 `/navigate_to_pose`，需由 02 的启动入口选择 `goal_owner:=decision`。此模式禁用自带探索器，MotionGate 仅接收决策节点的活动心跳，保留原手动使能与底盘保护。测试中不启用实车执行。

赛场点位、禁行区、经核验的路线证据发布端、雷达外参和真实裁判数据接线仍需实车验收。官方协议的 `0x020B` 不能直接作为 RMUL 队友定位来源。本车可以基于可靠的本地观察补充，缺少证据时保留未知。

### 3V3 场地底图

[RMUL2027_3V3_V1.0.0.stp](maps/RMUL2027_3V3_V1.0.0.stp) 是用于标注的场地 CAD 原件，长度单位为毫米，地面坐标在模型的 X–Z 平面。补给和控制区等区域按联盟赛比赛规则手册 §3.2 标注。将场地几何投影到二维后，还需与实测雷达地图的原点和朝向对齐，才能确定 ROS `map` 帧下的点位及禁区。完成标定后将目标写入 `config/approved_tasks.yaml`，并为 Nav2 生成对应的禁区地图。

[离线红方地图](maps/offline/red_map.yaml) 和 [离线点位](maps/offline/red_tasks.yaml) 可用于没有实体场地时的预览和栅格规划测试。坐标取自规则手册图 3-5、图 3-6 的简化俯视布局，中心为原点，X 向图右，Y 向图上，单位为米。栅格分辨率为 0.05 m。高地及隧道所占区域、工作台、蓝方补给区和场地外部均标为占用。离线样例用于软件验证，默认装车点位表仍须使用实际场地坐标。

决策发送的是车内导航目标。裁判系统的 `0x0120` 属于另一条链路，本包的 UART 不发送上行帧。当前仓库底盘源码的周期上报与实车烧录版本需单独核对。

### 自瞄状态接入

在同一台 Ubuntu 小电脑上重编译自瞄的 `auto_aim_test` 以及本决策包。自瞄的 `scripts/run_live_usb_sp.sh` 会启用状态输出，通过本机 UDP `127.0.0.1:9871` 发送跟踪状态和原始帧时间。决策整合入口默认接收，并在 `/decision2027/state` 和 `/decision2027/intent` 中显示 `vision_status`。

| 值 | 跟踪状态 |
| --- | --- |
| 0 | 未知。无数据、姿态不可用或帧已过期 |
| 1 | 未发现目标 |
| 2 | 已发现，等待稳定确认 |
| 3 | 稳定跟踪 |
| 4 | 暂时丢失 |
| 5 | 正在切换目标 |

状态超过 0.5 秒失效。使用仿真时钟时不接收实车视觉状态。启用 `vision_status:=false` 可在启动时关闭接收。该字段用于状态观察，现有导航优先级和云台、发射控制保持原有接口。

在已加载决策包安装环境的终端查看决策节点输出。

```bash
ros2 topic echo /decision2027/intent --field vision_status
```

## 整车启动与导航反馈

`sentry_bringup.launch.py` 统一启动状态接收、BehaviorTree.CPP 行为树、导航桥和路线证据节点。行为树从 `DecisionState` 选任务 ID，桥接器校验点位后把目标交给 Nav2。独立启动的视觉模块通过本机 UDP 提供观测状态。策略优先级在 `policy.hpp`，XML 组织状态读取、选点和发布。

总启动的组织方式参考 [北极熊上位机](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_ws#24-running)。车模和录包也沿用它的模块化做法，参数与话题对应本队工程。

### 仿真或已有导航进程

在同一 ROS 域启动仿真和导航，再加载本包的安装环境。此入口只预览目标。

```bash
decision_share=$(ros2 pkg prefix --share rm2027_decision)
ros2 launch rm2027_decision sentry_bringup.launch.py use_sim_time:=true \
  nav_namespace:=red_standard_robot1 base_frame:=gimbal_yaw_fake \
  tasks_file:="$decision_share/offline/red_tasks.yaml"
```

离线点位与本包离线 3V3 地图配套。将经核对的禁区栅格通过 `route_map_topic` 提供给证据节点，再设 `protected_map_confirmed:=true`。证据节点默认请求导航命名空间下的 `decision_route/compute_path_to_pose`。可用 `planner_action` 指定独立规划服务，避免与正在执行目标的 Nav2 互相抢占。通过 `tf_topic` 和 `tf_static_topic` 对齐仿真话题。若没有模型发布器，可用 `robot_urdf` 加载本队 URDF。

```bash
ros2 topic echo /decision2027/navigation_evidence --qos-reliability best_effort
ros2 topic echo /decision2027/intent
ros2 topic echo /decision2027/navigation_status
```

### 实车实时导航入口

先构建 `02_导航` 的 `rm_live_nav`。在仓库根目录加载导航和决策安装环境，再指定本队安装参数。

```bash
source '02_导航（含通信）/scripts/local_env.sh'
source ~/rm2027_decision_ws/install/local_setup.bash
ros2 launch rm2027_decision sentry_bringup.launch.py mode:=live \
  source_root:="$RM_NAV_RUNTIME/source" runtime:="$RM_NAV_RUNTIME" \
  mounting_config:="$RM_NAV_RUNTIME/source/local/live_mounting.yaml" \
  output_dir:="$RM_NAV_ROOT/local/maps/$(date +%Y%m%d_%H%M%S)"
```

此入口选择 `goal_owner:=decision`，由 `06_决策` 统一选点。雷达启动前需要核对网络地址。默认只预览，底盘 USB 和裁判串口均关闭。执行前需配置本车 `robot_id`、独立的 `referee_uart`、`serial_port`、实测点位和 `keepout_map`，再显式设置 `connect_chassis`、`protected_map_confirmed`、`execute_navigation`。底盘和运动门控仍沿用原使能流程。

`keepout_map` 使用 Nav2 栅格地图 YAML，标注不可通行区域、边界和实车尺寸所需余量。实时 SLAM 以本次启动位姿建立原点，每次启动须核对地图原点和车头方向。各批准任务点需填写 `risk`，风险未评估的点不会形成可执行证据。Nav2 重规划与证据节点使用同一份禁区图，证据节点还会检查路径周围是否留出车体所需空间。

### 车模与 TF 预览

`robot_description/sentry_preview.urdf.xacro` 从同目录 `offline_geometry.yaml` 读取尺寸、关节和传感器位置。同一份描述可生成 ROS 的 URDF 和 Gazebo Fortress 的 SDF。离线模型用于检查几何、TF 和模型加载。驱动、雷达数据和实际里程计由各自的模拟或实车节点提供。示例关节和安装角不可用作实车外参。[北极熊机器人描述与格式转换](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description#24-running)

```bash
ros2 launch rm2027_decision sentry_bringup.launch.py robot_model:=offline \
  nav_namespace:=/ vision_status:=false
ros2 run tf2_ros tf2_echo base_footprint front_camera_optical
```

在 RViz 中可把固定坐标系设为 `base_footprint` 后查看 RobotModel。需要把示例模型交给 Gazebo Fortress 时，在已加载安装环境的 Ubuntu 终端生成两种格式。

```bash
decision_share=$(ros2 pkg prefix --share rm2027_decision)
model_dir=$(mktemp -d)
xacro "$decision_share/robot_description/sentry_preview.urdf.xacro" > "$model_dir/sentry.urdf"
ign sdf -p "$model_dir/sentry.urdf" > "$model_dir/sentry.sdf"
```

### 联调录包与回放

在启动命令中加 `record_bag`，退出启动进程时保存。默认记录 TF、地图、里程计、决策输入与意图、导航结果。实时模式也记录扫描和底盘诊断，录原始点云和 IMU 时加 `record_lidar:=true`。每个存储分段约 256 MiB，录制会持续到退出。[北极熊录包与回放](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_ws#33-rosbags)

```bash
bag_dir="$PWD/sentry_$(date +%Y%m%d_%H%M%S)"
ros2 launch rm2027_decision sentry_bringup.launch.py robot_model:=offline \
  nav_namespace:=/ vision_status:=false record_bag:="$bag_dir"
```

回放时用同一个空闲 ROS 域，在两个终端分别运行下面两条命令。第一条只启动决策预览，第二条只回放决策输入。回放所得 `intent` 可以与原录包中的结果对比。

```bash
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 ros2 launch rm2027_decision intents.launch.py \
  enabled:=true use_sim_time:=true
ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 ros2 bag play "$bag_dir" --clock \
  --topics /decision2027/state
```

### 行为树怎么读

`intents.xml` 依次运行 `ReadNormalizedState`、`Evaluate2027TaskPriority` 和 `PublishTaskIntent`。树的端口定义可用 Groot 查看。`policy.hpp` 负责规则与状态记忆。它先检查输入和比赛状态，再判断定位与路线，最后按健康状态和当前任务选择等待、补给、守点或巡逻等模式。`/decision2027/intent` 给出 `mode`、`target`、`reason`，用这三项对照同一条录包里的 `state` 即可追查为什么选点或等待。参考 [北极熊 2025 联盟赛行为树](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior/blob/main/behavior_trees/rmul_2025.xml)。

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

加载构建后的 ROS 安装环境后，可在仓库根目录运行离线闭环回归。它用随仓库提供的 3V3 栅格和真实 Navfn 规划器验证可达性，用模拟 Action 服务接收导航目标并验证定位过期撤销。

```bash
ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 '06_决策/test/navigation_closed_loop.py'
```

已编写断言：本车门控和 HP 滞回、八种团队低血组合、队友滞回/最大血量、死亡/虚弱/未知、位置防堆叠、持守与交接、岗位漏项/争夺中不离岗、地形/路线硬过滤、风险预算/非法数值、无图等待与全低血不排队。`freshness_test.cpp` 检查重复来源时间、暂停、未来/乱序时间及超时边界。

原始审查记录保存在 [docs/reviews/20261121](docs/reviews/20261121/)。实际裁判串口、实车 Nav2 执行、场地禁区、长时间联合运行和实车停车仍待验收。控制度、阶段余时与中央区激活当前用于状态观察，尚未参与目标排序。裁判帧中的控制归属与激活标志不能给出双方完整占领进度。
