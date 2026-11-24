# 烧饼哨兵工程

本仓库包含小电脑上的自瞄、导航和决策，以及云台、底盘两块 C 板的固件。上位机联调从 [总启动说明](06_决策/README.md#整车启动与导航反馈) 开始。总启动可接入已有仿真或启动实时导航，默认预览决策结果。自瞄按自身入口启动，通过本机接口向决策提供视觉状态。

## 中文目录

按代码的运行位置查找。中文名称用于阅读，反引号内为仓库中的实际目录名。

| 运行位置 | 中文分类 | 英文名称 | 项目说明 |
| --- | --- | --- | --- |
| Ubuntu 小电脑 | [01 自瞄](#01-自瞄) | Auto Aim | [自瞄 README](01_自瞄（含通信）/readme.md) |
| Ubuntu 小电脑 | [02 导航](#02-导航) | Navigation | [导航 README](02_导航（含通信）/README.md) |
| Ubuntu 小电脑 | [06 决策](#06-决策) | Decision Making | [决策 README](06_决策/README.md) |
| 云台 C 板 | [05 云台电控](#05-云台电控) | Gimbal Firmware | [云台 README](05_云台/README.md) |
| 底盘 C 板 | [04 底盘电控](#04-底盘电控) | Chassis Firmware | [底盘 README](04_底盘电控（含通信）/README.md) |

### 01 自瞄

| 中文分类 | 目录 | 内容 |
| --- | --- | --- |
| 视觉任务与源码 | [`tasks/`](01_自瞄（含通信）/tasks/)、[`src/`](01_自瞄（含通信）/src/) | 视觉任务入口、算法及公共代码 |
| 相机与电控通信 | [`io/`](01_自瞄（含通信）/io/) | 相机驱动和小电脑端的电控收发代码 |
| 参数与标定 | [`configs/`](01_自瞄（含通信）/configs/)、[`calibration/`](01_自瞄（含通信）/calibration/) | 运行配置、相机和坐标系标定 |
| 资源与说明 | [`assets/`](01_自瞄（含通信）/assets/)、[`docs/`](01_自瞄（含通信）/docs/) | 配套资源及使用文档 |
| 脚本、工具与测试 | [`scripts/`](01_自瞄（含通信）/scripts/)、[`tools/`](01_自瞄（含通信）/tools/)、[`tests/`](01_自瞄（含通信）/tests/) | 启动、诊断及验证程序 |

### 02 导航

| 中文分类 | 目录 | 内容 |
| --- | --- | --- |
| 实车建图与导航 | [`src/rm_live_nav/`](02_导航（含通信）/src/rm_live_nav/) | 本队使用的雷达建图、定位与导航启动 |
| 北极熊导航源码 | [`src/pb2025_sentry_nav/`](02_导航（含通信）/src/pb2025_sentry_nav/) | 引入的导航算法和 ROS 2 功能包 |
| 小电脑端底盘通信 | [`src/rm_chassis_comm/`](02_导航（含通信）/src/rm_chassis_comm/) | 导航程序与电控之间的收发 |
| 本机参数与数据 | [`local/`](02_导航（含通信）/local/) | 安装参数、地图、备份与测试记录 |
| 部署脚本、资料及依赖 | [`scripts/`](02_导航（含通信）/scripts/)、[`docs/`](02_导航（含通信）/docs/)、[`外部依赖源码/`](02_导航（含通信）/外部依赖源码/) | 环境与运行入口、安装和验证文档、第三方驱动源码 |

### 06 决策

| 中文分类 | 目录 | 内容 |
| --- | --- | --- |
| 整车联调启动 | [`launch/`](06_决策/launch/) | 总启动、决策预览及 ROS 2 节点编排 |
| 行为树 | [`behavior_trees/`](06_决策/behavior_trees/) | XML 定义读取状态、选择任务和发布意图的流程 |
| 决策节点与规则 | [`src/`](06_决策/src/)、[`include/`](06_决策/include/) | C++ 节点和任务优先级，具体规则见 `policy.hpp` |
| 消息与输入适配 | [`msg/`](06_决策/msg/)、[`scripts/adapters/`](06_决策/scripts/adapters/) | 决策消息、裁判与视觉状态接收 |
| 导航接口 | [`scripts/navigation/`](06_决策/scripts/navigation/) | 路线证据及 Nav2 桥接 |
| 参数与任务点 | [`config/`](06_决策/config/) | 决策阈值和实车批准任务点位 |
| 车模与地图 | [`robot_description/`](06_决策/robot_description/)、[`maps/`](06_决策/maps/) | 离线车模、场地 CAD、离线地图与示例点位 |
| 文档、规则与审查 | [`docs/`](06_决策/docs/)、[`docs/rules/`](06_决策/docs/rules/)、[`docs/reviews/`](06_决策/docs/reviews/) | 工程说明、比赛规则及协议 PDF、历史审查 |
| 测试 | [`test/`](06_决策/test/) | 策略和导航联调测试 |

### 05 云台电控

| 中文分类 | 目录 | 内容 |
| --- | --- | --- |
| 应用与功能模块 | [`application/`](05_云台/application/)、[`modules/`](05_云台/modules/) | 云台应用任务与各功能模块 |
| 云台板端通信 | [`modules/nav_conversion/`](05_云台/modules/nav_conversion/) | 导航通信接收与板间转发 |
| 板级支持与驱动 | [`bsp/`](05_云台/bsp/)、[`Drivers/`](05_云台/Drivers/)、[`Middlewares/`](05_云台/Middlewares/)、[`Inc/`](05_云台/Inc/)、[`Src/`](05_云台/Src/) | 板级接口、外设驱动、中间件、头文件及初始化源码 |
| 构建产物与固件发布 | [`artifacts/`](05_云台/artifacts/)、[`release/`](05_云台/release/) | 编译输出和发布固件 |
| 说明、测试与日志 | [`docs/`](05_云台/docs/)、[`.Doc/`](05_云台/.Doc/)、[`tools/`](05_云台/tools/)、[`tests/`](05_云台/tests/)、[`logs/`](05_云台/logs/) | 工程说明、构建和验证工具、联调日志 |

### 04 底盘电控

| 中文分类 | 目录 | 内容 |
| --- | --- | --- |
| 当前底盘固件与通信 | [`chassis_driver/basic_framework/`](04_底盘电控（含通信）/chassis_driver/basic_framework/) | 底盘 C 板的现用源码与构建工程 |
| 历史通信参考 | [`communication_reference/`](04_底盘电控（含通信）/communication_reference/) | 历史协议和参考工程 |
| 联调工具与测试 | [`tools/`](04_底盘电控（含通信）/tools/)、[`tests/`](04_底盘电控（含通信）/tests/) | 小电脑联调工具及底盘回归测试 |
| 固件发布与构建产物 | [`release/`](04_底盘电控（含通信）/release/)、[`artifacts/`](04_底盘电控（含通信）/artifacts/)、[`build/`](04_底盘电控（含通信）/build/) | 发布固件、保留的编译输出及本机构建目录 |
| 文档与运行记录 | [`docs/`](04_底盘电控（含通信）/docs/)、[`logs/`](04_底盘电控（含通信）/logs/)、[`tmp/`](04_底盘电控（含通信）/tmp/) | 说明、日志与临时数据 |

## 模块之间的数据流

- **任务选择。** 裁判数据和导航证据汇总为 `DecisionState`，自瞄跟踪状态也汇入其中供联调观察。行为树输出 `DecisionIntent`，记录任务模式、目标和原因。
- **导航执行。** 获准执行的目标由决策桥交给 Nav2。Nav2 负责规划和速度输出，小电脑端的 `rm_chassis_comm` 负责与电控通信。采用云台转发方案时，小电脑到云台 C 板使用 USB，云台 C 板到底盘 C 板使用 CAN。
- **车模和定位。** [离线车模](06_决策/robot_description/sentry_preview.urdf.xacro) 用于检查 Gazebo 模型和 TF（坐标系变换）。[实车雷达安装参数](02_导航（含通信）/local/live_mounting.yaml) 随导航维护。[离线场地地图与点位](06_决策/maps/offline/) 用于决策和规划测试。

## 决策代码查阅顺序

1. 从 [sentry_bringup.launch.py](06_决策/launch/sentry_bringup.launch.py) 查看启动了哪些节点。
2. 看 [intents.xml](06_决策/behavior_trees/intents.xml) 怎样读取状态、调用策略和发布意图，再到 [policy.hpp](06_决策/include/rm2027_decision/policy.hpp) 查具体的判断顺序。
3. 到 [intent_nav_bridge.py](06_决策/scripts/navigation/intent_nav_bridge.py) 查看目标如何送给 Nav2、何时撤销。
4. 用 [联调录包与回放说明](06_决策/README.md#联调录包与回放) 将一次运行的输入、意图和导航状态对应起来。

## 北极熊上位机目录参考

北极熊在总仓库中维护 `pb2025_sentry_bringup` 和 `dependencies.repos`。导入依赖后，视觉、导航、行为树、串口通信、车模和仿真各自有独立源码目录，公共消息归入 `src/interfaces/`，外部依赖归入 `src/dependencies/`，录包和地图转换工具归入 `src/tools/`。总启动文件调用子模块入口，并通过 `node_params.yaml` 传递统一参数。[依赖与目录清单](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_ws/blob/main/dependencies.repos) · [总启动源码](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_ws/blob/main/src/pb2025_sentry_bringup/launch/bringup.launch.py)

这里是烧饼的所有文件，上传文件时记得附上姓名和日期
