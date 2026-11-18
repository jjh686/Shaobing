# 实车云台坐标与通信接入验证

日期：2026-11-09。版本0.2.0。所有执行均为WSL构建、主机模型、ROS节点与虚拟PTY；未打开物理串口/CAN、未运行车辆或烧录。

## 已通过

| 项目 | 方法 | 结果 |
| --- | --- | --- |
| rm_chassis_comm | ROS2 Humble、C99/C++17、Release构建 | 编译/链接通过 |
| fake_vel_transform | 实车导航修改组件，Release构建 | 编译/链接通过 |
| sensor_scan_generation | 实车缺TF处理修改组件，Release构建 | 编译/链接通过 |
| 通信及坐标测试 | 9项CTest，Release仍执行assert | 9/9通过 |
| 导航源节点测试 | 真实FakeVelTransform与ROS发布/订阅，同进程 | 1/1通过 |
| 新云台固件 | STM32F407 ARM GCC Release/O2 | 链接通过 |
| MCU适配器 | 真实gateway/chassis C代码与HAL模型 | 两项通过 |
| 原云台PI/驱动 | 原PI14组、DJI routing、原7组+新增编码器快照 | 全部通过，快照共8组 |

9项CTest包括原协议、MCU门控、CAN、单位适配、新关节46B协议、三类帧分流、坐标数学、四时间戳映射、真实ROS节点+PTY。最后一项实际创建ChassisNode、SentryFrameBridge和虚拟MCU端，验证关节反馈→JointState→动态TF、yaw=90°方向、π附近旋转不重复、显式使能、新命令、源时间戳过期、无效标定、采样重放和反馈断流停止。

导航源测试验证原odom时刻/云台frame保留、一输入仅一带戳输出、旧odom/plan不重新包装运动、伪坐标TF不重写旧角度时间、odom停顿后零指令清缓存、过期输入拒绝。驱动另验证启用前排队的带戳命令不能在启用后执行。三类分流包含SP43、底盘34、关节46混合与坏CRC恢复。

四时间戳测试使用数学时钟模型；诊断不确定度仅为网络RTT/2分量，MCU1ms量化、非对称链路、晶振漂移与真实调度需实测。ROS/PTY回归的几何和标定为测试输入，不是实车标定结果。

云台静态链接统计：RAM84776B/128KiB（64.68%），FLASH95564B/1MiB（9.11%）；不代表运行时堆栈余量或实时性已验证。

## 实车入口检查

`ros2 launch rm_chassis_comm sentry_reality.launch.py --show-args`通过。默认connect=false、geometry_confirmed=false、start_navigation=false；3秒启动检查中两个节点正常运行，显示禁止运动及未确认几何，随后SIGINT干净退出。

## 本机尚未通过

独立的Python测试端与C++驱动处于不同进程时，route1/route2均能通过PTY收到约188帧STOP，但ROS实测速度和诊断为0，测试未通过。此前独立双进程ROS探针也存在同类DDS通信问题；本轮没有修改全局网络或ROS配置。

因此，同进程ROS/PTY测试不能替代跨进程、完整Nav2栈或实车总线验收。交付的是已经编译和测试的源代码实现；本机跨进程演示仍需解决DDS障碍。

## 复现

```bash
source /opt/ros/humble/setup.bash
nav='/mnt/c/Users/zbh26/Desktop/code/robomaster/导航'
build="$HOME/.cache/rm_sentry_gimbal_check"
source "$build/install/local_setup.bash"
cmake -S "$nav/rm_chassis_comm" -B "$build/build/rm_chassis_comm" -DBUILD_TESTING=ON
cmake --build "$build/build/rm_chassis_comm" -j2
ctest --test-dir "$build/build/rm_chassis_comm" --output-on-failure
cmake -S "$nav/pb2025_sentry_nav/fake_vel_transform" -B "$build/build/fake_vel_transform" \
  -DBUILD_TESTING=OFF -DSENTRY_LINK_TESTING=ON
cmake --build "$build/build/fake_vel_transform" -j2
ctest --test-dir "$build/build/fake_vel_transform" --output-on-failure
```

FakeVelTransform测试显式优先加载本次build目录的共享库，避免LD_LIBRARY_PATH指向旧overlay而误测旧实现。原工程可选lint工具不属于SENTRY_LINK_TESTING。

## 实物验收缺项

固定ECD零位与关节符号、两转轴位置与固定雷达/相机外参、本地安全许可、轮径/减速比/轮序、CAN口/ID和停止响应均待确认。云台原±120°有限包络与连续小陀螺的协调、人工驾驶通道和交战策略未在本轮实现。

原下载ZIP、电控原件和自瞄算法未改写。修改前备份为版本020_20261109-184147。整体架构与审阅顺序见SENTRY_ARCHITECTURE.md，跨组件补丁为导航目录下的哨兵实车云台坐标接入-20261109.patch。
