# 两块 C 板分别烧录：MuseLab + EmberProbe

只需要一个下载器，依次连接两块板。每次只烧录当前 SWD 接线所连接的那块板。小电脑的 USB 经云台板再接 CAN，不能直接替底盘板下载固件；本工程没有 CAN bootloader。

## 一、先解决下载器连接

此前的 `CMSIS-DAP command CMD_INFO failed` 表示下载器信息查询失败，尚未进入 Flash 写入。继续反复点下载不能证明已烧录成功。

停止其他调试、采样和烧录任务，重新插拔下载器。无线下载器要让连接 C 板 SWD 的远端也单独供电，按厂商说明确认配对成功。可先执行：

```bash
python3 /home/wangjiaming/桌面/c_board_nav_gateway_20261111/tools/query_muselab_info.py
```

该脚本只查下载器信息，不烧录、不暂停、不复位 C 板。收到 PASS 后才能说明探针查询恢复；仍需下一步识别 STM32。失败则按《烧录报错CMD_INFO排查》处理。

## 二、烧录云台 C 板

1. 断开底盘/云台电机主电源，给目标 C 板保留正确的独立供电。第一次可只接这一块板，暂不接板间 CAN，便于区分。
2. 把下载器 SWD 接到**云台 C 板**：SWDIO→SWDIO、SWCLK→SWCLK、GND→GND。针序按板卡和下载器丝印/手册，不能按线色猜。若下载器有 VREF 电压检测输入，按其手册接目标 3.3V；不要把下载器的电源输出与已经独立供电的板子电源输出直接并联。
3. 在 EmberProbe 侧栏选择探针 `CMSIS-DAP-MuseLab / 0001A0000002`。当前 EmberProbe 0.9.2 的 Agent 配置中，探针脚本选择：

   ```text
   cmsis-dap.cfg
   ```

   Agent 仅允许软件 OpenOCD scripts 目录中的相对脚本名，不接受工作区的绝对路径。因此不要把 `tools/openocd_muselab_hid.cfg` 的绝对路径填入本版 Agent 的探针字段；当前保留它作为旧排查参考。

4. 目标选择 `stm32f4x.cfg`（对应安装目录的 `target/stm32f4x.cfg`），传输方式 SWD，设置 `emberprobe.adapterSpeedKhz=100`。OpenOCD 必须 >= 0.12.0；系统旧版 0.11.0 不通过本版 EmberProbe 预检查。若设置仍为 `openocd`，请由用户在 `emberprobe.openocdPath` 中选择 EmberProbe 自带的新版实际可执行文件，此字段禁止 Agent 代改。
5. 选择云台固件：

   ```text
   /home/wangjiaming/桌面/c_board_nav_gateway_20261111/artifacts/gimbal_nav_converter.hex
   ```

   如果 EmberProbe 的文件入口只支持 ELF，选择同目录的 `gimbal_nav_converter.elf`；HEX 和 ELF 是同一份程序，**选其中一个烧录即可**。不需要把文件内容复制到编辑器，也不需要现场重新编译。
6. 点击侧栏“下载/固件下载”。也可在资源管理器中右键同名 ELF，使用 EmberProbe 下载。先确认当前 SWD 连的是云台板：操作会覆盖板内原程序，若要保留需先备份可读 Flash 或准备好原固件。
7. 等待写入、校验和复位运行全部成功。仅看到“识别下载器”或 LED 亮不算烧录完成。若处于调试暂停状态，先结束调试/恢复运行，再检查 USB。

SWD 下载一般不需要改 BOOT 跳线；保持正常 Flash 启动配置。两板都使用 STM32F407，目标脚本相同，固件文件不同。

## 三、烧录底盘 C 板

1. 结束当前调试/采样会话，断电后，把下载器的 SWD 线从云台板移到**底盘 C 板**，重新核对 SWDIO、SWCLK、GND 及供电。
2. 给底盘 C 板和无线下载器远端正确供电，电机主电源仍保持断开。
3. 在 EmberProbe 刷新目标芯片识别。若工具记住了上一块芯片、提示芯片身份变化，确认当前确实连接底盘板后选择“改用当前芯片”；不要沿用旧芯片状态强行下载。
4. 探针和 STM32F4 目标脚本保持相同，只把固件换成：

   ```text
   /home/wangjiaming/桌面/c_board_nav_gateway_20261111/artifacts/chassis_nav_can.hex
   ```

   只支持 ELF 的入口用同目录 `chassis_nav_can.elf`。确认当前连接底盘板后点击下载，等待写入、校验和复位成功。

两份 HEX/ELF 都带 Flash 地址，不用手填烧录地址。若另用 BIN 下载工具，两个 BIN 的起始地址都为 `0x08000000`。

## 四、烧录完成后测试

先断电，按实际接线恢复：

```text
小电脑 USB → 云台 C 板自身 Micro-USB 数据口
云台 C 板 CAN1 H/L/GND → 底盘 C 板 CAN2 H/L/GND
```

云台 CAN1 和底盘 CAN2 均配置为 1Mbps，按总线物理末端配置终端电阻。供电和完整接线要求见《接线烧录与测试》。

启动正常运行后，查看应用 USB 串口：

```bash
ls -l /dev/serial/by-id/
```

云台固件产品名应为 `RM Nav Convert SP`。`CMSIS-DAP-MuseLab` 对应下载器，不是应用通信端口。

关闭其他物理串口程序，用实际云台应用设备名替换占位符：

```bash
bash /home/wangjiaming/桌面/c_board_nav_gateway_20261111/pc/run.sh zero /dev/serial/by-id/实际云台板设备名
```

测试只发送 STOP，收到持续新鲜底盘 ACK 才证明 USB→云台→CAN→底盘及反馈链路工作。两份交付固件默认关闭电机运动输出，烧录完成并不意味着可直接行驶；完整导航仍需轴向确认和云台几何/零位标定。

## 2026-11-11 接口调整

当前交付固件已改为云台 CAN1 ↔ 底盘 CAN2。此前烧入云台的 CAN2 版需要重烧当前 `artifacts/gimbal_nav_converter.elf`。底盘如已烧入本项目同一 v1 协议的 CAN2 固件，可继续兼容；若是旧哨兵原固件或尚未烧录，需要烧入 `artifacts/chassis_nav_can.elf`。无需重新安装 STM32 HAL 库或修改电脑 USB 协议。旧 CAN2↔CAN2 固件已移入 `artifacts/history/gimbal-CAN2-to-chassis-CAN2`，仅用于历史比对。
