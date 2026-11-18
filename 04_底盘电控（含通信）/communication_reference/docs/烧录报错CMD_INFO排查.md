# EmberProbe 烧录出现 CMSIS-DAP command CMD_INFO failed

日期：2026-11-11。该报错出现在查询 CMSIS-DAP 下载器自身信息时，早于识别 STM32 和写入 Flash。不能据此判定云台代码错误或 C 板损坏。

本文 HID 超时记录来自早期排查。之后曾成功烧入云台 CAN2 版；改为云台 CAN1 接底盘 CAN2 后，再次查询芯片又报 CMD_INFO failed，未执行新版本写入。最新状态见《云台CAN1到底盘CAN2改动说明》。

本次已核对交付的 `gimbal_nav_converter.hex` 为正常 Intel HEX，Flash 地址从 `0x08000000` 开始，对应 ELF 是 ARM Cortex-M 工程。EmberProbe 输出中的 `/tmp/emberprobe-ui-flash-.../firmware.elf` 是插件使用的临时文件路径，仅凭这个路径不能判定选错固件。

## 本机当前检查结果

- 系统识别到 `CMSIS-DAP-MuseLab`，VID:PID=`c251:f001`，序列号 `0001A0000002`。
- 该 USB 设备同时有 CDC 串口接口和 HID 接口。出现 `ttyACM0` 并不表示它是云台 C 板的应用串口。
- 本次能够打开 HID、写入 DAP_Info 请求，但没有收到响应。已测试的 packet size/count、固件信息、能力查询均超时。
- 本次没有暂停、复位、擦除或写入 C 板。

## EmberProbe 应选择的配置

| 配置项 | 设置 |
|---|---|
| 调试器/探针脚本 | EmberProbe Agent 使用安装目录相对名 `cmsis-dap.cfg` |
| 目标芯片脚本 | `stm32f4x.cfg`（对应安装目录 target/stm32f4x.cfg） |
| 传输方式 | SWD |
| 链路速度 | 100 kHz；插件设置 `emberprobe.adapterSpeedKhz=100` |
| 选择的下载器 | CMSIS-DAP-MuseLab，序列号 `0001A0000002` |
| 云台固件 | 当前 VS Code 云台工作区选择 `gimbal_firmware/firmware_release/gimbal_nav_converter.elf`；HEX 留给支持 HEX 的其他工具 |

`tools/openocd_muselab_hid.cfg` 保留为早期 HID 排查参考。当前 EmberProbe 0.9.2 Agent 拒绝工作区绝对路径，不能将这个文件路径直接填入 debugger 字段；不要为此改动软件内部脚本。当前实际使用软件自带的 `cmsis-dap.cfg`。仅配置加载成功**不代表探针响应或硬件连接已经恢复**。EmberProbe 所选 OpenOCD 应为现有的 >=0.12.0 新版。

不要继续使用交付源码内历史 `openocd_dap.cfg` / `openocd_dap_hid.cfg` 注释中描述的旧 PUSO 探针身份来判断当前设备。本次专用配置明确针对 MuseLab。

100kHz 用于后续提高 SWD 链路容错；DAP_Info 是 USB 控制查询，**降速本身不能保证修复该报错**。

## 按顺序排查

1. 停止所有 OpenOCD、调试会话、实时采样和其他烧录程序，避免多个窗口同时使用同一个下载器。仅关闭编辑窗口不一定结束后台调试会话。
2. 将小电脑端下载器重新插拔，尽量直接接电脑机身 USB，先重新检测探针。
3. 如果使用成对无线下载器：小电脑端 USB 插入只给该端供电；连接 C 板 SWD 的远端也必须按其规格单独供电，并按厂商说明确认两端配对、信道和连接指示。C 板电源灯不能代替无线接收端供电/配对状态。
4. 先重新运行只查询下载器信息的脚本：

   ```bash
   python3 /home/wangjiaming/桌面/c_board_nav_gateway_20261111/tools/query_muselab_info.py
   ```

5. 若依旧提示未收到 DAP_Info 响应，优先继续检查下载器/无线连接/软件占用；暂时不改云台源代码，不执行擦除。可使用已知正常的有线 CMSIS-DAP 或 ST-LINK 对照，但 ST-LINK 需选择其对应探针脚本。
6. 查询恢复后，如果错误改为 `cannot read IDR`、`target not examined` 等，再检查 C 板供电、SWDIO、SWCLK、GND 和正常启动状态。接线调整先断电，针序按板卡和下载器丝印确认。
7. 真正重新烧录前断开电机主电源，保留目标板和下载器所需供电。两板分别烧录的对应关系见《接线烧录与测试》。

不要为了此错误盲目改 BOOT 跳线、短接不明针脚或执行全片擦除。当前错误尚未证明 STM32 SWD 连接已经建立。

## 2026-11-11 最新复查与弹窗显示调整

用户截图中的原生 VS Code 弹窗请求确认当前 EmberProbe skills 内容可访问 Agent Bridge，正文列出了各 skill 的版本和指纹。它不是下载成功提示。窗口管理器报告其最小、最大尺寸均为 719 × 2009，原桌面为 1920 × 1080，因此普通拖拽缩小不能完整显示。

按用户要求，已临时执行 `xrandr --output DP-2 --scale 2x2`，将逻辑桌面改为 3840 × 2160，使画面在原 1920 × 1080 显示器上缩小到一半。未设置开机持久化。随后截图确认原弹窗已经关闭；本轮未通过模拟点击或回车替用户作授权决定。需要恢复原显示大小时执行：

```bash
xrandr --output DP-2 --scale 1x1
```

本轮随后重新通过官方脚本读取云台工作区配置成功：ELF 为工作区内 `firmware_release/gimbal_nav_converter.elf`，脚本 `cmsis-dap.cfg` / `stm32f4x.cfg`，SWD 100 kHz，OpenOCD 为用户已设置的扩展内置新版。EmberProbe 调试状态为 `none`，采样为 `stopped`；未查到 OpenOCD、GDB 或占用 HID/ACM 的进程。

本轮单次 DAP_Info 查询仍失败：MuseLab `c251:f001`、序列号 `0001A0000002` 的 `/dev/hidraw0` 可打开且请求可写入，但 1 秒内没有响应。该结果证明本次下载器控制查询未通过，不能单凭它区分无线配对、下载器固件或设备内部状态。当前没有新版本烧录成功证据；本轮未执行 STM32 连接、暂停、复位、擦除或写入。下一步需恢复探针响应，或用已知正常的有线下载器作对照，再继续芯片识别和下载。
