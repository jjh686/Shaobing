# DAPLink 烧录与调试（Ubuntu 22.04 / STM32F407IG）

本工程使用 GNU Arm 工具链编译，由 Cortex-Debug 调用 OpenOCD，通过 DAPLink 的 SWD 接口连接 STM32F407IG。当前探针是 PUSO_DAPLink，USB ID 为 `1234:5678`。

Linux 下此 USB ID 会被 `usbtouchscreen` 驱动识别并占用 CMSIS-DAP v2 接口，因此配置固定使用探针自带的 CMSIS-DAP v1/HID 接口，SWD 速度为 1000 kHz。

## 首次安装

在 VS Code 中点击“终端 → 运行任务”，选择 `setup daplink`，在底部终端输入 sudo 密码。该任务安装系统工具及 USB 权限规则。也可以在工程根目录手动运行以下命令：

```bash
bash tools/setup_daplink_ubuntu.sh
code --install-extension marus25.cortex-debug
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.makefile-tools
```

安装脚本会请求 sudo 密码，安装编译调试工具及当前探针的 USB 权限规则。密码输入时不会显示字符。脚本通过 `tools/ubuntu-jammy.list` 临时使用 Ubuntu 22.04 官方源，避免无关的 ROS 等第三方源失效阻塞安装；这些 APT 选项只对本次安装生效。安装后可运行 VS Code 的“Developer: Reload Window / 开发人员: 重新加载窗口”。Linux 工具路径在 `.vscode/settings.json` 中配置为 `/usr/bin`；同时保留原来的 Windows / MSYS2 路径设置。

## 接线

| DAPLink | STM32F407IG |
| --- | --- |
| SWDIO | PA13 / SWDIO |
| SWCLK | PA14 / SWCLK |
| GND | GND（必须共地） |
| VTref（若探针需要） | 目标板 3.3 V 参考电压 |
| nRESET（可选） | NRST |

目标板需要正常供电。探针上标为 3.3 V 的引脚可能是电源输出或电压参考输入，按探针说明接线。

## 检查连接

在工程根目录执行：

```bash
lsusb -d 1234:5678
make probe_dap
```

也可以运行 VS Code 任务 `check dap`。该任务打开连接并识别目标，不擦写 Flash、不复位。看到 CMSIS-DAP、SWD DPIDR、Cortex-M4 等信息说明连接成功。

若 USB 访问被拒绝，安装工程提供的设备规则：

```bash
sudo install -m 644 tools/60-puso-daplink.rules /etc/udev/rules.d/60-puso-daplink.rules
sudo udevadm control --reload-rules
```

随后拔插 DAPLink。规则只匹配 `1234:5678`，允许 `plugdev` 组和当前本地登录用户访问。

## 日常操作

| 操作 | 入口 |
| --- | --- |
| 首次安装系统工具及 USB 权限 | 任务 `setup daplink` |
| 验证编译、调试符号和芯片连接 | 任务 `verify daplink`，或 `bash tools/verify_daplink.sh` |
| 编译、烧录、校验并复位运行 | `Ctrl+Shift+B`，或 `make download_dap` |
| 编译、烧录并停在 `main` 调试 | 选择 `DAPLink：烧录并调试`，按 `F5` |
| 附加到已有固件 | 选择 `DAPLink：附加调试（不烧录）`，按 `F5` |
| 只编译 | 任务 `build task`，或 `make -j16 all` |
| 清理后重新编译 | 任务 `clean rebuild`，或 `make rebuild` |

附加调试时，`build/basic_framework.elf` 必须与板上固件一致。附加不会重新编译、下载，但调试器连接时可能暂停 CPU。调试会话占用 DAPLink，停止调试后再执行独立烧录任务。

启动调试后可使用 F9 设置断点、F10 单步跳过、F11 单步进入、F5 继续，以及“监视”窗口查看变量。FreeRTOS 线程查看和外设寄存器视图已配置。`watch flash` / `watch build only` 保留原有 Windows PowerShell 脚本，只适用于 Windows。

## 连接失败

- 找不到探针：确认 `lsusb` 中有 DAPLink，并检查 USB 访问权限。
- 找到探针但无法识别芯片：检查目标板供电、共地、SWDIO / SWCLK 接线；可将 `openocd_dap.cfg` 的 `set DAP_SPEED_KHZ 1000` 改成 `set DAP_SPEED_KHZ 100`，启动和复位阶段都会使用该速度。
- 现有固件关闭了 SWD 或进入低功耗：接好 NRST，并按实际硬件尝试 connect-under-reset 配置。
- 工程移动后断点位置不对：执行 `make rebuild`，重新烧录后调试。
- 找不到 GDB / OpenOCD：检查 `/usr/bin/gdb-multiarch`、`/usr/bin/openocd` 是否存在，安装工具后重新加载 VS Code 窗口。
- 报 `undefined-gdb`，同时输出插件版本为 `undefined`：执行命令面板中的 `Developer: Reload Window`（开发人员: 重新加载窗口），让新安装的 Cortex-Debug 完成激活。`launch.json` 的 Linux 配置已明确指定 GDB、OpenOCD 和工具链路径。
