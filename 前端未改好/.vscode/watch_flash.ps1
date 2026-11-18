<#
    源文件监听 -> 自动编译(可选自动烧录)
    用法(在工程根目录执行):
        powershell -NoProfile -ExecutionPolicy Bypass -File .vscode/watch_flash.ps1          # 保存即编译+烧录
        powershell -NoProfile -ExecutionPolicy Bypass -File .vscode/watch_flash.ps1 -NoFlash # 保存只编译
    停止: 在终端里按 Ctrl+C

    说明:
        1. 监听 application / modules / bsp / Src / Inc 下的 .c/.h/.s;
        2. 保存后会等 1.5s(QuietMs), 连续保存只触发一次;
        3. 编译失败不会烧录, 避免把旧固件或半成品刷进板子;
        4. 每次烧录都会复位单片机, 机器人运行中请勿开启自动烧录.
#>
# -NoFlash: 只编译, 不烧录
# -QuietMs: 最后一次修改后静默多久再动作(毫秒)
param(
    [switch]$NoFlash,
    [int]$QuietMs = 1500
)

$ErrorActionPreference = 'Continue'

# $PSScriptRoot = ...\.vscode, 上一级就是工程根目录
$root = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path (Join-Path $root 'Makefile'))) {
    Write-Host "[watch] 未找到 Makefile, 请把本脚本放在工程根目录的 .vscode 下" -ForegroundColor Red
    exit 1
}
Set-Location $root

$watcher = New-Object System.IO.FileSystemWatcher
$watcher.Path = $root
$watcher.IncludeSubdirectories = $true
$watcher.Filter = '*.*'
$watcher.NotifyFilter = [System.IO.NotifyFilters]::LastWrite -bor [System.IO.NotifyFilters]::FileName
$watcher.EnableRaisingEvents = $true

$mode = if ($NoFlash) { '仅编译' } else { '编译 + 烧录' }
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "[watch] 监听目录: application / modules / bsp / Src / Inc" -ForegroundColor Cyan
Write-Host "[watch] 模式: $mode   (Ctrl+C 退出)" -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan

while ($true)
{
    # 阻塞等待任意文件变化(1s 超时用于响应 Ctrl+C)
    $evt = $watcher.WaitForChanged([System.IO.WatcherChangeTypes]::All, 1000)
    if ($evt.TimedOut) { continue }

    $rel = [string]$evt.Name
    # 只关心源码; 忽略 build/ 产物和编辑器临时文件
    if ($rel -notmatch '\.(c|h|s)$') { continue }
    if ($rel -match '(^|[\\/])build([\\/])') { continue }
    if ($rel -match '(^|[\\/])\.') { continue }

    # 防抖: 连续保存时不断推迟, 直到安静 $QuietMs
    $deadline = (Get-Date).AddMilliseconds($QuietMs)
    while ((Get-Date) -lt $deadline)
    {
        $more = $watcher.WaitForChanged([System.IO.WatcherChangeTypes]::All, 200)
        if (-not $more.TimedOut) { $deadline = (Get-Date).AddMilliseconds($QuietMs) }
    }

    $time = Get-Date -Format 'HH:mm:ss'
    Write-Host ""
    Write-Host "[$time] 检测到修改: $rel -> 开始编译" -ForegroundColor Yellow

    make -j16
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[$time] 编译失败, 跳过烧录" -ForegroundColor Red
        continue
    }

    if ($NoFlash) {
        Write-Host "[$time] 编译完成(未烧录)" -ForegroundColor Green
        continue
    }

    Write-Host "[$time] 编译通过, 开始烧录..." -ForegroundColor Yellow
    make download_dap
    if ($LASTEXITCODE -eq 0) {
        Write-Host "[$time] 烧录完成, 固件已更新" -ForegroundColor Green
    } else {
        Write-Host "[$time] 烧录失败(调试器未连接?), 固件未更新" -ForegroundColor Red
    }
}
