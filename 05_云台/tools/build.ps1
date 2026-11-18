# Windows build entry for the gimbal firmware (05_云台).
# Linux keeps using: make -j16 all  (or `clean rebuild`), which produces the same
# build/basic_framework.elf that .vscode/launch.json points to.
#
# Why not just `make` on Windows:
#   1. no GNU make on PATH (only mingw32-make), and the Makefile uses POSIX shell commands;
#   2. the project path contains non-ASCII characters, which MSYS2 ld mangles when the
#      link line goes through a Ninja response file. Sources are referenced through an
#      ASCII junction under %LOCALAPPDATA% to avoid that.
#
# Output: build\basic_framework.elf / .hex / .bin / .map  (same path as the Makefile)
param(
    [switch]$Clean
)
$ErrorActionPreference = "Stop"

$root        = Split-Path -Parent $PSScriptRoot
$junctionDir = Join-Path $env:LOCALAPPDATA "rm-sentry-05"

$toolchainDir = if ($env:ARM_TOOLCHAIN_BIN) { $env:ARM_TOOLCHAIN_BIN } else { "D:\MSYS2\ucrt64\bin" }
$makeDir      = if ($env:MAKE_BIN_DIR)      { $env:MAKE_BIN_DIR }      else { "D:\MSYS2\mingw64\bin" }
$env:PATH     = "$toolchainDir;$makeDir;" + $env:PATH

function Resolve-Tool {
    param([string]$Name, [string]$ExeName)
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $roots = @(
        (Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages"),
        "C:\Program Files\CMake\bin",
        "C:\Program Files (x86)\CMake\bin",
        "C:\Program Files\Ninja",
        "C:\ninja"
    )
    foreach ($root2 in $roots) {
        if (-not (Test-Path $root2)) { continue }
        $hit = Get-ChildItem $root2 -Recurse -Depth 4 -Filter $ExeName -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($hit) { return $hit.FullName }
    }
    return $null
}

$cmake = Resolve-Tool -Name "cmake" -ExeName "cmake.exe"
if (-not $cmake) { throw "cmake not found. Install: winget install --id Kitware.CMake -e" }
$ninja = Resolve-Tool -Name "ninja" -ExeName "ninja.exe"
if (-not $ninja) { throw "ninja not found. Install: winget install --id Ninja-build.Ninja -e" }
if (-not (Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue)) {
    throw "arm-none-eabi-gcc not found. Set ARM_TOOLCHAIN_BIN to the folder containing it."
}

$env:PATH = "$(Split-Path -Parent $cmake);$(Split-Path -Parent $ninja);" + $env:PATH
Write-Host "cmake: $cmake"
Write-Host "ninja: $ninja"

# ASCII junction so no build path contains Chinese characters.
$junctionOk = $false
if (Test-Path $junctionDir) {
    $existing = (Get-Item $junctionDir).Target
    if ($existing) { $existing = $existing.TrimEnd('\') }
    $junctionOk = ($existing -eq $root.TrimEnd('\'))
    if (-not $junctionOk) { (Get-Item $junctionDir).Delete() }
}
if (-not $junctionOk) {
    New-Item -ItemType Junction -Path $junctionDir -Target $root | Out-Null
    Write-Host "created junction: $junctionDir -> $root"
}

$src   = $junctionDir
$build = Join-Path $junctionDir "build"

if ($Clean -and (Test-Path $build)) {
    Write-Host "clean: removing $build"
    Remove-Item $build -Recurse -Force
}

# Drop a cache generated with another generator or another configure path.
$cache = Join-Path $build "CMakeCache.txt"
if (Test-Path $cache) {
    $cacheDir = (Select-String -Path $cache -Pattern "^CMAKE_CACHEFILE_DIR:INTERNAL=(.+)$" -Encoding utf8 |
        Select-Object -First 1).Matches.Groups[1].Value
    $generator = (Select-String -Path $cache -Pattern "^CMAKE_GENERATOR:INTERNAL=(.+)$" -Encoding utf8 |
        Select-Object -First 1).Matches.Groups[1].Value
    $expectedDir = $build -replace '\\', '/'
    if (($cacheDir -and $cacheDir.Trim() -ne $expectedDir) -or ($generator -and $generator.Trim() -ne "Ninja")) {
        Write-Host "removing stale build cache (dir: $cacheDir, generator: $generator)"
        Remove-Item $build -Recurse -Force
    }
}

& $cmake -S $src -B $build -G Ninja "-DCMAKE_MAKE_PROGRAM=$($ninja -replace '\\', '/')"
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

& $cmake --build $build -j 4
if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

# EmberProbe ignores flash targets inside build/, so publish a copy under artifacts/
# (same convention as 04_底盘电控). .gitignore re-includes artifacts/*.elf on purpose.
$artifacts = Join-Path $root "artifacts"
New-Item -ItemType Directory -Force $artifacts | Out-Null
foreach ($ext in @("elf", "hex", "bin")) {
    $from = Join-Path $build "basic_framework.$ext"
    if (Test-Path $from) { Copy-Item $from (Join-Path $artifacts "Gimbal_USB_CAN1.$ext") -Force }
}

arm-none-eabi-size (Join-Path $build "basic_framework.elf")
Write-Host "Built gimbal firmware: $(Join-Path $root 'build\basic_framework.elf') (not flashed)"
Write-Host "Published for EmberProbe: $(Join-Path $artifacts 'Gimbal_USB_CAN1.elf')"
