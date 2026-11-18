# Windows build entry for the chassis firmware.
# Linux keeps using tools/build_chassis.sh (build/chassis -> artifacts/DiPan_Board.*).
# This script never touches build/chassis (the committed Linux tree); it builds into
# build/chassis-win with Ninja and copies the result to artifacts/DiPan_Board.*
#
# Two Windows-specific workarounds are baked in:
#  1. Ninja instead of "MinGW Makefiles" - make cannot handle this project path.
#  2. The project path contains non-ASCII (Chinese) characters, which MSYS2 ld mangles
#     when the command line is passed through a Ninja response file. The sources are
#     therefore referenced through an ASCII junction under %LOCALAPPDATA%.
$ErrorActionPreference = "Stop"

$root        = Split-Path -Parent $PSScriptRoot
$artifacts   = Join-Path $root "artifacts"
$junctionDir = Join-Path $env:LOCALAPPDATA "rm-sentry-04"

# Toolchain locations; override with env vars when paths differ.
$toolchainDir = if ($env:ARM_TOOLCHAIN_BIN) { $env:ARM_TOOLCHAIN_BIN } else { "D:\MSYS2\ucrt64\bin" }
$makeDir      = if ($env:MAKE_BIN_DIR)      { $env:MAKE_BIN_DIR }      else { "D:\MSYS2\mingw64\bin" }

$env:PATH = "$toolchainDir;$makeDir;" + $env:PATH

# Locate a tool: PATH first, then the usual install roots (winget portable packages
# are not always linked onto PATH, so their package directory is searched too).
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

# --- ASCII junction so that no build path contains Chinese characters -------------
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

$src   = Join-Path $junctionDir "chassis_driver\basic_framework"
$build = Join-Path $junctionDir "build\chassis-win"

# Drop a cache that was generated with another generator or another configure path.
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

New-Item -ItemType Directory -Force $artifacts | Out-Null
foreach ($ext in @("elf", "hex", "bin")) {
    $from = Join-Path $build "basic_framework.$ext"
    if (-not (Test-Path $from)) { throw "missing build output: $from" }
    Copy-Item $from (Join-Path $artifacts "DiPan_Board.$ext") -Force
}

arm-none-eabi-size (Join-Path $artifacts "DiPan_Board.elf")
Write-Host "Built chassis firmware: $(Join-Path $artifacts 'DiPan_Board.elf') (not flashed)"
