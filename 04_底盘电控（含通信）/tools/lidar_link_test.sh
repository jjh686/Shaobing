#!/usr/bin/env bash
# Independent of the historical navigation workspace and its /tmp aliases.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
lidar_test_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
lidar_test_runtime="${RM_LIDAR_TEST_RUNTIME:-${XDG_CACHE_HOME:-$HOME/.cache}/rm_lidar_link_test}"
lidar_test_mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$lidar_test_mode" == help || "$lidar_test_mode" == --help ]]; then
  echo '用法：bash tools/lidar_link_test.sh build | driver [--host-ip IP --lidar-ip IP] | observe | move --port PORT [--duration 1 --speed 0.05] | test'
  exit 0
fi
source /opt/ros/humble/setup.bash
case "$lidar_test_mode" in
  build)
    mkdir -p -- "$lidar_test_runtime"
    cd -- "$lidar_test_runtime"
    export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
    exec colcon --log-base "$lidar_test_runtime/log/build" build \
      --executor sequential \
      --base-paths "$lidar_test_root/../02_导航（含通信）/外部依赖源码/livox_ros_driver2" \
      --build-base "$lidar_test_runtime/build" --install-base "$lidar_test_runtime/install" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ;;
  driver)
    if [[ ! -f "$lidar_test_runtime/install/local_setup.bash" ]]; then
      echo '请先执行：bash tools/lidar_link_test.sh build' >&2
      exit 2
    fi
    source "$lidar_test_runtime/install/local_setup.bash"
    case "$(uname -m)" in
      x86_64) lidar_sdk_arch=amd64 ;;
      aarch64) lidar_sdk_arch=arm64 ;;
      *) echo '仓库的 Livox SDK 仅支持 x86_64/aarch64' >&2; exit 2 ;;
    esac
    # The upstream driver links the bundled SDK but does not install it.
    lidar_sdk_dir="$lidar_test_root/../02_导航（含通信）/外部依赖源码/livox_ros_driver2/Livox-SDK2/lib/$lidar_sdk_arch"
    export LD_LIBRARY_PATH="$lidar_sdk_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  observe|move) ;;
  test)
    exec python3 -m unittest discover -s "$lidar_test_root/tests" -p test_lidar_link.py -v ;;
  *) echo "未知模式：$lidar_test_mode" >&2; exit 2 ;;
esac
exec python3 "$lidar_test_root/communication_reference/pc/lidar_link_test.py" \
  "$lidar_test_mode" --runtime "$lidar_test_runtime" "$@"
