#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
live_mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "$live_mode" in
  build)
    ros2 pkg prefix livox_ros_driver2 >/dev/null || {
      echo '先运行：bash ../04_底盘电控（含通信）/tools/lidar_link_test.sh build' >&2; exit 2;
    }
    export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
    cd "$RM_NAV_RUNTIME"
    exec colcon --log-base "$RM_NAV_RUNTIME/log/build-live" build --symlink-install \
      --executor sequential --base-paths "$RM_NAV_RUNTIME/source/src/rm_live_nav" \
      "$RM_NAV_RUNTIME/source/src/rm_chassis_comm" \
      "$RM_NAV_RUNTIME/source/src/pb2025_sentry_nav/point_lio" \
      "$RM_NAV_RUNTIME/source/src/pb2025_sentry_nav/pointcloud_to_laserscan" \
      --packages-select rm_live_nav rm_chassis_comm point_lio pointcloud_to_laserscan \
      --build-base "$RM_NAV_RUNTIME/build" --install-base "$RM_NAV_RUNTIME/install" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF ;;
  mapping) live_nav=false; live_connect=false; live_driver=true; live_port=/dev/unused ;;
  preview) live_nav=true; live_connect=false; live_driver=false; live_port=/dev/unused ;;
  navigation)
    [[ $# -gt 0 ]] || { echo '用法：navigation /dev/serial/by-id/应用USB [参数]' >&2; exit 2; }
    live_port=$1; shift
    python3 "$RM_NAV_ROOT/../04_底盘电控（含通信）/communication_reference/pc/check_port.py" "$live_port" >/dev/null
    live_nav=true; live_connect=true; live_driver=true ;;
  save) exec timeout 20s ros2 service call /mid360_test/save_map std_srvs/srv/Trigger '{}' ;;
  test)
    export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
    exec python3 "$RM_NAV_ROOT/src/rm_live_nav/test/test_live_nav.py" ;;
  test-mapping)
    export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
    exec python3 "$RM_NAV_ROOT/src/rm_live_nav/test/test_live_pipeline.py" --mapping-only ;;
  test-pipeline|test-explorer)
    export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
    if [[ "$live_mode" == test-pipeline ]]; then live_test=test_live_pipeline.py; else live_test=test_explorer_action.py; fi
    exec python3 "$RM_NAV_ROOT/src/rm_live_nav/test/$live_test" ;;
  env) return 0 2>/dev/null || exit 0 ;;
  *)
    echo '用法：run_live_exploration.sh build | mapping | preview | navigation PORT | save | test | test-mapping | test-pipeline | test-explorer'
    echo '参数示例：host_ip:=192.168.1.50 lidar_ip:=192.168.1.106 rviz:=true'
    echo '启动默认不使能底盘或探索；安装参数未确认时只允许 mapping。'
    exit 0 ;;
esac
exec ros2 launch rm_live_nav live_mapping.launch.py \
  "navigation:=$live_nav" "connect:=$live_connect" "start_lidar:=$live_driver" \
  "serial_port:=$live_port" "source_root:=$RM_NAV_RUNTIME/source" \
  "runtime:=$RM_NAV_RUNTIME" \
  "mounting_config:=$RM_NAV_RUNTIME/source/local/live_mounting.yaml" \
  "output_dir:=$RM_NAV_ROOT/local/maps/$(date '+%Y%m%d_%H%M%S')" "$@"
