#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
cmake -S "$RM_NAV_RUNTIME/source/src/rm_chassis_comm" \
  -B "$RM_NAV_RUNTIME/build/rm_chassis_comm" -DBUILD_TESTING=ON
cmake --build "$RM_NAV_RUNTIME/build/rm_chassis_comm" -j2
ctest --test-dir "$RM_NAV_RUNTIME/build/rm_chassis_comm" --output-on-failure
cmake -S "$RM_NAV_RUNTIME/source/src/pb2025_sentry_nav/fake_vel_transform" \
  -B "$RM_NAV_RUNTIME/build/fake_vel_transform" -DBUILD_TESTING=OFF -DSENTRY_LINK_TESTING=ON
cmake --build "$RM_NAV_RUNTIME/build/fake_vel_transform" -j2
ctest --test-dir "$RM_NAV_RUNTIME/build/fake_vel_transform" --output-on-failure
python3 "$RM_NAV_RUNTIME/source/src/rm_chassis_comm/test/test_ros_pty.py" \
  "$RM_NAV_RUNTIME/install/rm_chassis_comm/lib/rm_chassis_comm/chassis_node"
