#!/usr/bin/env bash
# Source this file in bash. Use --dependencies-only internally before building.
# ROS Humble setup scripts do not support set -u.
RM_DECISION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
RM_DECISION_WS="${RM_DECISION_WS:-$HOME/.cache/rm2027_decision_ws}"
RM_DECISION_BT4_PREFIX="${RM_DECISION_BT4_PREFIX:-$HOME/.local/opt/rm2027_bt4/opt/ros/humble}"
export RM_DECISION_ROOT RM_DECISION_WS RM_DECISION_BT4_PREFIX

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo '缺少 ROS 2 Humble：/opt/ros/humble/setup.bash' >&2
    return 1
fi
source /opt/ros/humble/setup.bash || return 1

if [[ ! -f /opt/ros/humble/share/behaviortree_cpp/cmake/behaviortree_cppConfig.cmake ]]; then
    if [[ ! -f "$RM_DECISION_BT4_PREFIX/share/behaviortree_cpp/cmake/behaviortree_cppConfig.cmake" ||
          ! -f "$RM_DECISION_BT4_PREFIX/local_setup.bash" ]]; then
        echo '缺少 BehaviorTree.CPP v4；安装 ros-humble-behaviortree-cpp，或配置用户目录依赖前缀。' >&2
        return 1
    fi
    source "$RM_DECISION_BT4_PREFIX/local_setup.bash" || return 1
fi

if [[ "${1:-}" != --dependencies-only && -f "$RM_DECISION_WS/install/local_setup.bash" ]]; then
    source "$RM_DECISION_WS/install/local_setup.bash" || return 1
fi
