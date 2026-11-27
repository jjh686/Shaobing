#!/usr/bin/env bash
# 决策总入口：一条命令拉起决策、本队实时导航（goal_owner=decision）和当前自瞄入口。
# 默认不连接底盘、不打开 execute_navigation。安装参数未确认时，实时导航会拒绝启动。
# ROS Humble setup 不支持 set -u，因此只在加载环境前使用严格模式。
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "$SCRIPT_DIR/../launch" && -d "$SCRIPT_DIR/../../01_自瞄（含通信）" ]]; then
    DECISION_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
    REPO_ROOT="$(cd -- "$DECISION_ROOT/.." && pwd)"
elif [[ -n "${RM_11_13_ROOT:-}" && -d "${RM_11_13_ROOT}/01_自瞄（含通信）" ]]; then
    REPO_ROOT="$(cd -- "$RM_11_13_ROOT" && pwd)"
    DECISION_ROOT="$REPO_ROOT/06_决策"
elif [[ -n "${RM_DECISION_ROOT:-}" && -d "${RM_DECISION_ROOT}/../01_自瞄（含通信）" ]]; then
    DECISION_ROOT="$(cd -- "$RM_DECISION_ROOT" && pwd)"
    REPO_ROOT="$(cd -- "$DECISION_ROOT/.." && pwd)"
else
    echo '请在 06_决策/scripts 下运行本脚本，或设置 RM_11_13_ROOT 指向仓库根目录。' >&2
    exit 2
fi
export RM_11_13_ROOT="$REPO_ROOT"
NAV_ENV="$REPO_ROOT/02_导航（含通信）/scripts/local_env.sh"
DECISION_ENV="$DECISION_ROOT/scripts/local_env.sh"
# set -u is incompatible with /opt/ros/humble/setup.bash
set +e
set +u
# shellcheck disable=SC1090
if [[ -f "$NAV_ENV" ]]; then
    source "$NAV_ENV"
fi
if [[ ! -f "$DECISION_ENV" ]]; then
    echo "缺少决策环境脚本：$DECISION_ENV" >&2
    exit 2
fi
# shellcheck disable=SC1090
source "$DECISION_ENV" || exit 2
set -eo pipefail

user_keys=()
for arg in "$@"; do
    case "$arg" in
        *:=*) user_keys+=("${arg%%:=*}") ;;
    esac
done
has_arg() {
    local key="$1" item
    for item in "${user_keys[@]}"; do
        if [[ "$item" == "$key" ]]; then
            return 0
        fi
    done
    return 1
}
defaults=()
add_default() {
    if ! has_arg "$1"; then
        defaults+=("$1:=$2")
    fi
}
add_default mode live
add_default start_vision true
add_default vision_usb true
add_default vision_status true
add_default connect_chassis false
add_default execute_navigation false
add_default vision_project "$REPO_ROOT/01_自瞄（含通信）"
if [[ -n "${RM_NAV_RUNTIME:-}" ]]; then
    add_default source_root "$RM_NAV_RUNTIME/source"
    add_default runtime "$RM_NAV_RUNTIME"
    add_default mounting_config "$RM_NAV_RUNTIME/source/local/live_mounting.yaml"
fi
if [[ -n "${RM_NAV_ROOT:-}" ]]; then
    add_default output_dir "$RM_NAV_ROOT/local/maps/$(date +%Y%m%d_%H%M%S)"
fi

echo "仓库：$REPO_ROOT"
echo "总启动：决策 + 自瞄入口 +（mode:=live 时）rm_live_nav，goal_owner=decision"
echo "默认 connect_chassis:=false，execute_navigation:=false。未确认安装时实时导航会拒绝启动。"
exec ros2 launch rm2027_decision sentry_bringup.launch.py "${defaults[@]}" "$@"
