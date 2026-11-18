#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
if [[ "${1:-}" == --save ]]; then
    exec timeout 20s ros2 service call /mid360_test/save_map std_srvs/srv/Trigger '{}'
fi
output_dir="$RM_NAV_RUNTIME/source/local/maps/$(date '+%Y%m%d_%H%M%S')"
printf 'Mapping output: %s\n' "$output_dir"
exec ros2 launch pb2025_nav_bringup mid360_mapping_test.launch.py \
    "params_file:=$RM_NAV_RUNTIME/source/local/nav2_params.yaml" \
    "recorder_script:=$RM_NAV_RUNTIME/source/scripts/mid360_map_recorder.py" \
    "output_dir:=$output_dir" "$@"
