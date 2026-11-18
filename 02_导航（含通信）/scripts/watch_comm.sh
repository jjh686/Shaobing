#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
exec ros2 topic echo "${1:-/chassis/diagnostics}"
