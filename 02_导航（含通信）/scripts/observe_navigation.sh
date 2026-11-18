#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/local_env.sh"
exec python3 "$RM_NAV_ROOT/scripts/observe_navigation.py" "$@"
