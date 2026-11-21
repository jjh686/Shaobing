#!/usr/bin/env bash
set -eo pipefail
if [[ "${1:-}" == --save ]]; then
    exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_live_exploration.sh" save
fi
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/run_live_exploration.sh" mapping "$@"
