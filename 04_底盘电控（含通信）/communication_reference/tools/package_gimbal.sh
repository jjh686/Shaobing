#!/usr/bin/env bash
set -euo pipefail
gateway_package_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
gateway_release="$gateway_package_root/gimbal_firmware/firmware_release"
mkdir -p "$gateway_release"
for gateway_format in elf hex; do
  cp "$gateway_package_root/artifacts/gimbal_nav_converter.$gateway_format" "$gateway_release/gimbal_nav_converter.$gateway_format"
done
(cd "$gateway_release" && sha256sum gimbal_nav_converter.elf gimbal_nav_converter.hex > SHA256SUMS)
