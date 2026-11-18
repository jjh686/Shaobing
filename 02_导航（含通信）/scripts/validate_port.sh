#!/usr/bin/env bash
[[ -c "$PORT" && -r "$PORT" && -w "$PORT" ]] || {
    echo "Not an accessible serial device: $PORT" >&2; exit 2;
}
port_properties="$(udevadm info --query=property --name="$PORT")"
if [[ "$port_properties" =~ [Cc][Mm][Ss][Ii][Ss]|[Dd][Aa][Pp][Ll][Ii][Nn][Kk]|MuseLab|[Ss][Tt]-?[Ll][Ii][Nn][Kk] ]]; then
    echo 'Selected USB device is a debugger, not the C-board application USB port.' >&2
    exit 2
fi
