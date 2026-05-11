#!/usr/bin/env bash
# Dump every current v4l2 control value as a v4l2_camera ROS parameter
# YAML. The output drops into camera_info/v4l2_params.yaml and is
# auto-loaded the next time the `vlm` service starts.
#
# Usage:
#   save_camera_settings.sh [DEVICE] [OUTPUT]
#
# `v4l2-ctl --list-ctrls` reports each control's type in parentheses
# (`(int)`, `(bool)`, `(menu)`, ...) so we can emit the right YAML
# scalar type. v4l2_camera will reject mismatched types at startup.
set -euo pipefail

DEVICE="${1:-${CAMERA_DEVICE:-/dev/video0}}"
OUTPUT="${2:-/root/.ros/camera_info/v4l2_params.yaml}"

if [[ ! -e "$DEVICE" ]]; then
    echo "$DEVICE not present." >&2
    exit 1
fi
mkdir -p "$(dirname "$OUTPUT")"

# `v4l2-ctl --list-ctrls` lines look like:
#   brightness 0x00980900 (int)    : min=0 max=255 step=1 default=128 value=128
#   white_balance_automatic 0x0098090c (bool)   : default=1 value=0
# We extract <name>, <type>, <value> and format the right YAML.
{
    echo "# v4l2_camera ROS parameters --- saved $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "# Device: $DEVICE"
    echo "v4l2_camera:"
    echo "  ros__parameters:"
    v4l2-ctl -d "$DEVICE" --list-ctrls 2>/dev/null \
        | awk '
            /^[[:space:]]*[a-z_][a-z0-9_]*[[:space:]]+0x[0-9a-f]+/ && /value=/ {
                name = $1
                type = ""
                for (i = 1; i <= NF; i++) {
                    if ($i ~ /^\(/) { type = $i; gsub(/[()]/, "", type); break }
                }
                value = ""
                for (i = 1; i <= NF; i++) {
                    if ($i ~ /^value=/) {
                        split($i, a, "=")
                        value = a[2]
                        break
                    }
                }
                if (value == "") next
                # Skip read-only / inactive flagged controls --
                # setting them throws EPERM.
                if (index($0, "flags=") > 0 && index($0, "inactive") > 0) next
                if (type == "bool") {
                    print "    " name ": " (value+0 ? "true" : "false")
                } else if (type == "int" || type == "menu" || type == "intmenu") {
                    print "    " name ": " value+0
                }
                # button / string / bitmask are skipped --
                # v4l2_camera does not expose them as parameters.
            }
        '
} > "$OUTPUT"

echo "Wrote $(grep -c ':' "$OUTPUT") parameter lines to $OUTPUT"
