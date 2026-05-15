#!/usr/bin/env bash
# Prepare host-side X11 access for the `tune` service when SSH'd into
# the Jetson with X11 forwarding (`ssh -Y jetson`).
#
# Designed for the multi-user case: several users SSH into the same
# Jetson, each runs `docker compose run --rm tune`, and each expects
# qv4l2 to pop up on their own laptop. To avoid clobbering each other's
# X cookies, the xauth file goes under /tmp/.docker.xauth.$USER and the
# script exports XAUTH so the compose file's ${XAUTH:-/dev/null} bind
# picks up the per-user path.
#
# Usage (must use `source`, not `bash`, so XAUTH lands in your shell):
#   source scripts/jetson_x11_setup.sh
#   docker compose run --rm tune
#
# What it does:
#   - For SSH-forwarded DISPLAY (localhost:NN.0): builds a per-user
#     xauth file containing every entry from the user's ~/.Xauthority
#     rewritten to FamilyWild (so the family-mismatch issue between
#     `unix:NN` and TCP `localhost:NN.0` becomes irrelevant). The
#     container reads whichever entry matches its connection.
#   - For local DISPLAY (:0, :1, ...): grants the docker user X access
#     via xhost and touches the xauth file (empty is fine when xhost
#     is granting access).

set -e

if [ -z "$DISPLAY" ]; then
    echo "DISPLAY is unset. If you're SSH'd in, reconnect with 'ssh -Y'." >&2
    return 1 2>/dev/null || exit 1
fi

if ! command -v xauth >/dev/null 2>&1; then
    echo "xauth is missing on this Jetson. Install with: sudo apt-get install -y xauth" >&2
    return 1 2>/dev/null || exit 1
fi

# Per-user file so concurrent SSH sessions don't overwrite each other.
# Fall back to $UID if $USER is unset (e.g. some restricted shells).
USER_TAG="${USER:-$UID}"
export XAUTH="/tmp/.docker.xauth.${USER_TAG}"

case "$DISPLAY" in
    localhost:*|127.0.0.1:*|*:[0-9]*.[0-9])
        # SSH X11-forwarded display. Mirror every cookie in ~/.Xauthority
        # into $XAUTH with the family rewritten to ffff (FamilyWild),
        # so a connection from any source (including the container's
        # localhost binding under network_mode: host) matches.
        rm -f "$XAUTH"
        touch "$XAUTH"
        # `xauth nlist` lists every entry in numeric text format. The
        # first 4 hex chars are the family; ffff = FamilyWild. We
        # rewrite all entries blindly -- xauth dedupes on merge.
        xauth nlist | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null
        chmod 600 "$XAUTH"   # cookies are sensitive; don't world-read
        n_entries=$(xauth -f "$XAUTH" list 2>/dev/null | wc -l)
        if [ "$n_entries" = "0" ]; then
            echo "WARNING: $XAUTH is empty. Your ~/.Xauthority has no entries." >&2
            echo "         Disconnect and reconnect with: ssh -Y $(hostname)" >&2
            return 1 2>/dev/null || exit 1
        fi
        echo "[jetson_x11_setup] DISPLAY=$DISPLAY  XAUTH=$XAUTH  entries=$n_entries"
        ;;
    *)
        # Local console display. xhost is enough; the xauth file just
        # needs to exist so the compose bind-mount doesn't create a
        # directory.
        : > "$XAUTH"
        if command -v xhost >/dev/null 2>&1; then
            xhost +local:docker >/dev/null
            echo "[jetson_x11_setup] DISPLAY=$DISPLAY  granted xhost +local:docker"
        else
            echo "xhost not installed; install with: sudo apt-get install -y x11-xserver-utils" >&2
        fi
        ;;
esac
