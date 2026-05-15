#!/usr/bin/env bash
# Prepare host-side X11 access for the docker-compose.jetson.yml `tune`
# service. Run once per shell session on the Jetson before
# `docker compose ... run --rm tune`.
#
# Two cases:
#   - Local Jetson desktop terminal: just runs `xhost +local:docker`
#   - SSH session with X11 forwarding (DISPLAY=localhost:NN.0): also
#     builds /tmp/.docker.xauth with the auth cookie rewritten to
#     FamilyWild so connections from the container's namespace match.
#
# Usage:
#   source scripts/jetson_x11_setup.sh
#   docker compose -f docker-compose.jetson.yml --profile tune run --rm tune
#
# (Use `source`, not `bash`, so XAUTH is exported into the parent shell
# and the compose file's ${XAUTH} bind mount sees the same path.)

set -e

if [ -z "$DISPLAY" ]; then
    echo "DISPLAY is unset. If you're SSH'd in, reconnect with 'ssh -Y'." >&2
    return 1 2>/dev/null || exit 1
fi

case "$DISPLAY" in
    localhost:*|127.0.0.1:*|*:[0-9]*.[0-9])
        # SSH X11 forwarding (TCP-style display). Cookie-auth setup.
        export XAUTH=/tmp/.docker.xauth
        rm -f "$XAUTH"
        touch "$XAUTH"
        xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
        chmod 644 "$XAUTH"
        echo "Built $XAUTH for SSH-forwarded DISPLAY=$DISPLAY."
        ;;
    *)
        # Local display. xhost is enough.
        export XAUTH=/tmp/.docker.xauth
        # The compose file bind-mounts $XAUTH unconditionally; create an
        # empty file so docker doesn't create a directory there.
        touch "$XAUTH"
        if command -v xhost >/dev/null 2>&1; then
            xhost +local:docker >/dev/null
            echo "Granted local docker access via xhost (DISPLAY=$DISPLAY)."
        else
            echo "xhost not installed; install x11-xserver-utils on the host." >&2
        fi
        ;;
esac
