#!/usr/bin/env bash
# Prepare the current shell session for running Docker GUI apps with X11
# forwarding. Run once per SSH login (or whenever $DISPLAY changes).
#
# Two host scenarios are handled automatically:
#
#   1. SSH session with X11 forwarding  (DISPLAY=localhost:NN.0)
#      Builds a FamilyWild xauth cookie at /tmp/.docker.xauth so the
#      container can authenticate against the SSH-forwarded X server.
#      Exports XAUTH so docker-compose.jetson.yml's `tune` service
#      bind-mounts the right file.
#
#   2. Local desktop  (DISPLAY=:0)
#      Runs `xhost +local:docker` so the container's UID can connect
#      to the host's X server. No xauth file needed.
#
# Usage (this script MUST be sourced so the exports persist in your
# shell):
#     . scripts/prepare_gui_session.sh
#     # or
#     source scripts/prepare_gui_session.sh
#
# After it prints "ready", launch any GUI service, e.g.:
#     docker compose -f docker-compose.jetson.yml --profile tune run --rm tune
#
# Tips for Qt-OpenGL apps (qv4l2 etc.) over SSH X11: the SSH-forwarded
# display has no working GLX, which kills Qt's hardware OpenGL widgets.
# Force software rendering by also exporting:
#     export DOCKER_RUN_EXTRA_ENV='-e LIBGL_ALWAYS_SOFTWARE=1 -e QT_XCB_GL_INTEGRATION=none'
# (the `tune` service can consume these via `docker compose run -e ...`).

# Reject `./script.sh` invocation — exports wouldn't survive.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "[prepare_gui_session] ERROR: this script must be sourced." >&2
    echo "  Run:  source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

prep_gui_log() { echo "[prepare_gui_session] $*"; }
prep_gui_err() { echo "[prepare_gui_session] ERROR: $*" >&2; }

# ---------------------------------------------------------------------
# 1. DISPLAY must be set
# ---------------------------------------------------------------------
if [ -z "${DISPLAY:-}" ]; then
    prep_gui_err "\$DISPLAY is unset."
    cat >&2 <<'EOF'
  This shell has no X11 connection. Either:
    - You're not in an X session (no local desktop), and SSH did not
      forward X11 to you. Re-SSH with -Y:
          ssh -Y user@jetson
          ssh -Y -J user@jumphost user@jetson      # via a jump host
      Or add to ~/.ssh/config on your local PC:
          ForwardX11 yes
          ForwardX11Trusted yes
    - You're on the local Jetson desktop but launching from a non-GUI
      tty. Open a terminal inside the desktop session and try again.
EOF
    return 1
fi

# ---------------------------------------------------------------------
# 2. Detect SSH vs local desktop
# ---------------------------------------------------------------------
case "$DISPLAY" in
    localhost:*|127.0.0.1:*)
        prep_gui_mode="ssh"
        ;;
    :*)
        prep_gui_mode="local"
        ;;
    *)
        # Remote DISPLAY (foo:0.0) — TCP X11 from another host. Rare;
        # the xauth path below also covers this, treat as SSH-like.
        prep_gui_mode="ssh"
        ;;
esac
prep_gui_log "DISPLAY=$DISPLAY  (mode: $prep_gui_mode)"

# ---------------------------------------------------------------------
# 3. Tools must be available
# ---------------------------------------------------------------------
if ! command -v xauth >/dev/null 2>&1; then
    prep_gui_err "xauth not installed on this host. Fix: sudo apt install xauth"
    return 1
fi
if [ "$prep_gui_mode" = "local" ] && ! command -v xhost >/dev/null 2>&1; then
    prep_gui_err "xhost not installed on this host. Fix: sudo apt install x11-xserver-utils"
    return 1
fi

# ---------------------------------------------------------------------
# 4. Set up auth depending on mode
# ---------------------------------------------------------------------
if [ "$prep_gui_mode" = "ssh" ]; then
    export XAUTH=/tmp/.docker.xauth
    rm -f "$XAUTH"
    touch "$XAUTH"

    # Extract this DISPLAY's cookie, rewrite the family prefix to
    # FamilyWild (ffff) so the container's X11 client connection
    # matches any family the cookie was originally bound to.
    cookie=$(xauth nlist "$DISPLAY" 2>/dev/null)
    if [ -z "$cookie" ]; then
        prep_gui_err "no xauth entry for DISPLAY=$DISPLAY"
        cat >&2 <<'EOF'
  This usually means SSH X11 forwarding silently didn't happen.
  Sanity-check from this same shell:
      xeyes              # should pop a window on your local PC
      echo $DISPLAY      # should print localhost:NN.0
      xauth list         # should have an entry matching $DISPLAY
  If `xeyes` errors, fix SSH X11 first (see DISPLAY section above).
EOF
        return 1
    fi
    if ! printf '%s\n' "$cookie" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null; then
        prep_gui_err "xauth nmerge into $XAUTH failed."
        return 1
    fi
    chmod 644 "$XAUTH"

    if [ ! -s "$XAUTH" ]; then
        prep_gui_err "$XAUTH is empty after merge; cookie injection failed."
        return 1
    fi

    prep_gui_log "XAUTH=$XAUTH  ($(stat -c%s "$XAUTH") bytes, $(xauth -f "$XAUTH" list 2>/dev/null | wc -l) entries)"

elif [ "$prep_gui_mode" = "local" ]; then
    # Authorise the container's runtime to talk to the local X server.
    # Scoped to local Unix sockets only (`+local:`) -- does not open
    # the X server to the network.
    if ! xhost +local:docker >/dev/null 2>&1; then
        prep_gui_err "xhost +local:docker failed. Are you inside the desktop session?"
        return 1
    fi
    # Compose still bind-mounts $XAUTH; point it at a writable empty
    # file so the mount doesn't fail. The container won't actually
    # need it -- xhost covers auth for local Unix sockets.
    export XAUTH=/tmp/.docker.xauth
    : > "$XAUTH" 2>/dev/null || true
    chmod 644 "$XAUTH" 2>/dev/null || true
    prep_gui_log "xhost +local:docker  (local desktop auth)"
fi

prep_gui_log "ready"
cat <<EOF

Next:
  docker compose -f docker-compose.jetson.yml --profile tune run --rm tune

The tune service is already configured with software GL env vars
(LIBGL_ALWAYS_SOFTWARE=1, QT_OPENGL=software, __GLX_VENDOR_LIBRARY_NAME=mesa)
so qv4l2's preview should paint over SSH X11 via Mesa's llvmpipe.

If qv4l2 still fails to open a window, skip the GUI and use the
headless v4l2-ctl route -- see README, "Camera settings" section.

EOF

unset prep_gui_mode
unset -f prep_gui_log prep_gui_err
