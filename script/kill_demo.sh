#!/bin/zsh

set -e

SCRIPT_DIR="$(cd "$(dirname "${(%):-%N}")" && pwd)"
STATE_FILE="${FC_VISION_STATE_DIR:-/tmp/fc_vision_demo}/active.env"
SCENE=""

if [ -f "$STATE_FILE" ]; then
    SCENE="$(sed -n 's/^scene=//p' "$STATE_FILE" | head -n 1)"
fi

case "$SCENE" in
    east_church)
        exec "$SCRIPT_DIR/run_east_church.sh" --stop
        ;;
    kino_wall)
        exec "$SCRIPT_DIR/run_kino_wall.sh" --stop
        ;;
    tunnel)
        exec "$SCRIPT_DIR/run_tunnel.sh" --stop
        ;;
    *)
        "$SCRIPT_DIR/run_east_church.sh" --stop
        ;;
esac
