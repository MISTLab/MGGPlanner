#!/usr/bin/env bash
#
# One foot-bot exploring under the MGG planner, in ARGoS.
#
#   tools/run_argos_demo.sh              headless, 4 minutes
#   tools/run_argos_demo.sh --gui        with the Filament visualisation
#   tools/run_argos_demo.sh --length 600 longer run
#
# ARGoS runs on the host, because it needs the GPU and an installed argos3
# with the photorealism plugin. The planner runs in the mgg:jazzy container,
# because that is where ROS 2 is. They meet on a Unix socket in a directory
# bind-mounted into the container, which is why the container runs as the
# calling user: a socket created by root cannot be connected to by anyone
# else, and the failure reads as a bare "Permission denied".
#
# The sequence is not just "start both". The planner cannot bootstrap from a
# standing start: no sensor sees the ground beneath the robot, so every edge
# out of the graph's root is rejected as hanging and the first plan comes back
# empty. The robot has to be driven a short distance first, after which the
# map closes up and it plans unaided. That initial nudge is the bootstrap path
# below, and it doubles as a test of the command direction: if the robot moves,
# the whole return path from ROS through the protocol to the wheels works.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${MGG_IMAGE:-mgg:jazzy}"
IPC="${MGG_IPC_DIR:-/tmp/mgg_argos_ipc}"
PLUGINS="$REPO/ros2/src/mgg_argos/argos/build"
LENGTH=240
GUI=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) GUI=1; shift ;;
    --length) LENGTH="$2"; shift 2 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done

command -v argos3 >/dev/null || { echo "argos3 is not on PATH" >&2; exit 1; }
docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo "no $IMAGE image; build it with:" >&2
  echo "  docker build -f docker/jazzy-dev.Dockerfile -t mgg:jazzy ." >&2
  exit 1; }

cleanup() {
  [[ -n "${ARGOS_PID:-}" ]] && kill "$ARGOS_PID" 2>/dev/null || true
  [[ -n "${CID:-}" ]] && docker rm -f "$CID" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

# Builds are quiet unless they fail; ARGoS's own cmake modules emit a page of
# deprecation and OpenGL warnings that are nothing to do with this code.
BUILD_LOG="$(mktemp)"
quietly() {
  if ! "$@" >"$BUILD_LOG" 2>&1; then
    cat "$BUILD_LOG" >&2
    return 1
  fi
}

echo "==> building the ARGoS plugins"
quietly cmake -B "$PLUGINS" -S "$REPO/ros2/src/mgg_argos/argos"
quietly cmake --build "$PLUGINS" -j"$(nproc)"

echo "==> building the ROS 2 workspace"
quietly docker run --rm -v "$REPO/ros2:/ws" "$IMAGE" \
  bash -c 'source /opt/ros/jazzy/setup.bash && colcon build'

mkdir -p "$IPC"
# Only the two logs this script owns. A glob here would delete whatever the
# caller redirected its own output into, if they pointed it at this directory.
rm -f "$IPC"/argos.sock "$IPC"/ros.log "$IPC"/argos.log
# The socket path has to agree on both sides, and neither the config nor the
# experiment file can know where this script decided to put it.
sed "s|socket_path: /tmp/mgg_argos.sock|socket_path: $IPC/argos.sock|" \
  "$REPO/ros2/src/mgg_argos/config/argos_footbot.yaml" > "$IPC/run.yaml"
sed -e "s|/tmp/mgg_argos.sock|$IPC/argos.sock|" \
    -e "s|length=\"0\"|length=\"$LENGTH\"|" \
  "$REPO/ros2/src/mgg_argos/experiments/mgg_footbot.argos" > "$IPC/run.argos"
if [[ $GUI -eq 1 ]]; then
  # Swap the trailing comment for a real visualisation block.
  python3 - "$IPC/run.argos" <<'PY'
import sys
path = sys.argv[1]
text = open(path).read()
marker = "  <!-- No visualization by default"
block = ("  <visualization>\n    <filament>\n"
         "      <camera position=\"-8,-8,6\" look_at=\"0,0,0\" />\n"
         "    </filament>\n  </visualization>\n\n" + marker)
open(path, 'w').write(text.replace(marker, block, 1))
PY
fi

echo "==> starting the planner (bridge + mggplanner + pci)"
CID=$(docker run --rm -d --user "$(id -u):$(id -g)" -e HOME="$IPC" \
  -v "$REPO/ros2:/ws" -v "$REPO/tools:/tools:ro" -v "$IPC:$IPC" "$IMAGE" bash -c "
    source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
    ros2 launch mgg_argos argos_single.launch.py params:=$IPC/run.yaml \
      2>&1 | tee $IPC/ros.log")

for _ in $(seq 60); do [[ -S "$IPC/argos.sock" ]] && break; sleep 1; done
[[ -S "$IPC/argos.sock" ]] || { echo "the bridge never created its socket:" >&2
                                tail -20 "$IPC/ros.log" >&2; exit 1; }

echo "==> starting ARGoS"
# Only the demo's own plugins need adding; argos3 finds its installed ones.
export ARGOS_PLUGIN_PATH="$PLUGINS"
if [[ $GUI -eq 1 ]]; then
  ( cd "$IPC" && argos3 -c run.argos > "$IPC/argos.log" 2>&1 ) & ARGOS_PID=$!
else
  ( cd "$IPC" && env -u DISPLAY -u WAYLAND_DISPLAY argos3 -c run.argos \
      > "$IPC/argos.log" 2>&1 ) & ARGOS_PID=$!
fi
sleep 20
kill -0 "$ARGOS_PID" 2>/dev/null || { echo "ARGoS exited early:" >&2
                                      tail -20 "$IPC/argos.log" >&2; exit 1; }

echo "==> nudging the robot so it can see the ground it is standing on"
docker exec "$CID" bash -c \
  "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   python3 /tools/argos_bootstrap.py" || true

echo "==> handing over to the planner"
docker exec "$CID" bash -c \
  "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
   ros2 service call /pci_trigger std_srvs/srv/Trigger" | tail -2

echo
echo "==> exploring; ^C to stop early. Planning cycles as they land:"
tail -f "$IPC/ros.log" | grep --line-buffered -E "grid graph|forwarded a" &
TAIL_PID=$!
while kill -0 "$ARGOS_PID" 2>/dev/null; do sleep 2; done
kill "$TAIL_PID" 2>/dev/null || true

echo
echo "==> done. Logs: $IPC/ros.log (planner), $IPC/argos.log (simulator)"
