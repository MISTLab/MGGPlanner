#!/usr/bin/env bash
#
# One foot-bot exploring under the MGG planner, in ARGoS.
#
#   tools/run_argos_demo.sh              headless, one robot, small arena
#   tools/run_argos_demo.sh --maze       four robots in the maze
#   tools/run_argos_demo.sh --bistro     four robots in the Bistro street
#   tools/run_argos_demo.sh --gui        with the Filament visualisation
#   tools/run_argos_demo.sh --length 600 explore for 600 s of wall clock
#
# --gui is also what makes the planner's work visible: the bridge draws each
# robot's path and graph into the photorealism overlay, which the viewer shows
# and no sensor can see.
#
# ARGoS runs on the host, because it needs the GPU and an installed argos3
# with the photorealism plugin. The planner runs in the mgg:jazzy container,
# because that is where ROS 2 is. They meet on a Unix socket in a directory
# bind-mounted into the container, which is why the container runs as the
# calling user: a socket created by root cannot be connected to by anyone
# else, and the failure reads as a bare "Permission denied".
#
# --length is wall-clock seconds of EXPLORING, counted after the handover, not
# simulated seconds passed to ARGoS. The simulation is started unbounded and
# stopped by this script. Bounding it in simulated time is a trap: Bistro at a
# 2 Hz scan rate runs several times faster than real time, so a 300-simulated-
# second run finished 64 seconds in - before the robots had even been nudged -
# and the planning that followed ran against a map nothing was updating.
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
BISTRO=0
MAZE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui) GUI=1; shift ;;
    --bistro) BISTRO=1; shift ;;
    --maze) MAZE=1; shift ;;
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
  pkill -P $$ 2>/dev/null || true
  [[ -n "${ARGOS_PID:-}" ]] && kill "$ARGOS_PID" 2>/dev/null || true
  [[ -n "${CID:-}" ]] && docker rm -f "$CID" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM


# Refuse to start alongside a previous run. Two of these share one ROS graph and
# one socket path, so the second bridge publishes a second /clock and every node
# sees time jump backwards: "Detected jump back in time, clearing TF buffer" on
# repeat, no transforms, and no planning at all. The symptom points nowhere near
# the cause, so make it impossible rather than diagnosable.
STALE_ARGOS="$(pgrep -f 'argos3 -c run.argos' || true)"
STALE_CONTAINERS="$(docker ps -q --filter "ancestor=$IMAGE" || true)"
if [[ -n "$STALE_ARGOS" || -n "$STALE_CONTAINERS" ]]; then
  {
    echo "a previous run is still up:"
    [[ -n "$STALE_ARGOS" ]] && echo "  argos3 pids: $STALE_ARGOS"
    [[ -n "$STALE_CONTAINERS" ]] && echo "  containers: $STALE_CONTAINERS"
    echo "stop it first:"
    echo "  pkill -f 'argos3 -c run.argos'"
    echo "  docker ps -q --filter ancestor=$IMAGE | xargs -r docker rm -f"
  } >&2
  exit 1
fi

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
# Only the logs this script owns. A glob here would delete whatever the caller
# redirected its own output into, if they pointed it at this directory.
rm -f "$IPC"/argos.sock "$IPC"/ros.log "$IPC"/argos.log

# The socket path has to agree on both sides, and neither the config nor the
# experiment file can know where this script decided to put it.
if [[ $BISTRO -eq 1 ]]; then
  CONFIG="$REPO/ros2/src/mgg_argos/config/bistro.yaml"
  LAUNCH="swarm.launch.py"
  ROBOTS=(r0 r1 r2 r3)
  # Generated rather than shipped: the experiment inlines 4346 collision boxes
  # and 28 lamps from the argos3-examples scene, and its asset paths are
  # absolute, so a checked-in copy would be a 630 kB file that only works on
  # the machine it was made on.
  echo "==> generating the Bistro experiment"
  quietly python3 "$REPO/tools/make_bistro_mgg.py" \
    --socket "$IPC/argos.sock" --length 0 \
    $([[ $GUI -eq 1 ]] && echo --gui) -o "$IPC/run.argos"
elif [[ $MAZE -eq 1 ]]; then
  # Same arena as the single-robot run, one robot per corner. The interior
  # partitions mean no robot can see another's territory from its start, so
  # they have to explore towards each other before any graph exchange happens.
  CONFIG="$REPO/ros2/src/mgg_argos/config/maze.yaml"
  LAUNCH="swarm.launch.py"
  ROBOTS=(r0 r1 r2 r3)
  sed "s|/tmp/mgg_argos.sock|$IPC/argos.sock|" \
    "$REPO/ros2/src/mgg_argos/experiments/maze_mgg.argos" > "$IPC/run.argos"
else
  CONFIG="$REPO/ros2/src/mgg_argos/config/argos_footbot.yaml"
  LAUNCH="argos_single.launch.py"
  ROBOTS=(r0)
  sed "s|/tmp/mgg_argos.sock|$IPC/argos.sock|" \
    "$REPO/ros2/src/mgg_argos/experiments/mgg_footbot.argos" > "$IPC/run.argos"
fi
# The maze and single-robot scenes are hand-written files carrying a
# placeholder comment where a viewer would go; the Bistro generator emits its
# own block, so this applies to the other two only.
if [[ $GUI -eq 1 && $BISTRO -eq 0 ]]; then
  python3 - "$IPC/run.argos" <<'VIZ'
import sys
path = sys.argv[1]
text = open(path).read()
marker = "  <!-- No visualization by default"
if marker not in text:
    raise SystemExit("no visualisation placeholder in " + path)
block = ("  <visualization>\n    <filament medium=\"pr\" resolution=\"1280,720\"\n"
         "              position=\"-9,-9,11\" look_at=\"0,0,0\" />\n"
         "  </visualization>\n\n" + marker)
open(path, "w").write(text.replace(marker, block, 1))
VIZ
fi

sed "s|socket_path: /tmp/mgg_argos.sock|socket_path: $IPC/argos.sock|" \
  "$CONFIG" > "$IPC/run.yaml"

echo "==> starting the planner (bridge + ${#ROBOTS[@]} x mggplanner + pci)"
CID=$(docker run --rm -d --user "$(id -u):$(id -g)" -e HOME="$IPC" \
  -v "$REPO/ros2:/ws" -v "$REPO/tools:/tools:ro" -v "$IPC:$IPC" "$IMAGE" bash -c "
    source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash
    ros2 launch mgg_argos $LAUNCH params:=$IPC/run.yaml \
      2>&1 | tee $IPC/ros.log")

for _ in $(seq 60); do [[ -S "$IPC/argos.sock" ]] && break; sleep 1; done
[[ -S "$IPC/argos.sock" ]] || { echo "the bridge never created its socket:" >&2
                                tail -20 "$IPC/ros.log" >&2; exit 1; }

# Prefer local argos3 build if present, falling back to installed plugins.
ARGOS_BUILD="$REPO/../argos3/build"
if [[ -d "$ARGOS_BUILD" ]]; then
  export ARGOS_PLUGIN_PATH="$PLUGINS:$ARGOS_BUILD/plugins/simulator/photorealism:$ARGOS_BUILD/plugins/simulator/visualizations/filament"
else
  export ARGOS_PLUGIN_PATH="$PLUGINS"
fi

if [[ $GUI -eq 1 ]]; then
  ( cd "$IPC" && argos3 -c run.argos > "$IPC/argos.log" 2>&1 ) & ARGOS_PID=$!
else
  ( cd "$IPC" && env -u DISPLAY -u WAYLAND_DISPLAY argos3 -c run.argos \
      > "$IPC/argos.log" 2>&1 ) & ARGOS_PID=$!
fi
sleep 20
kill -0 "$ARGOS_PID" 2>/dev/null || { echo "ARGoS exited early:" >&2
                                      tail -20 "$IPC/argos.log" >&2; exit 1; }

echo "==> triggering autonomous exploration across all robots"
TRIGGER_PIDS=()
for robot in "${ROBOTS[@]}"; do
  if [[ ${#ROBOTS[@]} -eq 1 ]]; then srv="/pci_trigger"; else srv="/$robot/pci_trigger"; fi
  docker exec "$CID" bash -c \
    "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && \
     ros2 service call $srv std_srvs/srv/Trigger" >/dev/null 2>&1 &
  TRIGGER_PIDS+=($!)
done
wait "${TRIGGER_PIDS[@]}" 2>/dev/null || true



echo
echo "==> exploring for ${LENGTH}s; ^C to stop early. Planning cycles as they land:"
tail -n 0 -f --pid=$$ "$IPC/ros.log" | grep --line-buffered -E "grid graph|forwarded a|merged robot" &
TAIL_PID=$!
DEADLINE=$(( SECONDS + LENGTH ))
while kill -0 "$ARGOS_PID" 2>/dev/null && (( SECONDS < DEADLINE )); do sleep 2; done
kill "$TAIL_PID" 2>/dev/null || true
pkill -P $$ 2>/dev/null || true

# Stop the simulator rather than waiting for a length it was never given.
kill "$ARGOS_PID" 2>/dev/null || true
wait "$ARGOS_PID" 2>/dev/null || true

echo
echo "==> summary"
for robot in "${ROBOTS[@]}"; do
  cycles=$(grep -c "$robot\.mggplanner_node.*grid graph" "$IPC/ros.log" || true)
  last=$(grep "$robot\.mggplanner_node.*grid graph" "$IPC/ros.log" | tail -1 |
         sed 's/.*grid graph: //' | cut -c1-96)
  printf '  %s: %s planning cycles | %s\n' "$robot" "$cycles" "${last:-none}"
done
merges=$(grep -c "merged robot" "$IPC/ros.log" || true)
echo "  graph merges between robots: $merges"
if [[ $merges -gt 0 ]]; then
  # Which pairs actually met, not just how many times. Four robots that all
  # merged with one neighbour and none with the others is a different result
  # from a swarm that has joined up.
  grep -oE "\[r[0-9]\.mggplanner_node\]: merged robot [0-9]+" "$IPC/ros.log" |
    sed -E 's/\[(r[0-9])\.mggplanner_node\]: merged robot ([0-9]+)/  \1 <- robot \2/' |
    sort | uniq -c | sed 's/^/  /'
fi
# The global graph is what the robots exchange, so its size is the thing to
# watch: a graph that never grows means nothing is being shared.
for robot in "${ROBOTS[@]}"; do
  vertices=$(grep -oE "$robot\.mggplanner_node.*global graph seeded" "$IPC/ros.log" | wc -l)
  printf '  %s: global graph seeded %s time(s)\n' "$robot" "$vertices"
done

echo
echo "==> done. Logs: $IPC/ros.log (planner), $IPC/argos.log (simulator)"
