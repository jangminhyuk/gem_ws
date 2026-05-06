#!/bin/bash
# run_stanley.sh — bring up the full Stanley path-tracking stack on the
# real GEM e4 in one shot.
#
# USAGE
#   bash run_stanley.sh                      # default = with obstacle (replanner ON)
#   bash run_stanley.sh no_obstacle          # pure trajectory tracking (replanner OFF)
#   bash run_stanley.sh with_obstacle        # explicit; same as default
#
# These two modes correspond to "Experiment 1" and "Experiment 2" in
# COMMANDS.md — see §3 there.
#
# Opens 5 gnome-terminals in dependency order, each sourcing the
# workspace and launching its piece.  Each terminal stays open after
# the launch exits so you can read errors.
#
# Prereqs the script does NOT do for you:
#   1. CAN bus must already be up.   (sudo bash ~/Desktop/can_start.bash)
#   2. Joystick must be plugged in.  (see §2.a of COMMANDS.md)
#   3. Workspace must already be built:
#         cd ~/CS588/group9/gem_ws
#         colcon build --symlink-install --packages-select gem_gnss_control
#
# Override the workspace location:  WORKSPACE=/some/path bash run_stanley.sh
# Override the per-step delays:     SLEEP_SENSORS=8 SLEEP_GNSS=8 ... bash run_stanley.sh

set -e

# ─── parse mode argument ──────────────────────────────────────────────
MODE="${1:-with_obstacle}"
case "$MODE" in
    with_obstacle|with-obstacle|with)
        STANLEY_LAUNCH_ARGS=""
        MODE_LABEL="WITH obstacle (replanner ON)"
        ;;
    no_obstacle|no-obstacle|none|off|nobst)
        STANLEY_LAUNCH_ARGS="obstacles_yaml:=''"
        MODE_LABEL="NO obstacle (replanner OFF — pure trajectory tracking)"
        ;;
    *)
        echo "ERROR: unknown mode '$MODE'" >&2
        echo "Usage: $0 [no_obstacle|with_obstacle]" >&2
        exit 2
        ;;
esac

WORKSPACE="${WORKSPACE:-$HOME/CS588/group9/gem_ws}"
SETUP="$WORKSPACE/install/setup.bash"

SLEEP_SENSORS="${SLEEP_SENSORS:-5}"
SLEEP_GNSS="${SLEEP_GNSS:-5}"
SLEEP_JOY="${SLEEP_JOY:-2}"
SLEEP_PACMOD="${SLEEP_PACMOD:-3}"

if [ ! -f "$SETUP" ]; then
    echo "ERROR: $SETUP not found." >&2
    echo "Did you run 'colcon build --symlink-install' in $WORKSPACE?" >&2
    exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "ERROR: gnome-terminal not installed." >&2
    echo "If you're on a non-GNOME setup, follow §3.b of COMMANDS.md (manual)." >&2
    exit 1
fi

launch_term() {
    local title="$1"
    local cmd="$2"
    gnome-terminal --title="$title" -- bash -c \
        "source '$SETUP'; echo '>>> $title'; echo '>>> $cmd'; echo; $cmd; echo; echo '--- launch exited; press Enter to close ---'; read"
}

echo "Bringing up Stanley stack from $WORKSPACE"
echo "  mode: $MODE_LABEL"
echo

launch_term "1: sensors"     "ros2 launch basic_launch sensor_init.launch.py"
echo "  + sensors launching, waiting ${SLEEP_SENSORS}s..."
sleep "$SLEEP_SENSORS"

launch_term "2: GNSS+RViz"   "ros2 launch basic_launch visualization.launch.py"
echo "  + GNSS+RViz launching, waiting ${SLEEP_GNSS}s..."
sleep "$SLEEP_GNSS"

launch_term "3: joystick"    "ros2 launch basic_launch dbw_joystick.launch.py"
echo "  + joystick launching, waiting ${SLEEP_JOY}s..."
sleep "$SLEEP_JOY"

launch_term "4: pacmod"      "ros2 launch pacmod2 pacmod2.launch.xml"
echo "  + pacmod launching, waiting ${SLEEP_PACMOD}s..."
sleep "$SLEEP_PACMOD"

launch_term "5: Stanley"     "ros2 launch gem_gnss_control stanley.launch.py $STANLEY_LAUNCH_ARGS"
echo "  + Stanley launching ($MODE_LABEL)."
echo
echo "All 5 terminals up.  Verify each window shows no errors, then arm"
echo "the joystick (LB+RB) to start tracking lane2_refined.csv."
