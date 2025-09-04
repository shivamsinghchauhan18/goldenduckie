#!/bin/bash
set -euo pipefail

# Launch the full pp-navigation stack.
# Env knobs:
#   VEHICLE_NAME            vehicle namespace (default: default)
#   VERBOSE                 true/false for node verbosity (default: true)
#   OBSTACLE_TRACKING       true/false to enable tracker (default: true)

VEHICLE_NAME=${VEHICLE_NAME:-default}
VERBOSE=${VERBOSE:-true}
OBSTACLE_TRACKING=${OBSTACLE_TRACKING:-true}

# Select obstacle source topic for pure_pursuit
OBS_SOURCE="ground_projection/obslist_out"
if [[ "${OBSTACLE_TRACKING}" == "true" ]]; then
	OBS_SOURCE="obstacle_tracking/obslist_tracked"
fi

echo "[launch.sh] VEHICLE_NAME=${VEHICLE_NAME} VERBOSE=${VERBOSE} OBSTACLE_TRACKING=${OBSTACLE_TRACKING} OBS_SOURCE=${OBS_SOURCE}"

exec roslaunch pure_pursuit_lfv lfv_start.launch \
	veh:=${VEHICLE_NAME} \
	verbose:=${VERBOSE} \
	/lane_following/line_detection:=true \
	/lane_following/my_ground_projection:=true \
	/lane_following/my_lane_filter:=true \
	/lane_following/pure_pursuit_lfv:=true \
	/lane_following/duckiebot_detection:=true \
	/lane_following/obstacle_tracking:=${OBSTACLE_TRACKING} \
	obs_source:=${OBS_SOURCE} \
	visualization:=true
