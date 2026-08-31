#!/usr/bin/env bash
# Record the field-frame positions the overhead tracker publishes.
#
#   scripts/record_localization.sh              # timestamped bag, Ctrl-C to stop
#   scripts/record_localization.sh /path/to/bag # explicit destination
#
# mcap, not sqlite3: Foxglove opens it natively and it is the rosbag2 default
# going forward. Bags land OUTSIDE the workspace -- `colcon build` wipes
# install/, and a bag under the repo would end up in git status every run.
set -euo pipefail

OUT="${1:-$HOME/overhead_field/bags/localization_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"

# Regex, not a topic list: robots only get a /localization publisher when they
# LINK, so a fixed list would silently miss any robot linked after this starts.
#
# ~/detections is the same estimate with everything the pose drops: pixel u/v,
# match score, heading, per-robot status and miss count, plus frame_age_s. It is
# one String per tick for the whole fleet, so it costs a fraction of the six
# pose topics and is what an offline tool needs to rebuild the annotated frames.
exec ros2 bag record -s mcap -o "$OUT" \
    -e '/localization/.*/position|/overhead_tracker_node/detections'
