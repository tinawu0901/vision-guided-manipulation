#!/usr/bin/env bash
set -e
project_dir=$(cd "$(dirname "$0")/.." && pwd)
python3 "$project_dir/scripts/demo_session.py" start "$@"
python3 "$project_dir/scripts/demo_session.py" vision
printf 'Demo started. Logs and images: ~/vision_guided_ws/work/gazebo_results\n'
