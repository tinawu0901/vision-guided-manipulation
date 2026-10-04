#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source "$HOME/vision_guided_ws/install/setup.bash"
source "$HOME/vision_guided_ws/.venv-vision/bin/activate"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
project_dir=$(cd "$(dirname "$0")/.." && pwd)
exec python -m vision_guided_simulation.camera_vision --ros-args \
  --params-file "$project_dir/vision_guided_simulation/config/vision.yaml" \
  -p use_sim_time:=true \
  -p model_path:="$HOME/vision_guided_ws/work/vision_smoke_test/yolov8n.pt" "$@"
