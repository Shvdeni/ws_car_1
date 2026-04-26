#!/usr/bin/env bash

cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cleanup() {
  if [[ -n "${launch_pid:-}" ]]; then
    kill "$launch_pid" 2>/dev/null || true
    pkill -P "$launch_pid" 2>/dev/null || true
  fi
  pkill -f "ros2 launch touch_car_gazebo" 2>/dev/null || true
  pkill -f "parameter_bridge" 2>/dev/null || true
  pkill -f "bumper_driver" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
}
trap cleanup EXIT

ros2 launch touch_car_gazebo room.launch.py >/tmp/touch_car_launch.log 2>&1 &
launch_pid=$!
sleep 10

echo "--- nodes ---"
ros2 node list || true

echo "--- topics ---"
ros2 topic list | sort || true

echo "--- cmd_vel info ---"
ros2 topic info /model/touch_car/cmd_vel -v || true

echo "--- cmd_vel sample ---"
timeout 5s ros2 topic echo /model/touch_car/cmd_vel --once || true

echo "--- odometry sample ---"
timeout 5s ros2 topic echo /model/touch_car/odometry --once || true

echo "--- world pose sample ---"
gz topic -e -t /world/room_1_5m/pose/info -n 1 | grep -A12 -B2 "touch_car" | head -80 || true

echo "--- contact sample after short drive ---"
timeout 20s ros2 topic echo /touch_car/bumper_contacts --once || true

echo "--- gz topics ---"
gz topic -l | sort | grep -E "touch_car|cmd_vel|bumper|contact|world" || true

echo "--- launch log tail ---"
tail -120 /tmp/touch_car_launch.log || true
