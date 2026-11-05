#!/usr/bin/env bash
# Reproduce every result in this study. Needs ROS 2 Jazzy with MoveIt and the
# openamr-upperbody-sw workspace built as its README describes (~/openamr_ws).
# Simulation only: mock hardware, no robot.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-$HERE/../results}
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash
source ~/openamr_ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

# Two stacks on one ROS domain interleave /joint_states and TF and corrupt every live result.
if pgrep -f "moveit_ros_move_group/move_group|ros2_control_node" >/dev/null; then
  echo "ABORT: a move_group or ros2_control_node is already running; stop it first." >&2
  exit 2
fi

ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=mock rviz:=false > "$OUT/moveit_mock.log" 2>&1 &
LAUNCH=$!
trap 'kill $LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null' EXIT
until ros2 service list 2>/dev/null | grep -q '^/compute_ik$' && \
      ros2 control list_controllers 2>/dev/null | grep -q 'lift_controller.*active'; do sleep 2; done

python3 "$HERE/reach_sweep.py" "$OUT/reach.json"
python3 "$HERE/analyze_reach.py" "$OUT/reach.json"
python3 "$HERE/solver_check.py" "$OUT/reach.json" "$OUT/solver.json"
python3 "$HERE/lift_hold.py" "$OUT/lift_table.json" table 0,0,0,1.5708,0,0,0 base world
python3 "$HERE/lift_hold.py" "$OUT/lift_raised.json" raised 0.5236,0,0,1.5708,0,0,0 base world
python3 "$HERE/analyze_lift.py" "$OUT/lift_table.json" "$OUT/lift_raised.json"
python3 "$HERE/clutch_on_lift.py" "$OUT/clutch_lift_vz-0.05.json" -0.05 carriage world_target world_state
python3 "$HERE/clutch_on_lift.py" "$OUT/clutch_lift_vz0.025.json" 0.025 carriage world_target world_state
python3 "$HERE/make_figures.py" "$OUT" "$HERE/../figures"
