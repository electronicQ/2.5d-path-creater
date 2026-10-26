#!/usr/bin/env bash
set -euo pipefail
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
cd "$root_dir"
source /etc/os-release
case "${ID:-}:${VERSION_ID:-}" in
  ubuntu:18.04) distro=melodic; ros_python=/usr/bin/python2.7; planner_python="$root_dir/.runtime/planner/bin/python" ;;
  ubuntu:20.04) distro=noetic; ros_python=/usr/bin/python3; planner_python=/usr/bin/python3 ;;
  *) echo 'Unsupported native target. See README.' >&2; exit 1 ;;
esac
[[ -z ${ROS_DISTRO:-} || $ROS_DISTRO == "$distro" ]] || { echo 'Wrong ROS distribution loaded; open a clean shell.' >&2; exit 1; }
[[ -f devel/setup.bash && -x $planner_python ]] || { echo 'Run scripts/deploy_vehicle.sh first.' >&2; exit 1; }
# Catkin setup scripts consume the caller's positional arguments. Preserve
# server options, so --help/--config are never parsed by _setup_util.py.
requested_args=("$@")
set --
set +u
source "/opt/ros/$distro/setup.bash"
source devel/setup.bash
set -u
export VEHICLE_ROS_PYTHON="$ros_python"
export VEHICLE_ROS_PYTHONPATH="${PYTHONPATH:-}"
# Melodic's Python 2 YAML and generated messages must not shadow Python 3 modules.
unset PYTHONPATH PYTHONHOME
set -- "${requested_args[@]}"
if (($# == 0)); then
  set -- --config tools/ground_vehicle_tcp/config.yaml
fi
exec "$planner_python" -m tools.ground_vehicle_tcp.main "$@"
