#!/usr/bin/env bash
# Vehicle bootstrap for the supported Ubuntu 20.04 / ROS1 Noetic stack.
set -Eeuo pipefail

usage() {
  cat <<'HELP'
Usage: bash scripts/deploy_vehicle.sh [options]
  --jobs N          Build parallelism (default: 2)
  --deps-only       Install/check dependencies without compiling
  --build-only      Compile using existing dependencies (no apt/rosdep/network)
  --setup-ros-repo  Add the official signed ROS1 apt repository if needed
  --help            Print this help without changing the machine
Run as the ordinary vehicle user, not sudo. apt/rosdep use sudo when required.
Supported native target: Ubuntu 20.04 + ROS Noetic. Logs: log/deploy/.
HELP
}
jobs=2
mode=all
setup_repo=0
while (($#)); do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --jobs) [[ $# -ge 2 && $2 =~ ^[1-9][0-9]*$ ]] || { echo 'Invalid --jobs' >&2; exit 2; }; jobs=$2; shift ;;
    --deps-only) [[ $mode == all ]] || { echo 'Conflicting modes' >&2; exit 2; }; mode=deps ;;
    --build-only) [[ $mode == all ]] || { echo 'Conflicting modes' >&2; exit 2; }; mode=build ;;
    --setup-ros-repo) setup_repo=1 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
  shift
done
[[ $mode != build || $setup_repo == 0 ]] || { echo 'Repository setup is incompatible with --build-only' >&2; exit 2; }
[[ $EUID != 0 ]] || { echo 'Run as the ordinary vehicle user, without sudo.' >&2; exit 1; }
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
cd "$root_dir"
mkdir -p log/deploy
log_file="$root_dir/log/deploy/$(date +%Y%m%d-%H%M%S)-$$.log"
exec > >(tee -a "$log_file") 2>&1
trap 'code=$?; printf "\nDeployment failed (exit %s), line %s. Log: %s\n" "$code" "$LINENO" "$log_file" >&2; exit "$code"' ERR
fail() { echo "ERROR: $*" >&2; exit 1; }
# shellcheck disable=SC1091
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 20.04 ]] || fail "Native deployment supports Ubuntu 20.04 + ROS Noetic; detected ${PRETTY_NAME:-unknown}. Do not install focal ROS binaries on 22.04/24.04. Use a 20.04 system/container or port ROS/TBB first."
[[ -z ${VIRTUAL_ENV:-} && -z ${CONDA_PREFIX:-} ]] || fail 'Deactivate virtualenv/conda before deployment; this service uses system Python.'
[[ -z ${ROS_DISTRO:-} || $ROS_DISTRO == noetic ]] || fail "Another ROS environment is loaded: $ROS_DISTRO. Open a clean shell."
echo "Workspace: $root_dir; mode: $mode; jobs: $jobs; log: $log_file"

if [[ $mode != build ]]; then
  command -v sudo >/dev/null || fail 'sudo is required.'
  sudo -v
  sudo apt-get update
  sudo apt-get install -y build-essential cmake pkg-config curl ca-certificates gnupg software-properties-common
  sudo add-apt-repository -y universe
  if ((setup_repo)); then
    # Opt-in: do not rewrite the machine's existing repository configuration by default.
    key_tmp=$(mktemp)
    curl --fail --show-error --location --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o "$key_tmp"
    gpg --show-keys "$key_tmp"
    sudo install -m 0644 "$key_tmp" /usr/share/keyrings/vehicle-ros-archive-keyring.asc
    rm -f "$key_tmp"
    repo_line="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vehicle-ros-archive-keyring.asc] https://packages.ros.org/ros/ubuntu focal main"
    # Avoid conflicting Signed-By settings in an already configured ROS repository.
    if sudo grep -RqsE '^[[:space:]]*(deb .*packages\.ros\.org/ros/ubuntu|URIs:.*packages\.ros\.org/ros/ubuntu)' /etc/apt/sources.list /etc/apt/sources.list.d; then
      echo 'Existing official ROS apt source detected; leaving it unchanged.'
    else
      printf '%s\n' "$repo_line" | sudo tee /etc/apt/sources.list.d/vehicle-ros1.list >/dev/null
    fi
  fi
  sudo apt-get update
  apt-cache show ros-noetic-ros-base >/dev/null 2>&1 || fail 'ROS Noetic apt packages unavailable. Configure a working ROS repository or rerun with --setup-ros-repo. See README deployment troubleshooting.'
  # Bootstrap system Python and native libraries; satisfy project minimums below.
  sudo apt-get install -y ros-noetic-ros-base python3-rosdep python3-catkin-pkg python3-pip \
    python3-numpy python3-yaml python3-pil python3-scipy python3-pyproj \
    fonts-wqy-zenhei libeigen3-dev libgoogle-glog-dev libtbb-dev libpcl-dev libopencv-dev \
    libyaml-cpp-dev liboctomap-dev qtbase5-dev
fi
[[ -f /opt/ros/noetic/setup.bash ]] || fail 'Missing /opt/ros/noetic/setup.bash; install ROS Noetic first.'
# ROS setup files are not guaranteed to be nounset-safe.
set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_PYTHON_VERSION=3
if [[ $mode != build ]]; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  # Noetic is EOL, so its rules must explicitly be included in updates.
  rosdep update --rosdistro=noetic --include-eol-distros
  rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
  rosdep check --from-paths src --ignore-src --rosdistro=noetic
  # Focal apt Python versions can be below requirements.txt. Keep NumPy 1.x
  # for ROS/OpenCV binary compatibility; install as the ordinary user.
  /usr/bin/python3 -m pip install --user -r requirements.txt 'numpy<2'
fi
/usr/bin/python3 - <<'PY'
import numpy, yaml, PIL, scipy, pyproj
import rospy, rospkg
print('Python dependencies OK (system Python).')
PY
[[ $mode != deps ]] || { echo 'Dependencies ready. Rerun with --build-only to compile.'; exit 0; }
command -v catkin_make >/dev/null || fail 'catkin_make is unavailable.'
# Never silently reuse another machine's CMake build directory or erase user data.
if [[ -f build/CMakeCache.txt ]]; then
  cached_root=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' build/CMakeCache.txt)
  [[ $cached_root == "$root_dir/src" ]] || fail "build/ belongs to another workspace ($cached_root). Move build/ and devel/ aside, then rerun."
fi
# Select the active distro even if this branch retained a Melodic symlink.
if [[ -L src/CMakeLists.txt ]]; then
  ln -sfn /opt/ros/noetic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
elif [[ ! -e src/CMakeLists.txt ]]; then
  catkin_init_workspace src
fi
catkin_make -j"$jobs" -l"$jobs" -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF
set +u
source devel/setup.bash
set -u
for package in grid_map_pcl grid_map_demos grid_map_filters; do
  rospack find "$package"
done
[[ -x devel/lib/grid_map_pcl/grid_map_pcl_loader_node ]] || fail 'PCD loader executable missing.'
[[ -x devel/lib/grid_map_demos/filters_demo ]] || fail 'filters_demo executable missing.'
/usr/bin/python3 -m tools.ground_vehicle_tcp.main --help
cat <<DONE
Deployment build completed. Log: $log_file
Next: deploy map/pcd/*.pcd and its ENU origin, verify config, then run:
  cd "$root_dir"
  source devel/setup.bash
  /usr/bin/python3 -m tools.ground_vehicle_tcp.main --config tools/ground_vehicle_tcp/config.yaml
This script does not start the server or install a systemd service.
DONE
