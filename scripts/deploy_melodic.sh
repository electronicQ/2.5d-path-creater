#!/usr/bin/env bash
# Ubuntu 18.04: keep ROS Melodic on Python 2; isolate the Python 3 planner.
set -Eeuo pipefail
jobs=2
mode=all
setup_repo=0
while (($#)); do
  case "$1" in
    --help|-h) echo 'Usage: deploy_vehicle.sh [--jobs N] [--deps-only|--build-only] [--setup-ros-repo]'; exit 0 ;;
    --jobs) [[ $# -ge 2 && $2 =~ ^[1-9][0-9]*$ ]] || { echo 'Invalid --jobs' >&2; exit 2; }; jobs=$2; shift ;;
    --deps-only) [[ $mode == all ]] || { echo 'Conflicting modes' >&2; exit 2; }; mode=deps ;;
    --build-only) [[ $mode == all ]] || { echo 'Conflicting modes' >&2; exit 2; }; mode=build ;;
    --setup-ros-repo) setup_repo=1 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done
[[ $mode != build || $setup_repo == 0 ]] || { echo 'Conflicting options' >&2; exit 2; }
[[ $EUID != 0 ]] || { echo 'Run as the ordinary vehicle user, without sudo.' >&2; exit 1; }
root_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
cd "$root_dir"
mkdir -p log/deploy
log_file="$root_dir/log/deploy/melodic-$(date +%Y%m%d-%H%M%S)-$$.log"
exec > >(tee -a "$log_file") 2>&1
trap 'code=$?; printf "\nDeployment failed (%s), line %s. Log: %s\n" "$code" "$LINENO" "$log_file" >&2; exit "$code"' ERR
fail() { echo "ERROR: $*" >&2; exit 1; }
apt_run() {
  sudo apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 \
    -o Acquire::https::Timeout=30 "$@"
}
source /etc/os-release
[[ ${ID:-}:${VERSION_ID:-} == ubuntu:18.04 ]] || fail 'This script requires Ubuntu 18.04.'
[[ -z ${VIRTUAL_ENV:-} && -z ${CONDA_PREFIX:-} ]] || fail 'Deactivate conda/virtualenv first.'
[[ -z ${ROS_DISTRO:-} || $ROS_DISTRO == melodic ]] || fail 'Open a clean shell without another ROS distribution loaded.'
echo "Ubuntu 18.04 / Melodic; workspace: $root_dir; mode: $mode; jobs: $jobs"
if [[ $mode != build ]]; then
  sudo -v
  apt_run update
  apt_run install -y build-essential cmake pkg-config curl ca-certificates gnupg software-properties-common xz-utils
  sudo add-apt-repository -y universe
  if ((setup_repo)); then
    if sudo grep -RqsE '^[[:space:]]*(deb .*packages\.ros\.org/ros/ubuntu|URIs:.*packages\.ros\.org/ros/ubuntu)' /etc/apt/sources.list /etc/apt/sources.list.d; then
      echo 'Existing official ROS source detected; leaving it unchanged.'
    else
      key_tmp=$(mktemp)
      curl -fL --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o "$key_tmp"
      gpg --show-keys "$key_tmp"
      sudo install -m 0644 "$key_tmp" /usr/share/keyrings/vehicle-ros-archive-keyring.asc
      rm -f "$key_tmp"
      printf 'deb [arch=%s signed-by=/usr/share/keyrings/vehicle-ros-archive-keyring.asc] https://packages.ros.org/ros/ubuntu bionic main\n' "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/vehicle-ros1.list >/dev/null
    fi
  fi
  apt_run update
  apt-cache show ros-melodic-ros-base >/dev/null 2>&1 || fail 'Melodic packages unavailable; configure a working bionic ROS source or use --setup-ros-repo.'
  apt_run install -y ros-melodic-ros-base python-rosdep python-catkin-pkg \
    python-numpy python-yaml python-pathlib2 python-rospkg \
    libeigen3-dev libgoogle-glog-dev libtbb-dev libpcl-dev libopencv-dev \
    libyaml-cpp-dev liboctomap-dev qtbase5-dev \
    libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev \
    libffi-dev liblzma-dev libgdbm-dev libncursesw5-dev uuid-dev
fi
[[ -f /opt/ros/melodic/setup.bash ]] || fail 'ROS Melodic is not installed.'
set +u
source /opt/ros/melodic/setup.bash
set -u
export ROS_PYTHON_VERSION=2
if [[ $mode != build ]]; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
  rosdep update --rosdistro=melodic --include-eol-distros
  rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
  rosdep check --from-paths src --ignore-src --rosdistro=melodic
fi
/usr/bin/python2.7 - <<'PY'
import rospy, rospkg, numpy, yaml, pathlib2
from nav_msgs.msg import OccupancyGrid
print('Melodic Python 2 dependencies OK.')
PY
prefix="$root_dir/.runtime/python38"
planner="$root_dir/.runtime/planner/bin/python"
if [[ $mode != build ]]; then
  mkdir -p .runtime/downloads .runtime/build
  if [[ ! -x $prefix/bin/python3.8 ]]; then
    archive="$root_dir/.runtime/downloads/Python-3.8.20.tar.xz"
    # CPython source is installed inside this workspace, never over /usr/bin/python.
    if [[ ! -f $archive ]]; then
      curl -fL --retry 3 https://www.python.org/ftp/python/3.8.20/Python-3.8.20.tar.xz -o "$archive"
    fi
    printf '%s  %s\n' '6fb89a7124201c61125c0ab4cf7f6894df339a40c02833bfd28ab4d7691fafb4' "$archive" | sha256sum -c -
    tar -xJf "$archive" -C .runtime/build
    (
      cd .runtime/build/Python-3.8.20
      ./configure --prefix="$prefix" --with-ensurepip=install
      make -j"$jobs"
      make install
    )
  fi
  "$prefix/bin/python3.8" -c 'import ssl, venv, bz2, lzma, sqlite3, ctypes'
  if [[ ! -x $planner ]]; then
    "$prefix/bin/python3.8" -m venv "$root_dir/.runtime/planner"
  fi
  env -u PYTHONPATH -u PYTHONHOME "$planner" -m pip install 'pip==24.3.1' 'setuptools==75.3.2' 'wheel==0.45.1'
  # Use wheels to avoid accidental NumPy/SciPy/PROJ source builds on the IPC.
  env -u PYTHONPATH -u PYTHONHOME "$planner" -m pip install --only-binary=:all: -r requirements-melodic.txt
fi
[[ -x $planner ]] || fail 'Planner environment missing. Run deployment without --build-only first.'
env -u PYTHONPATH -u PYTHONHOME "$planner" - <<'PY'
import sys, numpy, scipy, PIL, yaml, pyproj
assert sys.version_info[:2] == (3, 8), sys.version
print('Isolated Python 3.8 planner dependencies OK.')
PY
[[ $mode != deps ]] || { echo 'Dependencies ready. Next: bash scripts/deploy_vehicle.sh --build-only'; exit 0; }
if [[ -f build/CMakeCache.txt ]]; then
  cached_root=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' build/CMakeCache.txt)
  [[ $cached_root == "$root_dir/src" ]] || fail "Old build belongs to $cached_root. Move build/ and devel/ aside and rerun."
  cached_python=$(sed -n 's/^PYTHON_EXECUTABLE:FILEPATH=//p' build/CMakeCache.txt)
  [[ -z $cached_python || $cached_python == /usr/bin/python2.7 ]] || fail 'Old build uses another Python. Move build/ and devel/ aside before compiling Melodic.'
fi
if [[ -L src/CMakeLists.txt ]]; then
  # The repository may have retained an absolute Noetic top-level symlink.
  ln -sfn /opt/ros/melodic/share/catkin/cmake/toplevel.cmake src/CMakeLists.txt
elif [[ ! -e src/CMakeLists.txt ]]; then
  catkin_init_workspace src
fi
catkin_make -j"$jobs" -l"$jobs" -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python2.7 -DCATKIN_ENABLE_TESTING=OFF
set +u
source devel/setup.bash
set -u
for package in grid_map_pcl grid_map_demos grid_map_filters; do rospack find "$package"; done
[[ -x devel/lib/grid_map_pcl/grid_map_pcl_loader_node && -x devel/lib/grid_map_demos/filters_demo ]] || fail 'Required ROS executables missing.'
bash scripts/run_vehicle.sh --help
cat <<DONE
Deployment completed. Log: $log_file
Prepare map/pcd/*.pcd, ENU metadata and TCP config, then run:
  bash scripts/run_vehicle.sh
Runtime and venv paths are machine-specific. Rerun deployment after moving the project.
DONE
