#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  echo 'Vehicle deploy: Ubuntu 18.04/Melodic or Ubuntu 20.04/Noetic (auto-detected).'
  echo 'Options: --jobs N --deps-only --build-only --setup-ros-repo --help'
  echo '18.04 creates a project-local Python 3.8 environment; ROS remains Python 2.'
  exit 0
fi
source /etc/os-release
case "${ID:-}:${VERSION_ID:-}" in
  ubuntu:18.04) exec bash "$script_dir/deploy_melodic.sh" "$@" ;;
  ubuntu:20.04) exec bash "$script_dir/deploy_noetic.sh" "$@" ;;
  *) echo "Unsupported native target: ${PRETTY_NAME:-unknown}. Use Ubuntu 18.04/Melodic or 20.04/Noetic." >&2; exit 1 ;;
esac
