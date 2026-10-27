#!/usr/bin/env bash
# True only when both executable paths resolve to the same installed binary.
# /usr/bin/python and /usr/bin/python2 can be valid aliases for python2.7.
vehicle_same_python_binary() {
  [[ -x $1 && -x $2 ]] || return 1
  local cached_target expected_target
  cached_target=$(readlink -f -- "$1") || return 1
  expected_target=$(readlink -f -- "$2") || return 1
  [[ $cached_target == "$expected_target" ]]
}
