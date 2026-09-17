#!/usr/bin/env bash
set -eu
python3 -m venv .venv-ground-station
.venv-ground-station/bin/python -m pip install --upgrade pip
.venv-ground-station/bin/python -m pip install -r tools/ground_station/requirements.txt pyside6-deploy
.venv-ground-station/bin/pyside6-deploy tools/ground_station/main.py --name vehicle-ground-station --force
