$ErrorActionPreference = "Stop"
py -3 -m venv .venv-ground-station
.\.venv-ground-station\Scripts\python.exe -m pip install --upgrade pip
.\.venv-ground-station\Scripts\python.exe -m pip install -r tools/ground_station/requirements.txt pyside6-deploy
.\.venv-ground-station\Scripts\pyside6-deploy.exe tools/ground_station/main.py --name vehicle-ground-station --force
