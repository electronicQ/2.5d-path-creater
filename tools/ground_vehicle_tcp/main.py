"""Start the Excel-protocol TCP server on the vehicle computer."""

from __future__ import annotations

import argparse
import asyncio
import logging
from pathlib import Path

import yaml

from .server import NavigationServer
from .workflow import NavigationWorkflow


def _project_path(project_root: Path, value: str) -> Path:
    path = Path(value)
    return (project_root / path).resolve() if not path.is_absolute() else path.resolve()


def build_server(config_path: Path) -> NavigationServer:
    config_path = Path(config_path).resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    project_root = _project_path(config_path.parent, str(config.get("project_root", "../..")))
    data_root = _project_path(project_root, str(config.get("data_root", "var/ground_vehicle_tcp")))
    workflow = NavigationWorkflow(
        project_root=project_root,
        data_root=data_root,
        pcd_path=_project_path(project_root, str(config.get("pcd_path", "map/pcd/scans_Global_ENU.pcd"))),
        pcd_name=str(config.get("pcd_name", "scans_Global_ENU.pcd")),
        metadata_path=_project_path(project_root, str(config.get("metadata_path", "tools/path_creater/config/map_enu.meta.yaml"))),
        planner_config_path=_project_path(project_root, str(config.get("planner_config_path", "tools/path_creater/config/config.yaml"))),
        conversion_timeout_s=int(config.get("conversion_timeout_s", 1800)),
    )
    return NavigationServer(workflow, host=str(config.get("host", "0.0.0.0")), port=int(config.get("port", 8079)))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Ground vehicle navigation TCP server")
    parser.add_argument("--config", default="tools/ground_vehicle_tcp/config.yaml")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    server = build_server(Path(args.config))
    try:
        asyncio.run(server.run_forever())
    except KeyboardInterrupt:
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
