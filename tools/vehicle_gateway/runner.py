"""Subprocess orchestration for ROS map conversion and offline planning."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

import yaml


class PipelineError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


class PipelineRunner:
    def __init__(self, project_root: Path, timeout_s: float = 1800.0, poll_s: float = 1.0, env: Optional[Dict[str, str]] = None) -> None:
        self.project_root = Path(project_root).resolve()
        self.timeout_s = timeout_s
        self.poll_s = poll_s
        self.env = dict(env or os.environ)
        self.processes: List[subprocess.Popen] = []

    def _start(self, command: List[str], log_path: Path) -> subprocess.Popen:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log = log_path.open("ab")
        try:
            process = subprocess.Popen(command, cwd=str(self.project_root), stdout=log, stderr=subprocess.STDOUT, env=self.env, start_new_session=True)
        except OSError:
            log.close()
            raise
        process._gateway_log = log  # type: ignore[attr-defined]
        self.processes.append(process)
        return process

    def _wait_topic(self, topic: str, log_path: Path, deadline: float) -> None:
        command = ["rostopic", "echo", "-n", "1", topic]
        while time.monotonic() < deadline:
            try:
                result = subprocess.run(command, cwd=str(self.project_root), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=max(self.poll_s, 1.0), env=self.env)
            except (OSError, subprocess.TimeoutExpired):
                result = None
            if result is not None and result.returncode == 0 and result.stdout:
                with log_path.open("ab") as log:
                    log.write(result.stdout)
                return
            time.sleep(self.poll_s)
        raise PipelineError("MAP_TOPIC_TIMEOUT", "timed out waiting for {}".format(topic))

    def convert(self, pcd_name: str, output_base: Path, log_path: Path, rviz: bool = False) -> Dict[str, str]:
        self.processes = []
        deadline = time.monotonic() + self.timeout_s
        try:
            self._start(["roslaunch", "grid_map_pcl", "grid_map_pcl_loader_node.launch", "pcd_filename:=" + pcd_name, "enable_rviz:=" + str(rviz).lower()], log_path)
            self._wait_topic("/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud", log_path, deadline)
            self._start(["roslaunch", "grid_map_demos", "filters_demo_gai.launch", "enable_rviz:=" + str(rviz).lower()], log_path)
            self._wait_topic("/grid_map_visualization/traversability_grid", log_path, deadline)
            saver = self._start(["python3", "tools/pgm_saver/save_continuous_map.py", "-n", str(output_base)], log_path)
            while saver.poll() is None:
                if time.monotonic() >= deadline:
                    raise PipelineError("JOB_TIMEOUT", "map saver timed out")
                time.sleep(self.poll_s)
            if saver.returncode != 0:
                raise PipelineError("PROCESS_FAILED", "map saver exited with {}".format(saver.returncode))
            pgm = Path(str(output_base) + ".pgm")
            yaml_path = Path(str(output_base) + ".yaml")
            if not pgm.is_file() or not yaml_path.is_file():
                raise PipelineError("OUTPUT_INVALID", "map saver did not produce PGM and YAML")
            with yaml_path.open("r", encoding="utf-8") as stream:
                map_yaml = yaml.safe_load(stream) or {}
            if map_yaml.get("image") != pgm.name:
                raise PipelineError("OUTPUT_INVALID", "map YAML does not reference its PGM")
            return {"pgm": str(pgm), "yaml": str(yaml_path)}
        except OSError as exc:
            raise PipelineError("ROS_UNAVAILABLE", str(exc)) from exc
        finally:
            self.cleanup()

    def plan(self, config_path: Path, output_dir: Path, log_path: Path) -> Dict[str, str]:
        output_dir.mkdir(parents=True, exist_ok=True)
        try:
            with log_path.open("ab") as log:
                result = subprocess.run(["python3", "-m", "tools.path_creater.main", "--config", str(config_path)], cwd=str(self.project_root), stdout=log, stderr=subprocess.STDOUT, timeout=self.timeout_s, env=self.env)
        except subprocess.TimeoutExpired as exc:
            raise PipelineError("JOB_TIMEOUT", "planner timed out") from exc
        except OSError as exc:
            raise PipelineError("PLANNER_FAILED", str(exc)) from exc
        if result.returncode != 0:
            raise PipelineError("PLANNER_FAILED", "planner exited with {}".format(result.returncode))
        names = [
            "traj_distance_geo.csv", "traj_distance_enu.csv", "traj_smooth_geo.csv",
            "traj_smooth_enu.csv", "traj_cost_aware_geo.csv", "traj_cost_aware_enu.csv",
            "overlay.png", "config.resolved.yaml",
        ]
        artifacts = {name: str(output_dir / name) for name in names if (output_dir / name).is_file()}
        required = names[:6]
        if any(name not in artifacts for name in required):
            raise PipelineError("OUTPUT_INVALID", "planner did not produce all trajectory files")
        return artifacts

    def cleanup(self) -> None:
        for process in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.0)
                except (OSError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except OSError:
                        pass
            log = getattr(process, "_gateway_log", None)
            if log:
                log.close()
        self.processes = []
