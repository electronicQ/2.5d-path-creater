"""Compact PySide6 GUI for the complete vehicle workflow."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

from .client import GatewayClient
from .models import GeoPoint, PlannerRequest
from .workers import GatewayWorker

try:
    from PySide6.QtCore import QThread, Qt
    from PySide6.QtGui import QPixmap
    from PySide6.QtWidgets import (
        QApplication, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
        QMainWindow, QMessageBox, QPushButton, QPlainTextEdit, QProgressBar, QSpinBox,
        QDoubleSpinBox, QTabWidget, QVBoxLayout, QWidget,
    )
except ImportError as exc:  # pragma: no cover - exercised only when GUI is launched.
    raise RuntimeError("PySide6 is required to run the ground station GUI") from exc


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Vehicle Ground Station")
        self.resize(1100, 760)
        self.client = GatewayClient("127.0.0.1")
        self.worker_thread: Optional[QThread] = None
        self.worker: Optional[GatewayWorker] = None
        self.local_root = Path.home() / "vehicle_ground_station"
        self._build_ui()

    def _build_ui(self) -> None:
        tabs = QTabWidget()
        tabs.addTab(self._connection_tab(), "连接")
        tabs.addTab(self._upload_tab(), "地图上传")
        tabs.addTab(self._convert_tab(), "地图转换")
        tabs.addTab(self._plan_tab(), "路径规划")
        tabs.addTab(self._result_tab(), "结果")
        self.setCentralWidget(tabs)

    def _connection_tab(self) -> QWidget:
        page, layout = self._page()
        form = QFormLayout()
        self.host = QLineEdit("127.0.0.1")
        self.port = QSpinBox(); self.port.setRange(1, 65535); self.port.setValue(19090)
        self.token = QLineEdit(); self.token.setEchoMode(QLineEdit.Password)
        form.addRow("车端 IP", self.host); form.addRow("端口", self.port); form.addRow("令牌", self.token)
        layout.addLayout(form)
        button = QPushButton("测试连接"); button.clicked.connect(self.connect_gateway); layout.addWidget(button)
        self.connection_status = QLabel("未连接"); layout.addWidget(self.connection_status)
        return page

    def _upload_tab(self) -> QWidget:
        page, layout = self._page(); form = QFormLayout()
        self.pcd_path = QLineEdit(); browse = QPushButton("选择"); browse.clicked.connect(self.choose_pcd)
        row = QHBoxLayout(); row.addWidget(self.pcd_path); row.addWidget(browse); form.addRow("PCD", row)
        self.map_id = QLineEdit("map_001"); self.pcd_name = QLineEdit("scans_Global_ENU.pcd")
        self.origin_lat = self._number(22.0); self.origin_lon = self._number(113.0); self.origin_alt = self._number(0.0)
        form.addRow("地图 ID", self.map_id); form.addRow("PCD 名称", self.pcd_name); form.addRow("原点纬度", self.origin_lat); form.addRow("原点经度", self.origin_lon); form.addRow("原点高程", self.origin_alt)
        layout.addLayout(form); self.upload_button = QPushButton("上传点云和 ENU 原点"); self.upload_button.clicked.connect(self.upload); layout.addWidget(self.upload_button)
        self.upload_progress = QProgressBar(); layout.addWidget(self.upload_progress); return page

    def _convert_tab(self) -> QWidget:
        page, layout = self._page(); form = QFormLayout(); self.convert_pgm = QLineEdit("map_001")
        form.addRow("地图 ID", self.map_id); form.addRow("PCD 名称", self.pcd_name); form.addRow("PGM 名称", self.convert_pgm)
        layout.addLayout(form); button = QPushButton("开始转换并保存 PGM"); button.clicked.connect(self.convert); layout.addWidget(button); self.convert_status = QLabel(); layout.addWidget(self.convert_status); return page

    def _plan_tab(self) -> QWidget:
        page, layout = self._page(); form = QFormLayout(); self.plan_pgm = QLineEdit("map_001")
        self.start_fields = [self._number(22.0), self._number(113.0), self._number(0.0)]
        self.goal_fields = [self._number(22.0), self._number(113.0), self._number(0.0)]
        form.addRow("PGM 名称", self.plan_pgm); form.addRow("起点纬度", self.start_fields[0]); form.addRow("起点经度", self.start_fields[1]); form.addRow("起点高程", self.start_fields[2]); form.addRow("目标纬度", self.goal_fields[0]); form.addRow("目标经度", self.goal_fields[1]); form.addRow("目标高程", self.goal_fields[2])
        self.overrides = QPlainTextEdit('{"inflation_radius_m": 1.6, "d_safe_m": 4.0, "smooth_paths": true}')
        layout.addLayout(form); layout.addWidget(QLabel("高级规划参数 JSON")); layout.addWidget(self.overrides); button = QPushButton("开始规划"); button.clicked.connect(self.plan); layout.addWidget(button); self.plan_status = QLabel(); layout.addWidget(self.plan_status); return page

    def _result_tab(self) -> QWidget:
        page, layout = self._page(); self.result_status = QLabel("暂无结果"); layout.addWidget(self.result_status); self.preview = QLabel(); self.preview.setAlignment(Qt.AlignCenter); layout.addWidget(self.preview); return page

    def _page(self):
        page = QWidget(); layout = QVBoxLayout(page); return page, layout

    def _number(self, value: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox(); box.setRange(-1000000000, 1000000000); box.setDecimals(9); box.setValue(value); return box

    def _prepare_worker(self) -> GatewayWorker:
        if self.worker is not None and self.worker_thread is not None and self.worker_thread.isRunning():
            return self.worker
        self.client = GatewayClient(self.host.text().strip(), self.port.value(), self.token.text())
        thread = QThread(self); worker = GatewayWorker(self.client); worker.moveToThread(thread); worker.failed.connect(self.show_error); worker.progress.connect(self.show_progress); worker.completed.connect(self.show_completed); thread.finished.connect(worker.deleteLater); thread.start(); self.worker_thread, self.worker = thread, worker; return worker

    def connect_gateway(self) -> None:
        worker = self._prepare_worker(); worker.connected.connect(self.on_connected); worker.connect_requested.emit()

    def choose_pcd(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择 PCD", "", "Point Cloud (*.pcd)"); self.pcd_path.setText(path)

    def upload(self) -> None:
        if not self.pcd_path.text(): return self.show_error("请选择 PCD 文件")
        self._prepare_worker().upload_requested.emit(self.pcd_path.text(), self.map_id.text(), self.pcd_name.text(), {"latitude": self.origin_lat.value(), "longitude": self.origin_lon.value(), "altitude": self.origin_alt.value()})

    def convert(self) -> None:
        self._prepare_worker().convert_requested.emit(self.map_id.text(), self.pcd_name.text(), self.convert_pgm.text())

    def plan(self) -> None:
        try: overrides = json.loads(self.overrides.toPlainText() or "{}")
        except ValueError as exc: return self.show_error("高级参数 JSON 无效：{}".format(exc))
        request = PlannerRequest(self.map_id.text(), self.plan_pgm.text(), GeoPoint(*(box.value() for box in self.start_fields)), GeoPoint(*(box.value() for box in self.goal_fields)), overrides)
        self._prepare_worker().plan_requested.emit(request)

    def show_progress(self, data: Dict[str, Any]) -> None:
        if data.get("operation") == "upload": self.upload_progress.setValue(int(100 * data["done"] / max(1, data["total"])))
        self.plan_status.setText(json.dumps(data, ensure_ascii=False))

    def on_connected(self, data: Dict[str, Any]) -> None:
        self.connection_status.setText("已连接：协议 {}".format(data.get("protocol")))

    def show_completed(self, data: Dict[str, Any]) -> None:
        self.result_status.setText(json.dumps(data, ensure_ascii=False, indent=2)); self.convert_status.setText("完成")
        directory = data.get("directory"); overlay = Path(directory) / "overlay.png" if directory else None
        if overlay and overlay.is_file(): self.preview.setPixmap(QPixmap(str(overlay)).scaled(900, 600, Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def show_error(self, message: str) -> None:
        QMessageBox.critical(self, "车端错误", message)


def run() -> int:
    app = QApplication([]); window = MainWindow(); window.show(); return app.exec()
