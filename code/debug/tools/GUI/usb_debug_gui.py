from __future__ import annotations

import argparse
import os
import sys
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np

try:
    try:
        from PySide6 import QtCore, QtGui, QtWidgets
        qt_binding = "PySide6"
    except ImportError:
        from PyQt6 import QtCore, QtGui, QtWidgets
        qt_binding = "PyQt6"

    os.environ.setdefault("PYQTGRAPH_QT_LIB", qt_binding)

    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    from serial import SerialException
    from serial.tools import list_ports
except ImportError as exc:
    print(
        "Missing GUI dependencies. Install with: "
        "python -m pip install -r tools/requirements-gui.txt",
        file=sys.stderr,
    )
    raise SystemExit(str(exc))

from usb_protocol import GimbalFeedback, mode_name
from usb_session import SessionEvent, UsbSession

PLOT_WINDOW_S = 10.0
MAX_LOG_LINES = 800

pg.setConfigOptions(antialias=True, foreground="#E6ECF2", background="#10171F")

QT_HORIZONTAL = getattr(getattr(QtCore.Qt, "Orientation", QtCore.Qt), "Horizontal")
QT_VERTICAL = getattr(getattr(QtCore.Qt, "Orientation", QtCore.Qt), "Vertical")
QT_FONT_MONOSPACE = getattr(getattr(QtGui.QFont, "StyleHint", QtGui.QFont), "Monospace")


class CubeView(gl.GLViewWidget):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(320)
        self.setCameraPosition(distance=6.0, elevation=18.0, azimuth=36.0)

        grid = gl.GLGridItem()
        grid.scale(0.25, 0.25, 0.25)
        grid.translate(0.0, 0.0, -1.0)
        self.addItem(grid)

        axis = gl.GLAxisItem()
        axis.setSize(1.8, 1.8, 1.8)
        self.addItem(axis)

        vertices = np.array(
            [
                [-0.5, -0.5, -0.5],
                [0.5, -0.5, -0.5],
                [0.5, 0.5, -0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, 0.5],
                [-0.5, 0.5, 0.5],
            ],
            dtype=float,
        )
        faces = np.array(
            [
                [0, 1, 2], [0, 2, 3],
                [4, 5, 6], [4, 6, 7],
                [0, 1, 5], [0, 5, 4],
                [2, 3, 7], [2, 7, 6],
                [1, 2, 6], [1, 6, 5],
                [3, 0, 4], [3, 4, 7],
            ],
            dtype=np.uint32,
        )
        mesh_data = gl.MeshData(vertexes=vertices, faces=faces)
        self.cube = gl.GLMeshItem(
            meshdata=mesh_data,
            smooth=False,
            drawFaces=True,
            drawEdges=True,
            edgeColor=(0.94, 0.97, 1.0, 1.0),
            color=(0.20, 0.72, 0.82, 0.72),
        )
        self.addItem(self.cube)
        self.set_rotation(0.0, 0.0, 0.0)

    def set_rotation(self, yaw_deg: float, pitch_deg: float, roll_deg: float) -> None:
        self.cube.resetTransform()
        self.cube.scale(1.6, 1.0, 0.45)
        self.cube.rotate(yaw_deg, 0, 0, 1)
        self.cube.rotate(pitch_deg, 0, 1, 0)
        self.cube.rotate(roll_deg, 1, 0, 0)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, initial_port: str | None = None) -> None:
        super().__init__()
        self.setWindowTitle("USB Gimbal Debug Console")
        self.resize(1480, 920)

        self.session = UsbSession(auto_search_on_handshake=True, keepalive_enabled=True)
        self.plot_times: deque[float] = deque()
        self.plot_yaw: deque[float] = deque()
        self.plot_pitch: deque[float] = deque()
        self.plot_roll: deque[float] = deque()
        self.log_lines: deque[str] = deque(maxlen=MAX_LOG_LINES)
        self.plot_origin_monotonic: float | None = None
        self._initial_port = initial_port

        self._build_ui()
        self._apply_style()
        self.refresh_ports(select_port=initial_port)
        self._bind_signals()
        self._apply_session_preferences()

        self.event_timer = QtCore.QTimer(self)
        self.event_timer.setInterval(33)
        self.event_timer.timeout.connect(self.process_session_events)
        self.event_timer.start()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        root_layout = QtWidgets.QVBoxLayout(central)
        root_layout.setContentsMargins(18, 18, 18, 18)
        root_layout.setSpacing(14)

        header = QtWidgets.QFrame()
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(18, 14, 18, 14)
        header_layout.setSpacing(14)

        title_layout = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("USB Gimbal Debug Console")
        title.setObjectName("heroTitle")
        subtitle = QtWidgets.QLabel("Serial protocol debugging, live attitude rendering, and motion traces")
        subtitle.setObjectName("heroSubtitle")
        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)
        header_layout.addLayout(title_layout)
        header_layout.addStretch(1)

        self.connection_status = QtWidgets.QLabel("DISCONNECTED")
        self.connection_status.setObjectName("connectionBadge")
        header_layout.addWidget(self.connection_status)

        port_label = QtWidgets.QLabel("Port")
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.refresh_button = QtWidgets.QPushButton("Refresh")
        baud_label = QtWidgets.QLabel("Baud")
        self.baudrate_spin = QtWidgets.QSpinBox()
        self.baudrate_spin.setRange(1200, 3000000)
        self.baudrate_spin.setValue(115200)
        self.connect_button = QtWidgets.QPushButton("Connect")

        header_layout.addWidget(port_label)
        header_layout.addWidget(self.port_combo)
        header_layout.addWidget(self.refresh_button)
        header_layout.addWidget(baud_label)
        header_layout.addWidget(self.baudrate_spin)
        header_layout.addWidget(self.connect_button)

        body_splitter = QtWidgets.QSplitter(QT_HORIZONTAL)
        body_splitter.setChildrenCollapsible(False)

        left_panel = QtWidgets.QFrame()
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(12)

        status_group = QtWidgets.QGroupBox("Session")
        status_grid = QtWidgets.QGridLayout(status_group)
        status_grid.setHorizontalSpacing(12)
        status_grid.setVerticalSpacing(10)
        self.mode_value = QtWidgets.QLabel("DISCONNECTED")
        self.mode_value.setObjectName("modeBadge")
        self.handshake_value = QtWidgets.QLabel("-")
        self.device_timestamp_value = QtWidgets.QLabel("-")
        self.yaw_value = QtWidgets.QLabel("0.000 deg")
        self.pitch_value = QtWidgets.QLabel("0.000 deg")
        self.roll_value = QtWidgets.QLabel("0.000 deg")
        status_grid.addWidget(QtWidgets.QLabel("Mode"), 0, 0)
        status_grid.addWidget(self.mode_value, 0, 1)
        status_grid.addWidget(QtWidgets.QLabel("Handshake ts"), 1, 0)
        status_grid.addWidget(self.handshake_value, 1, 1)
        status_grid.addWidget(QtWidgets.QLabel("Device ts"), 2, 0)
        status_grid.addWidget(self.device_timestamp_value, 2, 1)
        status_grid.addWidget(QtWidgets.QLabel("Yaw"), 3, 0)
        status_grid.addWidget(self.yaw_value, 3, 1)
        status_grid.addWidget(QtWidgets.QLabel("Pitch"), 4, 0)
        status_grid.addWidget(self.pitch_value, 4, 1)
        status_grid.addWidget(QtWidgets.QLabel("Roll"), 5, 0)
        status_grid.addWidget(self.roll_value, 5, 1)

        control_group = QtWidgets.QGroupBox("Controls")
        control_layout = QtWidgets.QVBoxLayout(control_group)
        control_layout.setSpacing(10)

        command_grid = QtWidgets.QGridLayout()
        command_grid.setHorizontalSpacing(10)
        command_grid.setVerticalSpacing(10)
        self.handshake_button = QtWidgets.QPushButton("Handshake")
        self.search_button = QtWidgets.QPushButton("Search")
        self.disable_button = QtWidgets.QPushButton("Disable")
        self.unlock_button = QtWidgets.QPushButton("Unlock")
        self.stop_button = QtWidgets.QPushButton("Stop Keepalive")
        command_grid.addWidget(self.handshake_button, 0, 0)
        command_grid.addWidget(self.search_button, 0, 1)
        command_grid.addWidget(self.disable_button, 1, 0)
        command_grid.addWidget(self.unlock_button, 1, 1)
        command_grid.addWidget(self.stop_button, 2, 0, 1, 2)
        control_layout.addLayout(command_grid)

        auto_group = QtWidgets.QGroupBox("Auto Aim")
        auto_grid = QtWidgets.QGridLayout(auto_group)
        auto_grid.setHorizontalSpacing(10)
        auto_grid.setVerticalSpacing(10)
        self.auto_yaw_spin = QtWidgets.QDoubleSpinBox()
        self.auto_yaw_spin.setRange(-180.0, 180.0)
        self.auto_yaw_spin.setDecimals(3)
        self.auto_yaw_spin.setSingleStep(0.5)
        self.auto_pitch_spin = QtWidgets.QDoubleSpinBox()
        self.auto_pitch_spin.setRange(-90.0, 90.0)
        self.auto_pitch_spin.setDecimals(3)
        self.auto_pitch_spin.setSingleStep(0.5)
        self.auto_button = QtWidgets.QPushButton("Send Auto")
        auto_grid.addWidget(QtWidgets.QLabel("Yaw (deg)"), 0, 0)
        auto_grid.addWidget(self.auto_yaw_spin, 0, 1)
        auto_grid.addWidget(QtWidgets.QLabel("Pitch (deg)"), 1, 0)
        auto_grid.addWidget(self.auto_pitch_spin, 1, 1)
        auto_grid.addWidget(self.auto_button, 2, 0, 1, 2)
        control_layout.addWidget(auto_group)

        keepalive_group = QtWidgets.QGroupBox("Keepalive")
        keepalive_grid = QtWidgets.QGridLayout(keepalive_group)
        keepalive_grid.setHorizontalSpacing(10)
        keepalive_grid.setVerticalSpacing(10)
        self.keepalive_checkbox = QtWidgets.QCheckBox("Enable periodic keepalive")
        self.keepalive_checkbox.setChecked(True)
        self.search_period_spin = QtWidgets.QSpinBox()
        self.search_period_spin.setRange(10, 5000)
        self.search_period_spin.setValue(200)
        self.search_period_spin.setSuffix(" ms")
        self.auto_period_spin = QtWidgets.QSpinBox()
        self.auto_period_spin.setRange(5, 1000)
        self.auto_period_spin.setValue(20)
        self.auto_period_spin.setSuffix(" ms")
        keepalive_grid.addWidget(self.keepalive_checkbox, 0, 0, 1, 2)
        keepalive_grid.addWidget(QtWidgets.QLabel("Search period"), 1, 0)
        keepalive_grid.addWidget(self.search_period_spin, 1, 1)
        keepalive_grid.addWidget(QtWidgets.QLabel("Auto period"), 2, 0)
        keepalive_grid.addWidget(self.auto_period_spin, 2, 1)
        control_layout.addWidget(keepalive_group)

        left_layout.addWidget(status_group)
        left_layout.addWidget(control_group)
        left_layout.addStretch(1)

        right_splitter = QtWidgets.QSplitter(QT_VERTICAL)
        right_splitter.setChildrenCollapsible(False)

        visual_card = QtWidgets.QFrame()
        visual_layout = QtWidgets.QVBoxLayout(visual_card)
        visual_layout.setContentsMargins(12, 12, 12, 12)
        visual_layout.setSpacing(10)
        visual_title = QtWidgets.QLabel("3D Attitude")
        visual_title.setObjectName("sectionTitle")
        self.cube_view = CubeView()
        visual_layout.addWidget(visual_title)
        visual_layout.addWidget(self.cube_view, 1)

        plot_card = QtWidgets.QFrame()
        plot_layout = QtWidgets.QVBoxLayout(plot_card)
        plot_layout.setContentsMargins(12, 12, 12, 12)
        plot_layout.setSpacing(10)
        plot_header = QtWidgets.QHBoxLayout()
        plot_title = QtWidgets.QLabel("Live Motion Traces")
        plot_title.setObjectName("sectionTitle")
        plot_header.addWidget(plot_title)
        plot_header.addStretch(1)
        self.clear_plot_button = QtWidgets.QPushButton("Clear Plot")
        plot_header.addWidget(self.clear_plot_button)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.setLabel("left", "Angle", units="deg")
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.addLegend(offset=(16, 8))
        self.yaw_curve = self.plot_widget.plot(name="yaw", pen=pg.mkPen("#5BD6FF", width=2))
        self.pitch_curve = self.plot_widget.plot(name="pitch", pen=pg.mkPen("#FFB347", width=2))
        self.roll_curve = self.plot_widget.plot(name="roll", pen=pg.mkPen("#7EF0A3", width=2))
        plot_layout.addLayout(plot_header)
        plot_layout.addWidget(self.plot_widget, 1)

        right_splitter.addWidget(visual_card)
        right_splitter.addWidget(plot_card)
        right_splitter.setStretchFactor(0, 3)
        right_splitter.setStretchFactor(1, 2)

        body_splitter.addWidget(left_panel)
        body_splitter.addWidget(right_splitter)
        body_splitter.setStretchFactor(0, 0)
        body_splitter.setStretchFactor(1, 1)
        body_splitter.setSizes([380, 980])

        log_card = QtWidgets.QFrame()
        log_layout = QtWidgets.QVBoxLayout(log_card)
        log_layout.setContentsMargins(12, 12, 12, 12)
        log_layout.setSpacing(10)
        log_header = QtWidgets.QHBoxLayout()
        log_title = QtWidgets.QLabel("Session Log")
        log_title.setObjectName("sectionTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)
        self.pause_log_checkbox = QtWidgets.QCheckBox("Pause")
        self.clear_log_button = QtWidgets.QPushButton("Clear")
        self.export_log_button = QtWidgets.QPushButton("Export")
        log_header.addWidget(self.pause_log_checkbox)
        log_header.addWidget(self.clear_log_button)
        log_header.addWidget(self.export_log_button)
        self.log_output = QtWidgets.QPlainTextEdit()
        self.log_output.setReadOnly(True)
        font = QtGui.QFont("Consolas")
        font.setStyleHint(QT_FONT_MONOSPACE)
        self.log_output.setFont(font)
        log_layout.addLayout(log_header)
        log_layout.addWidget(self.log_output, 1)

        root_layout.addWidget(header)
        root_layout.addWidget(body_splitter, 1)
        root_layout.addWidget(log_card, 1)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Ready")
        self._set_connected_state(False)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background: #0B1118;
                color: #E6ECF2;
                font-family: Segoe UI;
                font-size: 13px;
            }
            QFrame, QGroupBox {
                background: #121B25;
                border: 1px solid #243140;
                border-radius: 14px;
            }
            QGroupBox {
                margin-top: 12px;
                padding-top: 12px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 6px;
                color: #A9B7C6;
            }
            QLabel#heroTitle {
                font-size: 28px;
                font-weight: 700;
                color: #F3F7FB;
            }
            QLabel#heroSubtitle {
                color: #8EA2B6;
                font-size: 13px;
            }
            QLabel#sectionTitle {
                font-size: 16px;
                font-weight: 700;
                color: #F0F4F8;
            }
            QLabel#connectionBadge, QLabel#modeBadge {
                padding: 6px 12px;
                border-radius: 11px;
                background: #1C2A38;
                color: #F0F4F8;
                font-weight: 700;
            }
            QPushButton {
                background: #173042;
                border: 1px solid #2F556D;
                border-radius: 10px;
                padding: 8px 14px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #1D4158;
            }
            QPushButton:disabled {
                color: #6C7A89;
                background: #111820;
                border-color: #1E2731;
            }
            QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit {
                background: #0D141C;
                border: 1px solid #2A3948;
                border-radius: 10px;
                padding: 6px 8px;
            }
            QCheckBox {
                spacing: 8px;
            }
            QSplitter::handle {
                background: #1A2632;
            }
            """
        )

    def _bind_signals(self) -> None:
        self.refresh_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self.toggle_connection)
        self.handshake_button.clicked.connect(self.send_handshake)
        self.search_button.clicked.connect(self.send_search)
        self.disable_button.clicked.connect(self.send_disable)
        self.unlock_button.clicked.connect(self.send_unlock)
        self.stop_button.clicked.connect(self.stop_keepalive)
        self.auto_button.clicked.connect(self.send_auto)
        self.clear_plot_button.clicked.connect(self.clear_plot)
        self.clear_log_button.clicked.connect(self.clear_log)
        self.export_log_button.clicked.connect(self.export_log)
        self.keepalive_checkbox.toggled.connect(self._apply_session_preferences)
        self.search_period_spin.valueChanged.connect(self._apply_session_preferences)
        self.auto_period_spin.valueChanged.connect(self._apply_session_preferences)

    def refresh_ports(self, select_port: str | None = None) -> None:
        current = select_port or self.port_combo.currentText()
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current and current in ports:
            self.port_combo.setCurrentText(current)
        elif self._initial_port and self._initial_port in ports:
            self.port_combo.setCurrentText(self._initial_port)
        self.port_combo.blockSignals(False)
        self.statusBar().showMessage(f"Detected {len(ports)} serial ports", 3000)

    def toggle_connection(self) -> None:
        if self.session.connected:
            self.session.disconnect()
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.warning(self, "No Port", "Select a serial port first.")
            return

        try:
            self.session.connect(port, baudrate=self.baudrate_spin.value())
            self._apply_session_preferences()
        except SerialException as exc:
            self.append_log(f"Serial open failed: {exc}")
            QtWidgets.QMessageBox.critical(self, "Connection Error", str(exc))
            self._set_connected_state(False)

    def send_handshake(self) -> None:
        if self.session.connected:
            self.session.send_handshake()

    def send_search(self) -> None:
        if self.session.connected:
            self.session.start_search(periodic=self.keepalive_checkbox.isChecked())

    def send_auto(self) -> None:
        if self.session.connected:
            self.session.start_auto(
                self.auto_yaw_spin.value(),
                self.auto_pitch_spin.value(),
                periodic=self.keepalive_checkbox.isChecked(),
            )

    def send_disable(self) -> None:
        if self.session.connected:
            self.session.disable()

    def send_unlock(self) -> None:
        if self.session.connected:
            self.session.unlock()

    def stop_keepalive(self) -> None:
        if self.session.connected:
            self.session.stop_keepalive()

    def _apply_session_preferences(self) -> None:
        self.session.set_keepalive_enabled(self.keepalive_checkbox.isChecked())
        self.session.set_periods(
            self.search_period_spin.value() / 1000.0,
            self.auto_period_spin.value() / 1000.0,
        )

    def process_session_events(self) -> None:
        for event in self.session.poll_events(limit=400):
            self._handle_event(event)

    def _handle_event(self, event: SessionEvent) -> None:
        if event.kind not in {"feedback"}:
            self.append_log(event.text)

        if event.kind == "connected":
            self._set_connected_state(True)
            self.statusBar().showMessage(f"Connected to {event.port}", 3000)
            return

        if event.kind == "disconnected":
            self._set_connected_state(False)
            self._set_mode_label("DISCONNECTED")
            self.connection_status.setText("DISCONNECTED")
            self.statusBar().showMessage("Disconnected", 3000)
            return

        if event.kind == "handshake" and event.timestamp_ms is not None:
            self.handshake_value.setText(str(event.timestamp_ms))
            return

        if event.kind == "feedback" and event.feedback is not None:
            self._update_feedback(event.feedback)
            return

        if event.kind == "disabled":
            self._set_mode_label("DISABLED")
            self.statusBar().showMessage("Device entered disabled mode", 5000)
            return

        if event.kind == "error":
            self.statusBar().showMessage(event.text, 5000)

    def _update_feedback(self, feedback: GimbalFeedback) -> None:
        self.yaw_value.setText(f"{feedback.yaw_deg:.3f} deg")
        self.pitch_value.setText(f"{feedback.pitch_deg:.3f} deg")
        self.roll_value.setText(f"{feedback.roll_deg:.3f} deg")
        self.device_timestamp_value.setText(str(feedback.timestamp_ms))
        self._set_mode_label(mode_name(feedback.mode))
        self.cube_view.set_rotation(feedback.yaw_deg, feedback.pitch_deg, feedback.roll_deg)

        if self.plot_origin_monotonic is None:
            self.plot_origin_monotonic = feedback.host_received_monotonic

        relative_t = feedback.host_received_monotonic - self.plot_origin_monotonic
        self.plot_times.append(relative_t)
        self.plot_yaw.append(feedback.yaw_deg)
        self.plot_pitch.append(feedback.pitch_deg)
        self.plot_roll.append(feedback.roll_deg)

        while self.plot_times and relative_t - self.plot_times[0] > PLOT_WINDOW_S:
            self.plot_times.popleft()
            self.plot_yaw.popleft()
            self.plot_pitch.popleft()
            self.plot_roll.popleft()

        x_data = list(self.plot_times)
        self.yaw_curve.setData(x_data, list(self.plot_yaw))
        self.pitch_curve.setData(x_data, list(self.plot_pitch))
        self.roll_curve.setData(x_data, list(self.plot_roll))
        self.plot_widget.setXRange(max(0.0, relative_t - PLOT_WINDOW_S), max(PLOT_WINDOW_S, relative_t + 0.1), padding=0.0)

    def _set_connected_state(self, connected: bool) -> None:
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.baudrate_spin.setEnabled(not connected)
        self.connect_button.setText("Disconnect" if connected else "Connect")
        self.connection_status.setText("CONNECTED" if connected else "DISCONNECTED")
        for button in [
            self.handshake_button,
            self.search_button,
            self.disable_button,
            self.unlock_button,
            self.stop_button,
            self.auto_button,
        ]:
            button.setEnabled(connected)

    def _set_mode_label(self, mode_text: str) -> None:
        self.mode_value.setText(mode_text)
        palette = {
            "DISABLED": "#8E3B46",
            "SEARCH": "#185A7D",
            "AUTO_AIM": "#556B2F",
            "STANDBY": "#4C5B6A",
            "MANUAL": "#5B4B8A",
            "DISCONNECTED": "#2B3947",
        }
        color = palette.get(mode_text, "#2B3947")
        self.mode_value.setStyleSheet(
            "padding: 6px 12px; border-radius: 11px; "
            f"background: {color}; color: #F0F4F8; font-weight: 700;"
        )

    def append_log(self, text: str) -> None:
        self.log_lines.append(text)
        if self.pause_log_checkbox.isChecked():
            return
        self.log_output.setPlainText("\n".join(self.log_lines))
        scroll_bar = self.log_output.verticalScrollBar()
        scroll_bar.setValue(scroll_bar.maximum())

    def clear_plot(self) -> None:
        self.plot_times.clear()
        self.plot_yaw.clear()
        self.plot_pitch.clear()
        self.plot_roll.clear()
        self.plot_origin_monotonic = None
        self.yaw_curve.setData([], [])
        self.pitch_curve.setData([], [])
        self.roll_curve.setData([], [])

    def clear_log(self) -> None:
        self.log_lines.clear()
        self.log_output.clear()

    def export_log(self) -> None:
        target, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export Session Log",
            str(Path.cwd() / "usb_debug_log.txt"),
            "Text Files (*.txt)",
        )
        if not target:
            return
        Path(target).write_text("\n".join(self.log_lines), encoding="utf-8")
        self.statusBar().showMessage(f"Log exported to {target}", 4000)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.session.disconnect()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="USB gimbal GUI debug tool")
    parser.add_argument("--port", help="Optional serial port to preselect, e.g. COM5 or /dev/ttyACM0")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("USB Gimbal Debug Console")
    window = MainWindow(initial_port=args.port)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())




