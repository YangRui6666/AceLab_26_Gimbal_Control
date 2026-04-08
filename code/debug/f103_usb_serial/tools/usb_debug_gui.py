from __future__ import annotations

import argparse
import os
import sys
import time
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
MAX_LOG_LINES = 1200
RECONNECT_BASE_DELAY_MS = 1000
RECONNECT_MAX_DELAY_MS = 10000

pg.setConfigOptions(antialias=True, foreground="#385066", background="#FBFDFF")

QT_HORIZONTAL = getattr(getattr(QtCore.Qt, "Orientation", QtCore.Qt), "Horizontal")
QT_VERTICAL = getattr(getattr(QtCore.Qt, "Orientation", QtCore.Qt), "Vertical")
QT_FONT_MONOSPACE = getattr(getattr(QtGui.QFont, "StyleHint", QtGui.QFont), "Monospace")

MODE_META = {
    "DISCONNECTED": {
        "label": "未连接",
        "badge_bg": "#E8EDF2",
        "badge_fg": "#55687B",
        "window_bg": "#EEF3F7",
        "window_bg_2": "#F7FAFC",
    },
    "STANDBY": {
        "label": "自稳",
        "badge_bg": "#EEF7FF",
        "badge_fg": "#3A78B2",
        "window_bg": "#EEF4F8",
        "window_bg_2": "#F8FBFD",
    },
    "SEARCH": {
        "label": "搜索模式",
        "badge_bg": "#E9F8F0",
        "badge_fg": "#257A50",
        "window_bg": "#EEF7F1",
        "window_bg_2": "#FAFDFB",
    },
    "AUTO_AIM": {
        "label": "自瞄模式",
        "badge_bg": "#FFF6E4",
        "badge_fg": "#946718",
        "window_bg": "#FFF6E8",
        "window_bg_2": "#FFFDFA",
    },
    "DISABLED": {
        "label": "失能",
        "badge_bg": "#FFF0F0",
        "badge_fg": "#A13D3D",
        "window_bg": "#F8EEEE",
        "window_bg_2": "#FFFAFA",
    },
    "MANUAL": {
        "label": "手动",
        "badge_bg": "#F0EEFF",
        "badge_fg": "#5B4B8A",
        "window_bg": "#F5F3FF",
        "window_bg_2": "#FBFAFF",
    },
}


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
            edgeColor=(0.93, 0.97, 1.0, 1.0),
            color=(0.36, 0.66, 0.89, 0.70),
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
        self.setWindowTitle("USB 云台调试台")
        self.resize(1480, 960)

        self.session = UsbSession(auto_search_on_handshake=True, keepalive_enabled=True)
        self.plot_times: deque[float] = deque()
        self.plot_yaw: deque[float] = deque()
        self.plot_pitch: deque[float] = deque()
        self.plot_roll: deque[float] = deque()
        self.log_lines: deque[str] = deque(maxlen=MAX_LOG_LINES)
        self.plot_origin_monotonic: float | None = None

        self._initial_port = initial_port
        self._last_port = initial_port
        self._session_started_monotonic: float | None = None
        self._last_feedback_monotonic: float | None = None
        self._current_mode_key = "DISCONNECTED"
        self._manual_disconnect = False
        self._reconnect_attempts = 0
        self._reconnect_in_progress = False
        self._current_keepalive_text = "空闲"

        self._build_ui()
        self._apply_style("DISCONNECTED")
        self.refresh_ports(select_port=initial_port)
        self._bind_signals()
        self._apply_session_preferences()
        self._sync_reconnect_ui()

        self.event_timer = QtCore.QTimer(self)
        self.event_timer.setInterval(33)
        self.event_timer.timeout.connect(self.process_session_events)
        self.event_timer.start()

        self.status_timer = QtCore.QTimer(self)
        self.status_timer.setInterval(250)
        self.status_timer.timeout.connect(self._update_runtime_info)
        self.status_timer.start()

        self.reconnect_timer = QtCore.QTimer(self)
        self.reconnect_timer.setSingleShot(True)
        self.reconnect_timer.timeout.connect(self._attempt_serial_reconnect)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        central.setObjectName("rootWidget")
        root_layout = QtWidgets.QVBoxLayout(central)
        root_layout.setContentsMargins(18, 18, 18, 18)
        root_layout.setSpacing(14)

        header = QtWidgets.QFrame()
        header.setObjectName("headerCard")
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(18, 14, 18, 14)
        header_layout.setSpacing(14)

        title_layout = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("USB 云台调试台")
        title.setObjectName("heroTitle")
        subtitle = QtWidgets.QLabel("串口协议调试、姿态显示、实时曲线")
        subtitle.setObjectName("heroSubtitle")
        title_layout.addWidget(title)
        title_layout.addWidget(subtitle)
        header_layout.addLayout(title_layout)
        header_layout.addStretch(1)

        self.connection_status = QtWidgets.QLabel("未连接")
        self.connection_status.setObjectName("connectionBadge")
        header_layout.addWidget(self.connection_status)

        port_label = QtWidgets.QLabel("串口")
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setMinimumWidth(150)
        self.refresh_button = QtWidgets.QPushButton("刷新")
        baud_label = QtWidgets.QLabel("波特率")
        self.baudrate_spin = QtWidgets.QSpinBox()
        self.baudrate_spin.setRange(1200, 3000000)
        self.baudrate_spin.setValue(115200)
        self.connect_button = QtWidgets.QPushButton("连接")
        self.uptime_chip = QtWidgets.QLabel("连接时长：00:00")
        self.uptime_chip.setObjectName("smallChip")

        header_layout.addWidget(port_label)
        header_layout.addWidget(self.port_combo)
        header_layout.addWidget(self.refresh_button)
        header_layout.addWidget(baud_label)
        header_layout.addWidget(self.baudrate_spin)
        header_layout.addWidget(self.uptime_chip)
        header_layout.addWidget(self.connect_button)

        body_splitter = QtWidgets.QSplitter(QT_HORIZONTAL)
        body_splitter.setChildrenCollapsible(False)

        left_panel = QtWidgets.QFrame()
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(12)

        status_group = QtWidgets.QGroupBox("会话状态")
        status_grid = QtWidgets.QGridLayout(status_group)
        status_grid.setHorizontalSpacing(12)
        status_grid.setVerticalSpacing(10)
        self.mode_value = QtWidgets.QLabel("未连接")
        self.mode_value.setObjectName("modeBadge")
        self.uptime_value = QtWidgets.QLabel("00:00")
        self.feedback_age_value = QtWidgets.QLabel("--")
        self.yaw_value = QtWidgets.QLabel("0.000 deg")
        self.pitch_value = QtWidgets.QLabel("0.000 deg")
        self.roll_value = QtWidgets.QLabel("0.000 deg")
        status_grid.addWidget(QtWidgets.QLabel("当前模式"), 0, 0)
        status_grid.addWidget(self.mode_value, 0, 1)
        status_grid.addWidget(QtWidgets.QLabel("连接时长"), 1, 0)
        status_grid.addWidget(self.uptime_value, 1, 1)
        status_grid.addWidget(QtWidgets.QLabel("最近反馈"), 2, 0)
        status_grid.addWidget(self.feedback_age_value, 2, 1)
        status_grid.addWidget(QtWidgets.QLabel("Yaw"), 3, 0)
        status_grid.addWidget(self.yaw_value, 3, 1)
        status_grid.addWidget(QtWidgets.QLabel("Pitch"), 4, 0)
        status_grid.addWidget(self.pitch_value, 4, 1)
        status_grid.addWidget(QtWidgets.QLabel("Roll"), 5, 0)
        status_grid.addWidget(self.roll_value, 5, 1)

        control_group = QtWidgets.QGroupBox("控制区")
        control_layout = QtWidgets.QVBoxLayout(control_group)
        control_layout.setSpacing(10)
        command_grid = QtWidgets.QGridLayout()
        command_grid.setHorizontalSpacing(10)
        command_grid.setVerticalSpacing(10)
        self.standby_button = QtWidgets.QPushButton("自稳")
        self.standby_button.setEnabled(False)
        self.standby_button.setToolTip("待固件支持")
        self.search_button = QtWidgets.QPushButton("搜索")
        self.auto_button = QtWidgets.QPushButton("自瞄")
        self.disable_button = QtWidgets.QPushButton("失能")
        self.stop_button = QtWidgets.QPushButton("停止保活")
        command_grid.addWidget(self.standby_button, 0, 0)
        command_grid.addWidget(self.search_button, 0, 1)
        command_grid.addWidget(self.auto_button, 1, 0)
        command_grid.addWidget(self.disable_button, 1, 1)
        command_grid.addWidget(self.stop_button, 2, 0, 1, 2)
        control_layout.addLayout(command_grid)

        auto_group = QtWidgets.QGroupBox("自瞄参数")
        auto_grid = QtWidgets.QGridLayout(auto_group)
        auto_grid.setHorizontalSpacing(10)
        auto_grid.setVerticalSpacing(10)
        self.auto_yaw_spin = QtWidgets.QDoubleSpinBox()
        self.auto_yaw_spin.setRange(-180.0, 180.0)
        self.auto_yaw_spin.setDecimals(3)
        self.auto_yaw_spin.setSingleStep(0.5)
        self.auto_yaw_spin.setValue(12.5)
        self.auto_pitch_spin = QtWidgets.QDoubleSpinBox()
        self.auto_pitch_spin.setRange(-90.0, 90.0)
        self.auto_pitch_spin.setDecimals(3)
        self.auto_pitch_spin.setSingleStep(0.5)
        self.auto_pitch_spin.setValue(-2.75)
        self.search_period_spin = QtWidgets.QSpinBox()
        self.search_period_spin.setRange(10, 5000)
        self.search_period_spin.setValue(25)
        self.search_period_spin.setSuffix(" ms")
        self.auto_period_spin = QtWidgets.QSpinBox()
        self.auto_period_spin.setRange(5, 1000)
        self.auto_period_spin.setValue(4)
        self.auto_period_spin.setSuffix(" ms")
        auto_grid.addWidget(QtWidgets.QLabel("Yaw (deg)"), 0, 0)
        auto_grid.addWidget(self.auto_yaw_spin, 0, 1)
        auto_grid.addWidget(QtWidgets.QLabel("Pitch (deg)"), 1, 0)
        auto_grid.addWidget(self.auto_pitch_spin, 1, 1)
        auto_grid.addWidget(QtWidgets.QLabel("搜索周期"), 2, 0)
        auto_grid.addWidget(self.search_period_spin, 2, 1)
        auto_grid.addWidget(QtWidgets.QLabel("自瞄周期"), 3, 0)
        auto_grid.addWidget(self.auto_period_spin, 3, 1)

        debug_group = QtWidgets.QGroupBox("调试台")
        debug_layout = QtWidgets.QVBoxLayout(debug_group)
        debug_layout.setSpacing(10)
        debug_grid = QtWidgets.QGridLayout()
        debug_grid.setHorizontalSpacing(10)
        debug_grid.setVerticalSpacing(10)
        self.handshake_button = QtWidgets.QPushButton("握手")
        self.unlock_button = QtWidgets.QPushButton("解锁")
        self.reconnect_button = QtWidgets.QPushButton("立即重连")
        self.clear_log_shortcut_button = QtWidgets.QPushButton("清空日志")
        debug_grid.addWidget(self.handshake_button, 0, 0)
        debug_grid.addWidget(self.unlock_button, 0, 1)
        debug_grid.addWidget(self.reconnect_button, 1, 0)
        debug_grid.addWidget(self.clear_log_shortcut_button, 1, 1)
        debug_layout.addLayout(debug_grid)

        self.keepalive_checkbox = QtWidgets.QCheckBox("启用周期保活")
        self.keepalive_checkbox.setChecked(True)
        self.serial_reconnect_checkbox = QtWidgets.QCheckBox("串口自动重连")
        self.serial_reconnect_checkbox.setChecked(True)
        self.gimbal_reconnect_checkbox = QtWidgets.QCheckBox("云台自动重连")
        self.gimbal_reconnect_checkbox.setChecked(True)
        debug_layout.addWidget(self.keepalive_checkbox)
        debug_layout.addWidget(self.serial_reconnect_checkbox)
        debug_layout.addWidget(self.gimbal_reconnect_checkbox)

        left_layout.addWidget(status_group)
        left_layout.addWidget(control_group)
        left_layout.addWidget(auto_group)
        left_layout.addWidget(debug_group)
        left_layout.addStretch(1)

        right_splitter = QtWidgets.QSplitter(QT_VERTICAL)
        right_splitter.setChildrenCollapsible(False)

        visual_card = QtWidgets.QFrame()
        visual_layout = QtWidgets.QVBoxLayout(visual_card)
        visual_layout.setContentsMargins(12, 12, 12, 12)
        visual_layout.setSpacing(10)
        visual_title = QtWidgets.QLabel("3D 姿态")
        visual_title.setObjectName("sectionTitle")
        self.attitude_note_label = QtWidgets.QLabel("当前模式：未连接 / 最近反馈 --")
        self.attitude_note_label.setObjectName("heroSubtitle")
        self.cube_view = CubeView()
        visual_layout.addWidget(visual_title)
        visual_layout.addWidget(self.attitude_note_label)
        visual_layout.addWidget(self.cube_view, 1)

        preview_grid = QtWidgets.QGridLayout()
        preview_grid.setHorizontalSpacing(10)
        preview_grid.setVerticalSpacing(10)
        self.last_tx_preview = QtWidgets.QPlainTextEdit()
        self.last_tx_preview.setReadOnly(True)
        self.last_tx_preview.setFixedHeight(78)
        self.last_rx_preview = QtWidgets.QPlainTextEdit()
        self.last_rx_preview.setReadOnly(True)
        self.last_rx_preview.setFixedHeight(78)
        self.keepalive_status_label = QtWidgets.QLabel("搜索保活")
        self.serial_reconnect_status_label = QtWidgets.QLabel("开启")
        self.gimbal_reconnect_status_label = QtWidgets.QLabel("开启")
        preview_grid.addWidget(QtWidgets.QLabel("最近发送"), 0, 0)
        preview_grid.addWidget(QtWidgets.QLabel("最近接收"), 0, 1)
        preview_grid.addWidget(self.last_tx_preview, 1, 0)
        preview_grid.addWidget(self.last_rx_preview, 1, 1)
        preview_grid.addWidget(QtWidgets.QLabel("保活状态"), 2, 0)
        preview_grid.addWidget(self.keepalive_status_label, 2, 1)
        preview_grid.addWidget(QtWidgets.QLabel("串口自动重连"), 3, 0)
        preview_grid.addWidget(self.serial_reconnect_status_label, 3, 1)
        preview_grid.addWidget(QtWidgets.QLabel("云台自动重连"), 4, 0)
        preview_grid.addWidget(self.gimbal_reconnect_status_label, 4, 1)
        visual_layout.addLayout(preview_grid)

        plot_card = QtWidgets.QFrame()
        plot_layout = QtWidgets.QVBoxLayout(plot_card)
        plot_layout.setContentsMargins(12, 12, 12, 12)
        plot_layout.setSpacing(10)
        plot_header = QtWidgets.QHBoxLayout()
        plot_title = QtWidgets.QLabel("实时曲线")
        plot_title.setObjectName("sectionTitle")
        plot_header.addWidget(plot_title)
        plot_header.addStretch(1)
        self.clear_plot_button = QtWidgets.QPushButton("清空曲线")
        plot_header.addWidget(self.clear_plot_button)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.setLabel("left", "Angle", units="deg")
        self.plot_widget.setLabel("bottom", "Time", units="s")
        self.plot_widget.addLegend(offset=(16, 8))
        self.yaw_curve = self.plot_widget.plot(name="yaw", pen=pg.mkPen("#59AEE7", width=2))
        self.pitch_curve = self.plot_widget.plot(name="pitch", pen=pg.mkPen("#D69A2D", width=2))
        self.roll_curve = self.plot_widget.plot(name="roll", pen=pg.mkPen("#2F8B59", width=2))
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
        log_title = QtWidgets.QLabel("会话日志")
        log_title.setObjectName("sectionTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)
        self.pause_log_checkbox = QtWidgets.QCheckBox("暂停")
        self.clear_log_button = QtWidgets.QPushButton("清空")
        self.export_log_button = QtWidgets.QPushButton("导出")
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
        self.statusBar().showMessage("就绪")
        self._set_connected_state(False)
        self._set_mode_label("DISCONNECTED")
        self.last_tx_preview.setPlainText("-")
        self.last_rx_preview.setPlainText("-")

    def _apply_style(self, mode_key: str) -> None:
        meta = MODE_META.get(mode_key, MODE_META["DISCONNECTED"])
        self.setStyleSheet(
            f"""
            QWidget#rootWidget {{
                background: {meta['window_bg_2']};
                color: #223140;
                font-family: Segoe UI;
                font-size: 13px;
            }}
            QWidget#rootWidget > QFrame, QWidget#rootWidget > QSplitter {{
                background: transparent;
            }}
            QFrame, QGroupBox {{
                background: rgba(255, 255, 255, 0.90);
                border: 1px solid #D9E3EC;
                border-radius: 14px;
            }}
            QFrame#headerCard {{
                background: rgba(255, 255, 255, 0.97);
            }}
            QGroupBox {{
                margin-top: 12px;
                padding-top: 12px;
                font-weight: 600;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 6px;
                color: #6F8294;
            }}
            QLabel#heroTitle {{
                font-size: 28px;
                font-weight: 700;
                color: #203040;
            }}
            QLabel#heroSubtitle {{
                color: #708293;
                font-size: 13px;
            }}
            QLabel#sectionTitle {{
                font-size: 16px;
                font-weight: 700;
                color: #203040;
            }}
            QLabel#connectionBadge, QLabel#modeBadge, QLabel#smallChip {{
                padding: 6px 12px;
                border-radius: 11px;
                border: 1px solid #C8D4DF;
                background: #FFFFFF;
                color: #223140;
                font-weight: 700;
            }}
            QPushButton {{
                background: #F8FBFE;
                border: 1px solid #C8D4DF;
                border-radius: 10px;
                padding: 8px 14px;
                font-weight: 600;
                color: #223140;
            }}
            QPushButton:hover {{
                background: #EEF5FB;
            }}
            QPushButton:disabled {{
                color: #8A9AAB;
                background: #F4F7FA;
                border-color: #D7E1EA;
            }}
            QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit {{
                background: #FBFDFF;
                border: 1px solid #DFE7EF;
                border-radius: 10px;
                padding: 6px 8px;
                color: #223140;
            }}
            QCheckBox {{
                spacing: 8px;
                color: #223140;
                background: transparent;
            }}
            QSplitter::handle {{
                background: transparent;
            }}
            QStatusBar {{
                background: {meta['window_bg']};
                color: #4A6075;
            }}
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
        self.reconnect_button.clicked.connect(self.reconnect_now)
        self.clear_log_shortcut_button.clicked.connect(self.clear_log)
        self.clear_plot_button.clicked.connect(self.clear_plot)
        self.clear_log_button.clicked.connect(self.clear_log)
        self.export_log_button.clicked.connect(self.export_log)
        self.keepalive_checkbox.toggled.connect(self._apply_session_preferences)
        self.search_period_spin.valueChanged.connect(self._apply_session_preferences)
        self.auto_period_spin.valueChanged.connect(self._apply_session_preferences)
        self.serial_reconnect_checkbox.toggled.connect(self._serial_reconnect_toggled)
        self.gimbal_reconnect_checkbox.toggled.connect(self._gimbal_reconnect_toggled)

    def refresh_ports(self, select_port: str | None = None) -> None:
        current = select_port or self.port_combo.currentText() or self._last_port or ""
        ports = [port.device for port in list_ports.comports()]
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current and current in ports:
            self.port_combo.setCurrentText(current)
        elif self._initial_port and self._initial_port in ports:
            self.port_combo.setCurrentText(self._initial_port)
        self.port_combo.blockSignals(False)
        self.statusBar().showMessage(f"检测到 {len(ports)} 个串口", 3000)

    def toggle_connection(self) -> None:
        if self.session.connected:
            self._manual_disconnect = True
            self.reconnect_timer.stop()
            self.session.disconnect()
            return

        port = self.port_combo.currentText().strip()
        if not port:
            QtWidgets.QMessageBox.warning(self, "未选择串口", "请先选择串口。")
            return

        self._manual_disconnect = False
        self._reconnect_in_progress = False
        try:
            self.session.connect(port, baudrate=self.baudrate_spin.value())
            self._last_port = port
            self._apply_session_preferences()
        except SerialException as exc:
            self.append_log(f"串口打开失败: {exc}")
            QtWidgets.QMessageBox.critical(self, "连接失败", str(exc))
            self._set_connected_state(False)

    def send_handshake(self) -> None:
        if self.session.connected:
            self.session.send_handshake()

    def send_search(self) -> None:
        if not self.session.connected:
            return
        self.append_log("组合命令：解锁并进入搜索")
        self.session.unlock()
        self.session.start_search(periodic=self.keepalive_checkbox.isChecked())

    def send_auto(self) -> None:
        if not self.session.connected:
            return
        self.append_log("组合命令：解锁并进入自瞄")
        self.session.unlock()
        self.session.start_auto(
            self.auto_yaw_spin.value(),
            self.auto_pitch_spin.value(),
            periodic=self.keepalive_checkbox.isChecked(),
        )

    def send_disable(self) -> None:
        if self.session.connected:
            self.append_log("控制区：发送失能")
            self.session.disable()

    def send_unlock(self) -> None:
        if self.session.connected:
            self.session.unlock()

    def stop_keepalive(self) -> None:
        if self.session.connected:
            self.session.stop_keepalive()

    def reconnect_now(self) -> None:
        self._manual_disconnect = False
        if self.session.connected:
            self.append_log("调试台：立即重连")
            self._schedule_serial_reconnect("手动触发重连", delay_ms=200)
            self.session.disconnect()
            return
        self._schedule_serial_reconnect("手动触发重连", delay_ms=200)

    def _apply_session_preferences(self) -> None:
        self.session.set_keepalive_enabled(self.keepalive_checkbox.isChecked())
        self.session.set_periods(
            self.search_period_spin.value() / 1000.0,
            self.auto_period_spin.value() / 1000.0,
        )
        self._current_keepalive_text = "已启用" if self.keepalive_checkbox.isChecked() else "已停用"
        self.keepalive_status_label.setText(self._current_keepalive_text)

    def process_session_events(self) -> None:
        for event in self.session.poll_events(limit=400):
            self._handle_event(event)
        self._update_runtime_info()

    def _handle_event(self, event: SessionEvent) -> None:
        if event.kind != "feedback":
            self.append_log(event.text)

        if event.kind == "tx":
            self.last_tx_preview.setPlainText(event.text)
            return

        if event.kind in {"handshake", "rx", "disabled", "boot", "state"}:
            self.last_rx_preview.setPlainText(event.text)

        if event.kind == "connected":
            self._set_connected_state(True)
            self._session_started_monotonic = time.monotonic()
            self._reconnect_attempts = 0
            self._last_port = event.port or self._last_port
            self.connection_status.setText("已连接")
            self.statusBar().showMessage(f"已连接到 {event.port}", 3000)
            return

        if event.kind == "disconnected":
            self._set_connected_state(False)
            self._last_feedback_monotonic = None
            self.feedback_age_value.setText("--")
            if self._manual_disconnect:
                self._session_started_monotonic = None
                self._set_mode_label("DISCONNECTED")
                self.connection_status.setText("未连接")
                self.statusBar().showMessage("已断开连接", 3000)
                self._manual_disconnect = False
                return

            if self.serial_reconnect_checkbox.isChecked() and self._last_port:
                self._schedule_serial_reconnect("串口断开，准备自动重连")
                self.connection_status.setText("重连中")
            else:
                self._session_started_monotonic = None
                self._set_mode_label("DISCONNECTED")
                self.connection_status.setText("未连接")
                self.statusBar().showMessage("连接已断开", 3000)
            return

        if event.kind == "handshake":
            if self._reconnect_in_progress and self.gimbal_reconnect_checkbox.isChecked():
                self.append_log("云台自动重连：当前固件暂不支持独立自稳命令，等待后续固件接入")
            self._reconnect_in_progress = False
            return

        if event.kind == "feedback" and event.feedback is not None:
            self._update_feedback(event.feedback)
            return

        if event.kind == "disabled":
            self._set_mode_label("DISABLED")
            self.statusBar().showMessage("云台进入失能状态", 5000)
            return

        if event.kind == "error":
            self.statusBar().showMessage(event.text, 5000)

    def _update_feedback(self, feedback: GimbalFeedback) -> None:
        self._last_feedback_monotonic = feedback.host_received_monotonic
        self.yaw_value.setText(f"{feedback.yaw_deg:.3f} deg")
        self.pitch_value.setText(f"{feedback.pitch_deg:.3f} deg")
        self.roll_value.setText(f"{feedback.roll_deg:.3f} deg")
        self._set_mode_label(mode_name(feedback.mode))
        self.cube_view.set_rotation(feedback.yaw_deg, feedback.pitch_deg, feedback.roll_deg)
        self.attitude_note_label.setText(
            f"当前模式：{self.mode_value.text()} / 最近反馈 {self._feedback_age_text()}"
        )

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
        self.plot_widget.setXRange(
            max(0.0, relative_t - PLOT_WINDOW_S),
            max(PLOT_WINDOW_S, relative_t + 0.1),
            padding=0.0,
        )
        self.last_rx_preview.setPlainText(
            "RX 0x03 云台反馈\n"
            f"yaw={feedback.yaw_deg:.3f} pitch={feedback.pitch_deg:.3f} roll={feedback.roll_deg:.3f}\n"
            f"mode={mode_name(feedback.mode)}"
        )

    def _set_connected_state(self, connected: bool) -> None:
        self.port_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)
        self.baudrate_spin.setEnabled(not connected)
        self.connect_button.setText("断开" if connected else "连接")
        self.reconnect_button.setEnabled(True)
        for button in [
            self.handshake_button,
            self.search_button,
            self.disable_button,
            self.unlock_button,
            self.stop_button,
            self.auto_button,
        ]:
            button.setEnabled(connected)

    def _set_mode_label(self, mode_key: str) -> None:
        meta = MODE_META.get(mode_key, MODE_META["DISCONNECTED"])
        self.mode_value.setText(meta["label"])
        self.mode_value.setStyleSheet(
            "padding: 6px 12px; border-radius: 11px; font-weight: 700;"
            f"background: {meta['badge_bg']}; color: {meta['badge_fg']};"
        )
        self._current_mode_key = mode_key
        self._apply_style(mode_key)
        if mode_key == "SEARCH":
            self._current_keepalive_text = "搜索保活"
        elif mode_key == "AUTO_AIM":
            self._current_keepalive_text = "自瞄保活"
        elif mode_key == "DISABLED":
            self._current_keepalive_text = "保活停止"
        elif mode_key == "STANDBY":
            self._current_keepalive_text = "等待启动"
        elif mode_key == "DISCONNECTED":
            self._current_keepalive_text = "空闲"

    def _feedback_age_text(self) -> str:
        if self._last_feedback_monotonic is None:
            return "--"
        age_ms = int((time.monotonic() - self._last_feedback_monotonic) * 1000.0)
        return f"{age_ms} ms 前"

    def _update_runtime_info(self) -> None:
        if self._session_started_monotonic is None:
            uptime = "00:00"
        else:
            elapsed = int(time.monotonic() - self._session_started_monotonic)
            uptime = f"{elapsed // 60:02d}:{elapsed % 60:02d}"
        self.uptime_chip.setText(f"连接时长：{uptime}")
        self.uptime_value.setText(uptime)
        self.feedback_age_value.setText(self._feedback_age_text())
        self.keepalive_status_label.setText(self._current_keepalive_text)
        self.serial_reconnect_status_label.setText("开启" if self.serial_reconnect_checkbox.isChecked() else "关闭")
        self.gimbal_reconnect_status_label.setText("开启" if self.gimbal_reconnect_checkbox.isChecked() else "关闭")
        self.attitude_note_label.setText(
            f"当前模式：{self.mode_value.text()} / 最近反馈 {self._feedback_age_text()}"
        )

    def _serial_reconnect_toggled(self, checked: bool) -> None:
        self._sync_reconnect_ui()
        self.append_log(f"串口自动重连{'开启' if checked else '关闭'}")
        if not checked:
            self.reconnect_timer.stop()

    def _gimbal_reconnect_toggled(self, checked: bool) -> None:
        self._sync_reconnect_ui()
        self.append_log(f"云台自动重连{'开启' if checked else '关闭'}")

    def _sync_reconnect_ui(self) -> None:
        self.serial_reconnect_status_label.setText("开启" if self.serial_reconnect_checkbox.isChecked() else "关闭")
        self.gimbal_reconnect_status_label.setText("开启" if self.gimbal_reconnect_checkbox.isChecked() else "关闭")

    def _schedule_serial_reconnect(self, reason: str, delay_ms: int | None = None) -> None:
        if not self._last_port:
            return
        if delay_ms is None:
            self._reconnect_attempts += 1
            delay_ms = min(RECONNECT_BASE_DELAY_MS * (2 ** max(0, self._reconnect_attempts - 1)), RECONNECT_MAX_DELAY_MS)
        self._reconnect_in_progress = True
        self.connection_status.setText("重连中")
        self.statusBar().showMessage(reason, 3000)
        self.reconnect_timer.start(delay_ms)

    def _attempt_serial_reconnect(self) -> None:
        if self.session.connected:
            return
        port = self._last_port or self.port_combo.currentText().strip()
        if not port:
            self._reconnect_in_progress = False
            return
        try:
            self.session.connect(port, baudrate=self.baudrate_spin.value())
            self._apply_session_preferences()
            self._last_port = port
        except SerialException as exc:
            self.append_log(f"串口自动重连失败: {exc}")
            self._schedule_serial_reconnect("串口自动重连失败，稍后重试")

    def append_log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {text}"
        self.log_lines.append(line)
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
            "导出会话日志",
            str(Path.cwd() / "usb_debug_log.txt"),
            "Text Files (*.txt)",
        )
        if not target:
            return
        Path(target).write_text("\n".join(self.log_lines), encoding="utf-8")
        self.statusBar().showMessage(f"日志已导出到 {target}", 4000)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.reconnect_timer.stop()
        self.session.disconnect()
        super().closeEvent(event)


def main() -> int:
    parser = argparse.ArgumentParser(description="USB 云台调试台")
    parser.add_argument("--port", help="可选串口，例如 COM5 或 /dev/ttyACM0")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("USB 云台调试台")
    window = MainWindow(initial_port=args.port)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
