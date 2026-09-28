#!/usr/bin/python3

# Copyright 2024 Laboratoire des signaux et systèmes
#
# This program is free software: you can redistribute it and/or 
# modify it under the terms of the GNU General Public License as 
# published by the Free Software Foundation, either version 3 of 
# the License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful, 
# but WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program. If not, see <https://www.gnu.org/licenses/>. 
#
# Author: Aarsh Thakker <aarsh.thakker@centralesupelec.fr>

PACKAGE='natnet_ros2'
from ament_index_python.packages import get_package_share_directory
PKG_PATH = get_package_share_directory(PACKAGE)

import sys
import rclpy
from rclpy.node import Node
from ros2param.api import load_parameter_file
from natnet_ros2_py.node_module import HelperNode

import subprocess
import os
import re
import html
from collections import deque
import shutil
import signal
import threading
import time
import traceback
import yaml
from pathlib import Path

from PyQt5 import QtWidgets, uic
import sys

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, QTimer, pyqtSignal, Qt

class WorkerThread(QThread):
  started_signal = pyqtSignal(int, int)
  log_line_signal = pyqtSignal(str, str)
  finished_signal = pyqtSignal(int)
  def __init__(self, cmd, env=None, cwd=None) -> None:
    super().__init__()

    self.cmd = cmd
    self.env = env
    self.cwd = cwd
    self.proc = None

  def _stream(self, stream, level):
    try:
      for line in iter(stream.readline, ''):
        log_line = line.rstrip('\n')
        if log_line:
          self.log_line_signal.emit(level, log_line)
    finally:
      stream.close()

  def run(self):
    try:
      self.proc = subprocess.Popen(
        self.cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        env=self.env,
        cwd=self.cwd,
        preexec_fn=os.setsid,
      )
      self.started_signal.emit(self.proc.pid, os.getpgid(self.proc.pid))
      stdout_thread = threading.Thread(target=self._stream, args=(self.proc.stdout, 'INFO'), daemon=True)
      stderr_thread = threading.Thread(target=self._stream, args=(self.proc.stderr, 'ERROR'), daemon=True)
      stdout_thread.start()
      stderr_thread.start()
      rc = self.proc.wait()
      stdout_thread.join(timeout=1.0)
      stderr_thread.join(timeout=1.0)
      self.finished_signal.emit(rc)
    except Exception as e:
      self.log_line_signal.emit('ERROR', f'Failed to run launch command: {e}')
      self.log_line_signal.emit('ERROR', traceback.format_exc().strip())
      self.finished_signal.emit(-1)

class MarkerRefreshThread(QThread):
  result_signal = pyqtSignal(object)
  error_signal = pyqtSignal(str)

  def __init__(self, node):
    super().__init__()
    self.node = node

  def run(self):
    try:
      self.result_signal.emit(self.node.request_markerposes(timeout_sec=5.0, max_attempts=3))
    except RuntimeError as e:
      self.error_signal.emit(str(e))
    except Exception:
      self.error_signal.emit(traceback.format_exc().strip())

class PyQt5Widget(QtWidgets.QMainWindow):
  def __init__(self,node:Node):
    super(PyQt5Widget, self).__init__()
    self.node = node
    ui_file = os.path.join(PKG_PATH, 'ui', 'helper.ui')
    uic.loadUi(ui_file, self)
    self.setWindowIcon(QtGui.QIcon(os.path.join(PKG_PATH,'ui','logo.png')))
    self._theme_dark = False
    self._log_entries = deque(maxlen=1000)
    self._build_interface()

    self.pwd = os.getcwd()
    self.workspace_root = None
    self.workspace_setup_script = None
    self.ros2_executable = None
    self.msg_types = ["PoseStamped","PointStamped"]
    self.im_msg_type = self.msg_types[0]
    self.config_file = None
    if os.path.exists(os.path.join(PKG_PATH, 'config','conf_autogen.yaml')):
      self.config_file = os.path.join(PKG_PATH, 'config','conf_autogen.yaml')
    self.name = 'natnet_ros2'
    self.pub_params = {"pub_individual_marker":False,
                      "pub_rigid_body": False,
                      "pub_rigid_body_marker": False,
                      "pub_pointcloud": False,
                      "pub_rigid_body_wmc": False,
                      "individual_marker_msg_type": "PoseStamped",
                      }
    self.log_params = {"log_internals": False,
                      "log_frames": False,
                      "log_latencies": False,
                      }
    self.conn_params = {"serverIP": None,
                        "clientIP": None,
                        "serverType": None,
                        "multicastAddress": None,
                        "serverCommandPort": None,
                        "serverDataPort": None,
                        "globalFrame": None,
                        }
    self.natnet_params = {"{name}".format(name=self.name):None}
    self.error_pass = True
    self.num_of_markers=0
    self.x_position=None
    self.y_position=None
    self.z_position=None
    self.id = 0
    self.launch_proc = None
    self.launch_pgid = None
    self.is_running = False
    self.start_node_thread = None
    self.client_ip_raw = None
    self.server_ip_raw = None
    self.marker_service_available = False
    self._last_marker_service_state = None
    self.marker_refresh_thread = None
    self.marker_service_check_timer = QTimer(self)
    self.marker_service_check_timer.setInterval(2000)
    self.marker_service_check_timer.timeout.connect(self._refresh_marker_service_status)
    
    self.Log('info', 'Set connection details, choose publishers, then press Start. Stop affects only this window’s launch.')
    self.Log('info', 'Enter client and server IPs manually if no network interface is detected.')
    try:
      IP_data = subprocess.check_output(
        ['lshw', '-c', 'network'], stderr=subprocess.DEVNULL, timeout=3).decode('utf-8')
    except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
      IP_data = ''
      self.Log('warn', f' Network detection unavailable ({e}). Enter client and server IPs manually.')
    IP_data=re.sub(r"[^a-zA-Z0-9. ]", "", IP_data)
    IP_data=IP_data.split('network')
    IP_data.pop(0)
    self.IP_LIST = []

    for data in IP_data:
      try:
        detected_ips = re.findall(r"(?:\d{1,3}\.){3}\d{1,3}", data)
        if len(detected_ips) == 0:
          continue
        detected_ip = detected_ips[0]
        self.Log('info', 'Found the connection on IP ' + self._mask_ip_for_gui(detected_ip))
        self.IP_LIST.append(detected_ip)
      except Exception as e:
        self._log_exception('Failed to parse network interface details', e)

    del IP_data
    if len(self.IP_LIST)>0:
      self.client_ip_spin.setEnabled(True)
      self.client_ip_spin.setMinimum(1)
      default_client_ip = str(self.IP_LIST[self.client_ip_spin.value()-1])
      self.client_ip_raw = default_client_ip
      self.textClientIP.setText(self._mask_ip_for_gui(default_client_ip))
      self.textServerIP.setText(self._masked_server_hint_from_client(default_client_ip))
      self.conn_params["clientIP"] = default_client_ip
    self.client_ip_spin.setMaximum(len(self.IP_LIST))

    self.pub_rbm.setEnabled(False)
    self.domain_id_spin.valueChanged.connect(self.select_network)
    self.client_ip_spin.valueChanged.connect(self.spin_clientIP)
    self.msg_type_spin.valueChanged.connect(self.spin_msg_type)
    self.textClientIP.editingFinished.connect(self._apply_client_ip_visual_mask)
    self.textServerIP.editingFinished.connect(self._apply_server_ip_visual_mask)
    self.multicast_radio.clicked.connect(self.clicked_multicast)
    self.unicast_radio.clicked.connect(self.clicked_unicast)
    self.start_node.clicked.connect(self.start)
    self.stop_node.clicked.connect(self.stop)
    self.log_frames.clicked.connect(self.log_frames_setting)
    self.log_internal.clicked.connect(self.log_internal_setting)
    self.log_latencies.clicked.connect(self.log_latencies_setting)
    self.pub_im.clicked.connect(self.pub_im_setting)
    self.pub_rb.clicked.connect(self.pub_rb_setting)
    self.pub_rbm.clicked.connect(self.pub_rbm_setting)
    self.pub_rbwmc.clicked.connect(self.pub_rbwmc_setting)
    self.pub_pc.clicked.connect(self.pub_pc_setting)
    self.push_refresh.clicked.connect(self.call_MarkerPoses_srv)
    self.push_ok.clicked.connect(self.yaml_dump)

    self.push_refresh.setEnabled(False)
    self.push_refresh.setToolTip('Marker service unavailable. Start marker_poses_server to enable refresh.')
    self._set_launch_status('Stopped', 'muted')
    self._refresh_marker_service_status()
    self.marker_service_check_timer.start()

  def _build_interface(self):
    self.setWindowTitle('NatNet Control Center')
    self.setCentralWidget(self.verticalLayoutWidget)
    self.setMinimumSize(760, 560)
    self.resize(1020, 740)
    self.tabWidget.setTabText(0, 'Connection & Control')
    self.tabWidget.setTabText(1, 'Marker Naming')
    self.tabWidget.tabBar().setExpanding(False)
    self.tabWidget.tabBar().setUsesScrollButtons(True)
    self.tabWidget.tabBar().setElideMode(Qt.ElideNone)
    self._layout_settings_groups()

    global_bar = QtWidgets.QHBoxLayout()
    global_bar.setContentsMargins(18, 6, 18, 0)
    brand = QtWidgets.QLabel('NatNet')
    brand.setObjectName('brandLabel')
    global_bar.addWidget(brand)
    global_bar.addStretch()
    self.theme_toggle = QtWidgets.QPushButton('Dark mode')
    self.theme_toggle.setObjectName('themeToggle')
    self.theme_toggle.setCheckable(True)
    self.theme_toggle.setToolTip('Switch between light and dark appearance.')
    self.theme_toggle.toggled.connect(self._apply_theme)
    global_bar.addWidget(self.theme_toggle)
    self.verticalLayoutWidget.layout().insertLayout(0, global_bar)

    control_layout = QtWidgets.QVBoxLayout(self.control_tab)
    control_layout.setContentsMargins(18, 16, 18, 16)
    control_layout.setSpacing(14)
    header = QtWidgets.QHBoxLayout()
    heading = QtWidgets.QVBoxLayout()
    title = QtWidgets.QLabel('NatNet control')
    title.setObjectName('pageTitle')
    subtitle = QtWidgets.QLabel('Connect to motion capture, choose output, and monitor launch activity.')
    subtitle.setObjectName('pageSubtitle')
    subtitle.setWordWrap(True)
    heading.addWidget(title)
    heading.addWidget(subtitle)
    header.addLayout(heading)
    header.addStretch()
    self.launch_status_label = QtWidgets.QLabel()
    self.launch_status_label.setAlignment(Qt.AlignCenter)
    self.launch_status_label.setMinimumWidth(110)
    header.addWidget(self.launch_status_label)
    control_layout.addLayout(header)

    body = QtWidgets.QHBoxLayout()
    body.setSpacing(16)
    settings_column = QtWidgets.QWidget()
    settings_column.setMinimumWidth(330)
    settings_column_layout = QtWidgets.QVBoxLayout(settings_column)
    settings_column_layout.setContentsMargins(0, 0, 0, 0)
    settings_column_layout.setSpacing(10)
    settings_scroll = QtWidgets.QScrollArea()
    settings_scroll.setWidgetResizable(True)
    settings_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    settings_panel = QtWidgets.QWidget()
    settings_panel.setObjectName('settings_panel')
    settings_layout = QtWidgets.QVBoxLayout(settings_panel)
    settings_layout.setContentsMargins(2, 2, 10, 4)
    settings_layout.setSpacing(12)
    for group in (self.parameters, self.connection_settings, self.publisher_setting):
      settings_layout.addWidget(group)
    settings_layout.addStretch()
    settings_scroll.setWidget(settings_panel)
    settings_scroll_row = QtWidgets.QHBoxLayout()
    settings_scroll_row.setSpacing(7)
    settings_scroll_row.addWidget(settings_scroll, 1)
    settings_scroll_row.addWidget(self._scroll_controls(settings_scroll))
    settings_column_layout.addLayout(settings_scroll_row, 1)
    actions = QtWidgets.QHBoxLayout()
    actions.addStretch()
    actions.addWidget(self.stop_node)
    actions.addWidget(self.start_node)
    settings_column_layout.addLayout(actions)
    body.addWidget(settings_column, 5)

    activity = QtWidgets.QVBoxLayout()
    activity.setSpacing(12)
    for group in (self.ros_network, self.logging_settings):
      activity.addWidget(group)
    log_heading = QtWidgets.QHBoxLayout()
    log_title = QtWidgets.QLabel('Activity')
    log_title.setObjectName('sectionTitle')
    clear_logs = QtWidgets.QPushButton('Clear')
    clear_logs.setToolTip('Clear visible activity messages.')
    clear_logs.clicked.connect(self._clear_logs)
    log_heading.addWidget(log_title)
    log_heading.addStretch()
    log_heading.addWidget(clear_logs)
    activity.addLayout(log_heading)
    self.outputBox.setParent(self.control_tab)
    self.outputBox.setMinimumWidth(300)
    self.outputBox.document().setMaximumBlockCount(1000)
    activity.addWidget(self.outputBox, 1)
    self.outputBox.show()
    self.outputBox_scroll.hide()
    body.addLayout(activity, 6)
    control_layout.addLayout(body, 1)

    marker_layout = QtWidgets.QVBoxLayout(self.marker_name_tab)
    marker_layout.setContentsMargins(18, 16, 18, 16)
    marker_layout.setSpacing(12)
    marker_title = QtWidgets.QLabel('Name individual markers')
    marker_title.setObjectName('pageTitle')
    marker_hint = QtWidgets.QLabel('Refresh positions, enter names for markers you need, then save configuration.')
    marker_hint.setObjectName('pageSubtitle')
    marker_hint.setWordWrap(True)
    marker_layout.addWidget(marker_title)
    marker_layout.addWidget(marker_hint)

    marker_status = QtWidgets.QHBoxLayout()
    self.marker_service_label = QtWidgets.QLabel('Checking service…')
    self.marker_count_label = QtWidgets.QLabel('0 markers')
    self.marker_count_label.setObjectName('markerCount')
    marker_status.addWidget(self.marker_service_label)
    marker_status.addWidget(self.marker_count_label)
    marker_status.addStretch()
    marker_layout.addLayout(marker_status)

    marker_actions = QtWidgets.QHBoxLayout()
    marker_actions.setSpacing(10)
    marker_actions.addWidget(QtWidgets.QLabel('Message type'))
    self.textMsgType.setFixedWidth(120)
    marker_actions.addWidget(self.textMsgType)
    marker_actions.addWidget(self._spin_controls(self.msg_type_spin))
    marker_actions.addStretch()
    marker_actions.addWidget(self.push_refresh)
    self.push_refresh.setText('Refresh markers')
    marker_layout.addLayout(marker_actions)

    marker_content = QtWidgets.QWidget()
    marker_content.setObjectName('marker_content')
    marker_rows = QtWidgets.QGridLayout(marker_content)
    marker_rows.setContentsMargins(2, 2, 2, 2)
    marker_rows.setHorizontalSpacing(8)
    marker_rows.setVerticalSpacing(5)
    self.marker_value_labels = {}
    for column in range(2):
      headings = QtWidgets.QWidget()
      heading_row = QtWidgets.QHBoxLayout(headings)
      heading_row.setContentsMargins(6, 0, 6, 0)
      heading_row.setSpacing(4)
      for title, width in (('#', 22), ('X (m)', 56), ('Y (m)', 56), ('Z (m)', 56)):
        heading = QtWidgets.QLabel(title)
        heading.setObjectName('markerHeading')
        heading.setAlignment(Qt.AlignCenter)
        heading.setMinimumWidth(width)
        if title == '#':
          heading.setFixedWidth(width)
        heading_row.addWidget(heading, 0 if title == '#' else 1)
      name_heading = QtWidgets.QLabel('Name')
      name_heading.setObjectName('markerHeading')
      heading_row.addWidget(name_heading, 2)
      marker_rows.addWidget(headings, 0, column)
    for index in range(1, 41):
      marker_box = getattr(self, f'markerBox_{index}')
      marker_box.setProperty('markerRow', True)
      marker_box.setTitle('')
      marker_box.setEnabled(True)
      marker_box.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
      row = QtWidgets.QHBoxLayout(marker_box)
      row.setContentsMargins(6, 4, 6, 4)
      row.setSpacing(4)
      serial = getattr(self, f'srn_{index}')
      serial.setText(str(index))
      serial.setObjectName('markerSerial')
      serial.setFixedWidth(22)
      serial.setEnabled(True)
      row.addWidget(serial)
      for axis in ('X', 'Y', 'Z'):
        lcd = getattr(self, f'{axis}_{index}')
        lcd.hide()
        value_label = QtWidgets.QLabel('—')
        value_label.setObjectName('markerValue')
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setMinimumWidth(56)
        value_label.setFixedHeight(24)
        row.addWidget(value_label, 1)
        self.marker_value_labels[index, axis] = value_label
      name = getattr(self, f'name_{index}')
      name.setProperty('markerName', True)
      name.setPlaceholderText('Name')
      name.setMinimumWidth(70)
      name.setFixedHeight(24)
      name.setEnabled(False)
      row.addWidget(name, 2)
      marker_box.setMinimumHeight(34)
      marker_rows.addWidget(marker_box, (index - 1) % 20 + 1, (index - 1) // 20)
    marker_rows.setRowStretch(21, 1)
    self.marker_scroll = QtWidgets.QScrollArea()
    self.marker_scroll.setWidgetResizable(True)
    self.marker_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
    self.marker_scroll.setWidget(marker_content)
    marker_scroll_row = QtWidgets.QHBoxLayout()
    marker_scroll_row.setSpacing(7)
    marker_scroll_row.addWidget(self.marker_scroll, 1)
    marker_scroll_row.addWidget(self._scroll_controls(self.marker_scroll))
    marker_layout.addLayout(marker_scroll_row, 1)

    self.push_ok.setParent(self.marker_name_tab)
    self.scrollArea.hide()
    self.verticalLayoutWidget_2.hide()
    self.groupBox_Fix.hide()
    self.groupBox_Fix_2.hide()
    save_row = QtWidgets.QHBoxLayout()
    save_row.addStretch()
    self.push_ok.setText('Save marker names')
    save_row.addWidget(self.push_ok)
    self.push_ok.show()
    self.push_ok.setEnabled(False)
    marker_layout.addLayout(save_row)
    self._apply_theme(False)

  def _scroll_controls(self, area):
    controls = QtWidgets.QWidget()
    controls.setObjectName('scrollControls')
    layout = QtWidgets.QVBoxLayout(controls)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(7)
    for label, direction, tooltip in (('▲', -1, 'Scroll up'), ('▼', 1, 'Scroll down')):
      button = QtWidgets.QToolButton()
      button.setObjectName('scrollControl')
      button.setText(label)
      button.setToolTip(tooltip)
      button.setFixedSize(34, 34)
      button.clicked.connect(
        lambda _checked=False, scroll_area=area, step=direction: self._scroll_area(scroll_area, step))
      if direction > 0:
        layout.addStretch()
      layout.addWidget(button)
    return controls

  def _spin_controls(self, spin):
    spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
    spin.setMinimumWidth(52)
    container = QtWidgets.QWidget()
    layout = QtWidgets.QHBoxLayout(container)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(3)
    layout.addWidget(spin)
    for label, step, tooltip in (('▼', -1, 'Decrease value'), ('▲', 1, 'Increase value')):
      button = QtWidgets.QToolButton()
      button.setObjectName('spinControl')
      button.setText(label)
      button.setToolTip(tooltip)
      button.setFixedSize(27, 30)
      button.clicked.connect(
        lambda _checked=False, target=spin, amount=step:
          target.stepUp() if amount > 0 else target.stepDown())
      layout.addWidget(button)
    return container

  def _scroll_area(self, area, direction):
    scrollbar = area.verticalScrollBar()
    distance = max(48, scrollbar.pageStep() // 3)
    scrollbar.setValue(scrollbar.value() + direction * distance)

  def _apply_theme(self, dark):
    self._theme_dark = dark
    colors = {
      'background': '#0f172a' if dark else '#f4f7fb',
      'surface': '#1e293b' if dark else '#ffffff',
      'text': '#e2e8f0' if dark else '#18263d',
      'muted': '#d1deed' if dark else '#334155',
      'border': '#8fa3bd' if dark else '#94a3b8',
      'soft_border': '#52647c' if dark else '#b8c6d8',
      'input_disabled': '#304158' if dark else '#e3eaf3',
      'disabled_text': '#f1f5f9' if dark else '#1e293b',
      'accent': '#93c5fd' if dark else '#2563eb',
      'accent_button': '#2563eb',
      'accent_hover': '#1d4ed8',
      'accent_disabled': '#304158' if dark else '#e3eaf3',
      'hover': '#334155' if dark else '#eff6ff',
      'pressed': '#475569' if dark else '#dbeafe',
      'selection': '#1e3a8a' if dark else '#dbeafe',
      'danger': '#fca5a5' if dark else '#b91c1c',
      'danger_border': '#7f1d1d' if dark else '#fecaca',
      'danger_hover': '#3f1d27' if dark else '#fef2f2',
      'success': '#4ade80' if dark else '#15803d',
      'warning': '#fbbf24' if dark else '#b45309',
      'track': '#1e293b' if dark else '#e2e8f0',
      'handle': '#94a3b8' if dark else '#64748b',
      'header': '#273449' if dark else '#eaf0f8',
    }
    self._theme_colors = colors
    palette = self.palette()
    disabled_foreground = QtGui.QColor(colors['disabled_text'])
    disabled_background = QtGui.QColor(colors['input_disabled'])
    for role in (QtGui.QPalette.WindowText, QtGui.QPalette.Text,
                 QtGui.QPalette.ButtonText, QtGui.QPalette.PlaceholderText):
      palette.setColor(QtGui.QPalette.Disabled, role, disabled_foreground)
    for role in (QtGui.QPalette.Base, QtGui.QPalette.Button):
      palette.setColor(QtGui.QPalette.Disabled, role, disabled_background)
    self.setPalette(palette)
    self.setStyleSheet('''
      QWidget { color: %(text)s; font-family: "DejaVu Sans", sans-serif; font-size: 12px; }
      QMainWindow, QWidget#control_tab, QWidget#marker_name_tab, QWidget#settings_panel,
      QWidget#marker_content, QWidget#scrollControls { background: %(background)s; }
      QLabel#brandLabel { color: %(accent)s; font-size: 17px; font-weight: 700; }
      QLabel#pageTitle { color: %(text)s; font-size: 21px; font-weight: 700; }
      QLabel#sectionTitle { color: %(text)s; font-size: 15px; font-weight: 700; }
      QLabel#pageSubtitle, QLabel#markerCount { color: %(muted)s; }
      QTabWidget::pane { border: 0; background: %(background)s; }
      QTabBar::tab { background: transparent; color: %(muted)s; min-width: 180px;
                     padding: 11px 14px; margin: 2px 4px 0 2px;
                     border-bottom: 2px solid transparent; font-weight: 600; }
      QTabBar::tab:selected { color: %(accent)s; border-bottom: 2px solid %(accent)s; }
      QTabBar::tab:hover { color: %(accent)s; }
      QTabBar::tab:disabled { color: %(disabled_text)s; }
      QGroupBox { background: %(surface)s; border: 1px solid %(soft_border)s;
                  border-radius: 12px; margin-top: 15px; padding-top: 12px; font-weight: 600; }
      QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left;
                         left: 14px; padding: 0 5px; color: %(text)s; background: %(background)s; }
      QGroupBox::title:disabled { color: %(disabled_text)s; }
      QGroupBox[markerRow="true"] { margin-top: 0; padding-top: 0;
                                      border: 1px solid %(border)s; border-radius: 8px; }
      QLabel#markerHeading, QLabel#markerSerial { color: %(muted)s; font-size: 10px; }
      QLabel#markerValue { background: %(input_disabled)s; color: %(text)s;
                           border: 1px solid %(border)s; border-radius: 5px;
                           font-family: monospace; font-size: 10px; }
      QLineEdit, QSpinBox, QTextEdit { background: %(surface)s; color: %(text)s;
                                      border: 1px solid %(border)s; border-radius: 7px;
                                      padding: 4px 7px; selection-background-color: %(selection)s; }
      QLineEdit[markerName="true"] { font-size: 10px; padding: 2px 4px; }
      QLineEdit:focus, QSpinBox:focus, QTextEdit:focus { border: 2px solid %(accent)s; }
      QLineEdit:disabled, QSpinBox:disabled { background: %(input_disabled)s;
                                             color: %(disabled_text)s; border-color: %(border)s; }
      QLabel:disabled, QCheckBox:disabled, QRadioButton:disabled,
      QGroupBox:disabled { color: %(disabled_text)s; }
      QPushButton { background: %(surface)s; color: %(text)s; border: 1px solid %(border)s;
                    border-radius: 8px; padding: 8px 15px; font-weight: 600; min-height: 20px; }
      QPushButton:hover { background: %(hover)s; border-color: %(accent)s; }
      QPushButton:pressed { background: %(pressed)s; }
      QPushButton:disabled { background: %(input_disabled)s; color: %(disabled_text)s;
                             border-color: %(border)s; }
      QPushButton#start_node, QPushButton#push_refresh { background: %(accent_button)s;
                                                        border-color: %(accent_button)s; color: white; }
      QPushButton#start_node:hover, QPushButton#push_refresh:hover { background: %(accent_hover)s; }
      QPushButton#start_node:disabled, QPushButton#push_refresh:disabled {
          background: %(accent_disabled)s; border-color: %(border)s;
          color: %(disabled_text)s; }
      QPushButton#stop_node { color: %(danger)s; border-color: %(danger_border)s; }
      QPushButton#stop_node:hover { background: %(danger_hover)s; }
      QPushButton#stop_node:disabled { background: %(input_disabled)s;
                                       color: %(disabled_text)s; border-color: %(border)s; }
      QPushButton#themeToggle:checked { background: %(selection)s; border-color: %(accent)s; }
      QTextEdit#outputBox { background: %(surface)s; color: %(text)s;
                            border: 1px solid %(soft_border)s; border-radius: 10px; padding: 8px; }
      QScrollArea { border: 0; background: %(background)s; }
      QScrollBar:vertical { background: %(track)s; width: 14px; margin: 2px; border-radius: 6px; }
      QScrollBar::handle:vertical { background: %(handle)s; border-radius: 5px; min-height: 30px; }
      QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
      QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
      QScrollBar:horizontal { background: %(track)s; height: 14px; margin: 2px; border-radius: 6px; }
      QScrollBar::handle:horizontal { background: %(handle)s; border-radius: 5px; min-width: 30px; }
      QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
      QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }
      QToolButton#scrollControl, QToolButton#spinControl {
          background: %(surface)s; color: %(text)s; border: 1px solid %(border)s;
          border-radius: 8px; font-size: 16px; font-weight: 700; }
      QToolButton#scrollControl:hover, QToolButton#spinControl:hover {
          background: %(hover)s; border-color: %(accent)s; }
      QToolButton#scrollControl:pressed, QToolButton#spinControl:pressed {
          background: %(pressed)s; }
      QToolButton:disabled { background: %(input_disabled)s;
                             color: %(disabled_text)s; border-color: %(border)s; }
      QLCDNumber { background: %(input_disabled)s; color: %(accent)s;
                   border: 1px solid %(border)s; border-radius: 5px; }
      QToolTip { background: %(surface)s; color: %(text)s; border: 1px solid %(border)s; }
      QCheckBox { spacing: 8px; }
      QCheckBox::indicator { width: 16px; height: 16px; }
      QCheckBox::indicator:unchecked:disabled, QRadioButton::indicator:unchecked:disabled {
          background: %(input_disabled)s; border: 2px solid %(border)s; }
      QCheckBox::indicator:checked:disabled, QRadioButton::indicator:checked:disabled {
          background: %(accent)s; border: 2px solid %(border)s; }
    ''' % colors)
    self.theme_toggle.setText('Light mode' if dark else 'Dark mode')
    if hasattr(self, '_launch_status_text'):
      self._set_launch_status(self._launch_status_text, self._launch_status_tone)
    self._style_marker_service(getattr(self, '_marker_service_tone', 'muted'))
    self._render_logs()

  def _style_marker_service(self, tone):
    self._marker_service_tone = tone
    self.marker_service_label.setStyleSheet(
      f'color: {self._theme_colors[tone]}; font-weight: 600;')

  def _layout_settings_groups(self):
    for group, title in ((self.parameters, 'Parameters'),
                         (self.connection_settings, 'Connection settings'),
                         (self.publisher_setting, 'Publisher settings'),
                         (self.ros_network, 'ROS network'),
                         (self.logging_settings, 'Logging settings')):
      group.setTitle(title)
      group.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

    self.label_11.setText('World frame')
    self.label_12.setText('Node name')
    parameters = QtWidgets.QGridLayout(self.parameters)
    parameters.setContentsMargins(14, 22, 14, 12)
    parameters.setHorizontalSpacing(12)
    parameters.setVerticalSpacing(8)
    parameters.addWidget(self.label_11, 0, 0)
    parameters.addWidget(self.textFrameName, 0, 1)
    parameters.addWidget(self.label_12, 1, 0)
    parameters.addWidget(self.textNodeName, 1, 1)
    parameters.setColumnStretch(1, 1)

    self.label_5.setText('Server IP')
    self.label_6.setText('Client IP')
    self.label_7.setText('Server type')
    self.label_8.setText('Multicast address')
    self.label_9.setText('Command port')
    self.label_10.setText('Data port')
    self.textServerIP.setToolTip('Enter full server IPv4 address or its last octet on the selected client network.')
    self.textClientIP.setToolTip('Choose a detected interface or enter the client IPv4 address.')
    self.client_ip_spin.setFixedWidth(58)
    self.textCommandPort.setMinimumWidth(66)
    self.textDataPort.setMinimumWidth(66)
    connection = QtWidgets.QGridLayout(self.connection_settings)
    connection.setContentsMargins(14, 22, 14, 12)
    connection.setHorizontalSpacing(8)
    connection.setVerticalSpacing(7)
    connection.addWidget(self.label_5, 0, 0)
    connection.addWidget(self.textServerIP, 0, 1)
    connection.addWidget(self.label_6, 1, 0)
    connection.addWidget(self.textClientIP, 1, 1)
    connection.addWidget(QtWidgets.QLabel('Interface'), 2, 0)
    connection.addWidget(self._spin_controls(self.client_ip_spin), 2, 1, Qt.AlignLeft)
    connection.addWidget(self.label_7, 3, 0)
    server_type = QtWidgets.QVBoxLayout()
    server_type.setSpacing(0)
    server_type.addWidget(self.multicast_radio)
    server_type.addWidget(self.unicast_radio)
    connection.addLayout(server_type, 3, 1)
    connection.addWidget(self.label_8, 4, 0)
    connection.addWidget(self.textMulticastAddr, 4, 1)
    connection.addWidget(self.label_9, 5, 0)
    connection.addWidget(self.textCommandPort, 5, 1)
    connection.addWidget(self.label_10, 6, 0)
    connection.addWidget(self.textDataPort, 6, 1)
    connection.setColumnStretch(1, 1)

    for checkbox, text in ((self.pub_rb, 'Rigid bodies'),
                           (self.pub_rbm, 'Rigid body markers'),
                           (self.pub_im, 'Individual markers'),
                           (self.pub_pc, 'Point cloud'),
                           (self.pub_rbwmc, 'Body with marker config (coming soon)')):
      checkbox.setText(text)
    self.pub_rbwmc.setEnabled(False)
    self.pub_rbwmc.setToolTip('This publisher is not supported yet.')
    publishers = QtWidgets.QVBoxLayout(self.publisher_setting)
    publishers.setContentsMargins(14, 20, 14, 12)
    publishers.setSpacing(4)
    for checkbox in (self.pub_rb, self.pub_rbm, self.pub_im, self.pub_pc, self.pub_rbwmc):
      publishers.addWidget(checkbox)

    network = QtWidgets.QHBoxLayout(self.ros_network)
    network.setContentsMargins(14, 20, 14, 12)
    network.setSpacing(12)
    network.addWidget(self.label_13)
    network.addWidget(self._spin_controls(self.domain_id_spin))
    network.addStretch()

    logging = QtWidgets.QHBoxLayout(self.logging_settings)
    logging.setContentsMargins(14, 20, 14, 12)
    logging.setSpacing(16)
    for checkbox in (self.log_internal, self.log_frames, self.log_latencies):
      logging.addWidget(checkbox)
    logging.addStretch()

  def _set_launch_status(self, text, tone):
    self._launch_status_text = text
    self._launch_status_tone = tone
    self.launch_status_label.setText(text)
    self.launch_status_label.setStyleSheet(
      f'background: {self._theme_colors["surface"]}; color: {self._theme_colors[tone]}; '
      f'border: 1px solid {self._theme_colors["soft_border"]}; '
      'border-radius: 10px; padding: 9px 12px; font-weight: 700;')
    self.start_node.setEnabled(not self.is_running)
    self.stop_node.setEnabled(self.is_running)

  def Log(self,type:str="",msg=''):
    self._log_entries.append((type, str(msg)))
    self.outputBox.append(self._format_log(type, msg))
    self.outputBox.moveCursor(QtGui.QTextCursor.End)
    self.outputBox.ensureCursorVisible()
    self.outputBox.verticalScrollBar().setValue(self.outputBox.verticalScrollBar().maximum())

  def _format_log(self, type, msg):
    if type == 'block':
      return f'<span style="color:{self._theme_colors["muted"]}">────────────────────────</span>'
    tone = {'error': 'danger', 'warn': 'warning'}.get(type, 'text')
    color = self._theme_colors[tone]
    label = type.upper() if type in ('info', 'error', 'warn') else 'INFO'
    return (f'<span style="color:{self._theme_colors["text"]}">'
            f'<span style="color:{color};font-weight:600">[{label}]</span> '
            f'{html.escape(str(msg))}</span>')

  def _render_logs(self):
    self.outputBox.clear()
    for type, msg in self._log_entries:
      self.outputBox.append(self._format_log(type, msg))
    self.outputBox.moveCursor(QtGui.QTextCursor.End)

  def _clear_logs(self):
    self._log_entries.clear()
    self.outputBox.clear()

  def _is_valid_ipv4(self, ip_text: str) -> bool:
    octets = str(ip_text).strip().split('.')
    if len(octets) != 4:
      return False
    try:
      return all(0 <= int(octet) <= 255 for octet in octets)
    except ValueError:
      return False

  def _is_valid_node_name(self, node_name: str) -> bool:
    name = str(node_name).strip()
    return bool(re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name))

  def _is_valid_port(self, port: int) -> bool:
    return isinstance(port, int) and 1 <= port <= 65535

  def _log_exception(self, context: str, exception: Exception):
    self.Log('error', f' {context}: {exception}')
    self.node.get_logger().error(f'{context}: {exception}')
    self.node.get_logger().error(traceback.format_exc())

  def _mask_ip_for_gui(self, ip_text: str) -> str:
    ip_str = str(ip_text).strip()
    if self._is_valid_ipv4(ip_str):
      octets = ip_str.split('.')
      return f'{octets[0]}.***.***.{octets[3]}'
    return ip_str

  def _masked_server_hint_from_client(self, client_ip_text: str) -> str:
    client_ip = str(client_ip_text).strip()
    if not self._is_valid_ipv4(client_ip):
      return ''
    client_octets = client_ip.split('.')
    return f'{client_octets[0]}.***.***.'

  def _resolve_workspace_setup_script(self):
    candidate_paths = []
    env_workspace = os.environ.get('NATNET_WS')
    if env_workspace:
      candidate_paths.append(Path(env_workspace).expanduser())

    cwd_path = Path(os.getcwd()).resolve()
    candidate_paths.extend([cwd_path, *cwd_path.parents])

    pkg_path = Path(PKG_PATH).resolve()
    candidate_paths.extend([pkg_path, *pkg_path.parents])

    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    for prefix in ament_prefix_path.split(os.pathsep):
      if not prefix:
        continue
      prefix_path = Path(prefix).expanduser().resolve()
      candidate_paths.extend([prefix_path, *prefix_path.parents])

    checked = set()
    for path in candidate_paths:
      resolved_path = path.resolve()
      if resolved_path in checked:
        continue
      checked.add(resolved_path)

      workspace_root = resolved_path
      if resolved_path.name in ('install', 'src'):
        workspace_root = resolved_path.parent

      install_dir = workspace_root / 'install'
      if not install_dir.is_dir():
        continue

      for setup_name in ('setup.bash', 'local_setup.bash'):
        setup_path = install_dir / setup_name
        if setup_path.is_file():
          return str(workspace_root), str(setup_path)

    return None, None

  def _run_preflight_checks(self) -> bool:
    self.ros2_executable = shutil.which('ros2')
    self.workspace_root, self.workspace_setup_script = self._resolve_workspace_setup_script()
    package_share_exists = os.path.isdir(PKG_PATH)

    ok = True
    if self.ros2_executable is None:
      self.Log('error', ' Preflight failed: ros2 executable was not found in PATH.')
      self.node.get_logger().error('Preflight failed: ros2 executable was not found in PATH.')
      ok = False
    if self.workspace_root is None or self.workspace_setup_script is None:
      self.Log('error', ' Preflight failed: could not locate workspace install setup script (install/setup.bash).')
      self.node.get_logger().error('Preflight failed: could not locate workspace install setup script (install/setup.bash). Set NATNET_WS or launch from a valid workspace.')
      ok = False
    if not package_share_exists:
      self.Log('error', f' Preflight failed: package share path does not exist: {PKG_PATH}')
      self.node.get_logger().error(f'Preflight failed: package share path does not exist: {PKG_PATH}')
      ok = False

    if ok:
      self.Log('info', f' Preflight OK: workspace={self.workspace_root}')
      self.node.get_logger().info(f'Preflight OK: workspace={self.workspace_root}, setup={self.workspace_setup_script}, ros2={self.ros2_executable}')
    return ok

  def _refresh_marker_service_status(self):
    if self.marker_refresh_thread is not None:
      return
    is_ready = self.node.marker_service_ready(timeout_sec=0.0)
    self.marker_service_available = is_ready
    self.push_refresh.setEnabled(is_ready)
    self.marker_service_label.setText('Service ready' if is_ready else 'Service unavailable')
    self._style_marker_service('success' if is_ready else 'warning')

    if is_ready:
      self.push_refresh.setToolTip('Refresh marker poses from marker_poses_server.')
    else:
      self.push_refresh.setToolTip('Marker service unavailable. Start marker_poses_server to enable refresh.')

    if self._last_marker_service_state is None or self._last_marker_service_state != is_ready:
      if is_ready:
        self.Log('info', ' Marker service is available. Refresh is enabled.')
        self.node.get_logger().info('Marker service is available. Refresh is enabled.')
      else:
        self.Log('warn', ' Marker service is unavailable. Refresh is disabled and will retry automatically.')
        self.node.get_logger().warn('Marker service is unavailable. Refresh is disabled and will retry automatically.')
      self._last_marker_service_state = is_ready

  def _clear_marker_display(self):
    self.marker_count_label.setText('0 markers')
    self.push_ok.setEnabled(False)
    for index in range(1, 41):
      getattr(self, f'name_{index}').setEnabled(False)
      for axis in ('X', 'Y', 'Z'):
        self.marker_value_labels[index, axis].setText('—')
        self.marker_value_labels[index, axis].setToolTip('')

  def _resolve_client_ip_text(self, client_text: str) -> str:
    value = str(client_text).strip()
    if value == '':
      return ''
    if self.client_ip_raw is not None and value == self._mask_ip_for_gui(self.client_ip_raw):
      return self.client_ip_raw
    if self._is_valid_ipv4(value):
      return value
    return ''

  def _resolve_server_ip_text(self, server_text: str) -> str:
    value = str(server_text).strip()
    if value == '':
      return ''

    if self.server_ip_raw is not None and value == self._mask_ip_for_gui(self.server_ip_raw):
      return self.server_ip_raw

    if self._is_valid_ipv4(value):
      return value

    if self.client_ip_raw is None or not self._is_valid_ipv4(self.client_ip_raw):
      return ''

    client_octets = self.client_ip_raw.split('.')
    if re.fullmatch(r'\d{1,3}', value):
      last_octet = int(value)
      if 0 <= last_octet <= 255:
        return f'{client_octets[0]}.{client_octets[1]}.{client_octets[2]}.{last_octet}'

    masked_match = re.match(r'^\d{1,3}\.\*{3}\.\*{3}\.(\d{1,3})$', value)
    if masked_match:
      last_octet = int(masked_match.group(1))
      if 0 <= last_octet <= 255:
        return f'{client_octets[0]}.{client_octets[1]}.{client_octets[2]}.{last_octet}'

    return ''

  def _apply_client_ip_visual_mask(self):
    resolved_client_ip = self._resolve_client_ip_text(self.textClientIP.text())
    if resolved_client_ip != '':
      self.client_ip_raw = resolved_client_ip
      self.conn_params["clientIP"] = resolved_client_ip
      self.textClientIP.setText(self._mask_ip_for_gui(resolved_client_ip))

  def _apply_server_ip_visual_mask(self):
    resolved_server_ip = self._resolve_server_ip_text(self.textServerIP.text())
    if resolved_server_ip != '':
      self.server_ip_raw = resolved_server_ip
      self.conn_params["serverIP"] = resolved_server_ip
      self.textServerIP.setText(self._mask_ip_for_gui(resolved_server_ip))

  def _on_launch_started(self, pid: int, pgid: int):
    self.launch_proc = pid
    self.launch_pgid = pgid
    self.is_running = True
    self._set_launch_status('Running', 'success')
    self.Log('info', f' Started natnet launch process (pid={pid}, pgid={pgid})')
    self.node.get_logger().info(f'Started natnet launch process (pid={pid}, pgid={pgid})')

  def _on_launch_log_line(self, level: str, line: str):
    if level == 'ERROR':
      self.Log('error', f' [LAUNCH][STDERR] {line}')
      self.node.get_logger().error(f'[LAUNCH][STDERR] {line}')
    else:
      self.Log('info', f' [LAUNCH][STDOUT] {line}')
      self.node.get_logger().info(f'[LAUNCH][STDOUT] {line}')

  def _on_launch_finished(self, return_code: int):
    if return_code == 0:
      self.Log('info', f' NatNet launch exited with code {return_code}')
      self.node.get_logger().info(f'NatNet launch exited with code {return_code}')
    else:
      self.Log('error', f' NatNet launch exited with code {return_code}')
      self.node.get_logger().error(f'NatNet launch exited with code {return_code}')
    self._clear_launch_state()
    self._set_launch_status('Stopped' if return_code == 0 else 'Launch failed',
                            'muted' if return_code == 0 else 'danger')

  def _clear_launch_state(self):
    self.launch_proc = None
    self.launch_pgid = None
    self.is_running = False
    self.start_node_thread = None
    self._set_launch_status('Stopped', 'muted')

  def _is_tracked_process_alive(self) -> bool:
    if self.start_node_thread and self.start_node_thread.proc is not None:
      return self.start_node_thread.proc.poll() is None
    if self.launch_proc is None:
      return False
    try:
      os.kill(self.launch_proc, 0)
    except ProcessLookupError:
      return False
    except PermissionError:
      return True
    return True

  def _wait_for_launch_exit(self, timeout_sec: float) -> bool:
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
      if not self._is_tracked_process_alive():
        return True
      time.sleep(0.1)
    return not self._is_tracked_process_alive()

  def _send_lifecycle_shutdown(self):
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(self.id)
    command = ['ros2', 'lifecycle', 'set', f'/{self.name}', 'shutdown']
    self.Log('info', f" Sending lifecycle shutdown to /{self.name}")
    self.node.get_logger().info(f"Sending lifecycle shutdown to /{self.name}")
    result = subprocess.run(command, check=False, env=env, capture_output=True, text=True, timeout=4)
    if result.stdout.strip():
      self.Log('info', f" [LIFECYCLE] {result.stdout.strip()}")
      self.node.get_logger().info(f"[LIFECYCLE] {result.stdout.strip()}")
    if result.stderr.strip():
      self.Log('error', f" [LIFECYCLE] {result.stderr.strip()}")
      self.node.get_logger().error(f"[LIFECYCLE] {result.stderr.strip()}")

  def _terminate_tracked_group(self, sig):
    if self.launch_pgid is not None:
      os.killpg(self.launch_pgid, sig)
    elif self.launch_proc is not None:
      os.kill(self.launch_proc, sig)

  def shutdown_launch_process(self, reason: str):
    if not self._is_tracked_process_alive() and not self.is_running:
      self._clear_launch_state()
      return

    self.Log('block')
    self.Log('info', f' Stopping natnet launch ({reason})')
    self.node.get_logger().info(f'Stopping natnet launch ({reason})')
    try:
      self._send_lifecycle_shutdown()
      if self._wait_for_launch_exit(timeout_sec=2.0):
        self.Log('info', ' Launch exited after lifecycle shutdown')
      else:
        self.Log('warn', ' Launch still active, sending SIGTERM to tracked process group')
        self.node.get_logger().warn('Launch still active, sending SIGTERM to tracked process group')
        self._terminate_tracked_group(signal.SIGTERM)
        if self._wait_for_launch_exit(timeout_sec=3.0):
          self.Log('info', ' Launch exited after SIGTERM')
        else:
          self.Log('warn', ' Launch still active, sending SIGKILL to tracked process only')
          self.node.get_logger().warn('Launch still active, sending SIGKILL to tracked process only')
          if self.launch_proc is not None:
            os.kill(self.launch_proc, signal.SIGKILL)
          if self._wait_for_launch_exit(timeout_sec=2.0):
            self.Log('info', ' Launch exited after SIGKILL')
          else:
            self.Log('error', ' Failed to stop tracked launch process')
            self.node.get_logger().error('Failed to stop tracked launch process')
    except Exception as e:
      self._log_exception('Failed while stopping tracked natnet launch process', e)
    finally:
      self._clear_launch_state()
      self.Log('block')

  def closeEvent(self, event):
    if self.marker_service_check_timer.isActive():
      self.marker_service_check_timer.stop()
    if self.marker_refresh_thread is not None:
      if not self.marker_refresh_thread.wait(22000):
        self.Log('warn', ' Marker refresh is still stopping. Close the window after it finishes.')
        event.ignore()
        return
    self.shutdown_launch_process('GUI closed')
    super().closeEvent(event)

#-----------------------------------------------------------------------------------------
# START/STOP NATNET BUTTON

  def start(self):
    self.Log('block')
    if self._is_tracked_process_alive() or self.is_running:
      self.Log('warn', ' Start ignored. NatNet launch is already running for this GUI session.')
      self.node.get_logger().warn('Start ignored. NatNet launch is already running for this GUI session.')
      self.Log('block')
      return
    try:
      self.check_all_params()
      if not self.error_pass:
        self.Log('error', ' Launch aborted because one or more required parameters are invalid.')
        self.node.get_logger().error('Launch aborted because one or more required parameters are invalid.')
        self.Log('block')
        return
      if not self._run_preflight_checks():
        self.Log('error', ' Launch aborted due to failed preflight checks.')
        self.node.get_logger().error('Launch aborted due to failed preflight checks.')
        self.Log('block')
        return

      env = os.environ.copy()
      env['ROS_DOMAIN_ID'] = str(self.id)

      def bool_str(value):
        return str(bool(value)).lower()

      command = [
        'ros2', 'launch', 'natnet_ros2', 'natnet_ros2.launch.py',
        f'node_name:={self.name}',
        f'serverIP:={self.conn_params["serverIP"]}',
        f'clientIP:={self.conn_params["clientIP"]}',
        f'serverType:={self.conn_params["serverType"]}',
        f'multicastAddress:={self.conn_params["multicastAddress"]}',
        f'serverCommandPort:={self.conn_params["serverCommandPort"]}',
        f'serverDataPort:={self.conn_params["serverDataPort"]}',
        f'global_frame:={self.conn_params["globalFrame"]}',
        'remove_latency:=false',
        f'pub_rigid_body:={bool_str(self.pub_params["pub_rigid_body"])}',
        f'pub_rigid_body_marker:={bool_str(self.pub_params["pub_rigid_body_marker"])}',
        f'pub_individual_marker:={bool_str(self.pub_params["pub_individual_marker"])}',
        f'pub_pointcloud:={bool_str(self.pub_params["pub_pointcloud"])}',
        f'log_internals:={bool_str(self.log_params["log_internals"])}',
        f'log_frames:={bool_str(self.log_params["log_frames"])}',
        f'log_latencies:={bool_str(self.log_params["log_latencies"])}',
        'conf_file:=conf_autogen.yaml',
        'activate:=true',
        f'immt:={self.pub_params["individual_marker_msg_type"]}',
      ]

      launch_cwd = self.workspace_root if self.workspace_root is not None else self.pwd
      self.start_node_thread = WorkerThread(command, env=env, cwd=launch_cwd)
      self.start_node_thread.started_signal.connect(self._on_launch_started)
      self.start_node_thread.log_line_signal.connect(self._on_launch_log_line)
      self.start_node_thread.finished_signal.connect(self._on_launch_finished)
      self.is_running = True
      self._set_launch_status('Starting…', 'accent')
      self.start_node_thread.start()
      self.Log('info', ' Starting natnet launch process...')
      self.node.get_logger().info('Starting natnet launch process...')
    except Exception as e:
      self._log_exception('Failed to start natnet launch process', e)
      self._clear_launch_state()
    finally:
      self.Log('block')

  def stop(self):
    self.shutdown_launch_process('Stop button pressed')

#----------------------------------------------------------------------------------------
# PUBLISHING RELATED STUFF

  def pub_im_setting(self):
    self.Log('block')
    
    if self.pub_im.isChecked():
      self.set_conn_params('marker_poses_server')
    if self.error_pass:
      if self.pub_im.isChecked():
        self.pub_params["pub_individual_marker"] = True
        self.Log('block')
        msg = '''
        It will only take upto 40 unlabled markers from the available list.
        If you do not see the marker in the list, make sure things other than markers are masked and markers are clearly visible.
        '''
        self.Log('info',msg)
        self.Log('info','Go to Single marker naming tab and press the refresh button to complete the configuration of for initial position of the markers (wait for a second or two after pressing the refresh).')
        self.Log('info',' If you do not wish to name some marker, you can leave it empty. Do not repeat names of the markers.')
      else:
        self.pub_params["pub_individual_marker"] = False
    else:
      self.Log('error','Can not get the data of markers from the natnet server. One or more parameters from conenction settings are missing')

  def pub_rb_setting(self):
    if self.pub_rb.isChecked():
      self.pub_params["pub_rigid_body"] = True
      self.Log('info','Enabled publishing rigidbody')
      self.pub_rbm.setEnabled(True)
    else:
      self.pub_params["pub_rigid_body"] = False
      self.pub_rbm.setEnabled(False)

  def pub_rbm_setting(self):
    if self.pub_rbm.isChecked():
      self.pub_params["pub_rigid_body_marker"] = True
      self.Log('info','Enabled publishing rigidbody markers')
    else:
      self.pub_params["pub_rigid_body_marker"] = False

  def pub_pc_setting(self):
    if self.pub_pc.isChecked():
      self.pub_params["pub_pointcloud"] = True
      self.Log('info','Enabled publishing pointcloud')
    else:
      self.pub_params["pub_pointcloud"] = False
  
  def pub_rbwmc_setting(self):
    self.Log('info','This functionality is not supported yet.')
    
  def pub_immt_setting(self):
    self.pub_params["individual_marker_msg_type"] = self.im_msg_type

#----------------------------------------------------------------------------------------
# NETWORK SELECTION THINGS

  def select_network(self):
    self.id = int(self.domain_id_spin.value())
    self.Log('warn','Do not cahnge Domain ID until you know what you are trying.')
    self.Log('info','Using ROS domain ID: '+str(self.id))

#----------------------------------------------------------------------------------------
# NATNET CONNECTION RELATED

  def get_server_ip(self):
    resolved_server_ip = self._resolve_server_ip_text(self.textServerIP.text())
    self.conn_params["serverIP"] = resolved_server_ip
    self.server_ip_raw = resolved_server_ip if resolved_server_ip != '' else None
    if resolved_server_ip != '':
      self.textServerIP.setText(self._mask_ip_for_gui(resolved_server_ip))
      self.Log('info','setting servet ip '+self._mask_ip_for_gui(resolved_server_ip))
    #self.Log('info','setting server ip '+str(self.conn_params["serverIP"]).split('.')[0]+'***'+'***'+str(self.conn_params["serverIP"]).split('.')[-1])

  def get_client_ip(self):
    resolved_client_ip = self._resolve_client_ip_text(self.textClientIP.text())
    self.conn_params["clientIP"] = resolved_client_ip
    self.client_ip_raw = resolved_client_ip if resolved_client_ip != '' else self.client_ip_raw
    if resolved_client_ip != '':
      self.textClientIP.setText(self._mask_ip_for_gui(resolved_client_ip))
      self.Log('info','setting client ip '+self._mask_ip_for_gui(resolved_client_ip))
    #self.Log('info','setting client ip '+str(self.conn_params["clientIP"]).split('.')[0]+'***'+'***'+str(self.conn_params["clientIP"]).split('.')[-1])

  def get_server_type(self):
    if self.multicast_radio.isChecked():
      self.conn_params["serverType"] = 'multicast'
      self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))
      self.get_multicast_addr()
    if self.unicast_radio.isChecked():
      self.conn_params["serverType"] = 'unicast'
      self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def clicked_unicast(self):
    self.conn_params["serverType"] = 'unicast'
    self.textMulticastAddr.setEnabled(False)
    self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def clicked_multicast(self):
    self.conn_params["serverType"] = 'multicast'
    self.textMulticastAddr.setEnabled(True)
    self.Log('info','setting broadcat to '+str(self.conn_params["serverType"]))

  def get_multicast_addr(self):
    self.conn_params["multicastAddress"] = self.textMulticastAddr.text()
    self.Log('info','setting multicast address '+str(self.conn_params["multicastAddress"]))

  def get_command_port(self):
    port_text = self.textCommandPort.text()
    try:
      port_value = int(port_text)
    except ValueError:
      self.conn_params["serverCommandPort"] = None
      self.error_pass = False
      self.Log('error', f'Invalid command port value: {port_text}. Use an integer between 1 and 65535.')
      self.node.get_logger().error(f'Invalid command port value: {port_text}. Use an integer between 1 and 65535.')
      return

    if not self._is_valid_port(port_value):
      self.conn_params["serverCommandPort"] = None
      self.error_pass = False
      self.Log('error', f'Command port out of range: {port_value}. Allowed range is 1-65535.')
      self.node.get_logger().error(f'Command port out of range: {port_value}. Allowed range is 1-65535.')
      return

    self.conn_params["serverCommandPort"] = port_value
    self.Log('info','setting command port '+str(self.conn_params["serverCommandPort"]))

  def get_data_port(self):
    port_text = self.textDataPort.text()
    try:
      port_value = int(port_text)
    except ValueError:
      self.conn_params["serverDataPort"] = None
      self.error_pass = False
      self.Log('error', f'Invalid data port value: {port_text}. Use an integer between 1 and 65535.')
      self.node.get_logger().error(f'Invalid data port value: {port_text}. Use an integer between 1 and 65535.')
      return

    if not self._is_valid_port(port_value):
      self.conn_params["serverDataPort"] = None
      self.error_pass = False
      self.Log('error', f'Data port out of range: {port_value}. Allowed range is 1-65535.')
      self.node.get_logger().error(f'Data port out of range: {port_value}. Allowed range is 1-65535.')
      return

    self.conn_params["serverDataPort"] = port_value
    self.Log('info','setting data port '+str(self.conn_params["serverDataPort"]))

  def get_world_frame(self):
    self.conn_params["globalFrame"] = self.textFrameName.text()
    if self.conn_params["globalFrame"]=='':
      self.conn_params["globalFrame"]='world'
      self.Log('warn','setting world frame name to world as no input provided')
    else:
      self.Log('info','setting world frame name '+str(self.conn_params["globalFrame"]))

  def spin_clientIP(self):
    selected_client_ip = str(self.IP_LIST[self.client_ip_spin.value()-1])
    self.client_ip_raw = selected_client_ip
    self.conn_params["clientIP"] = selected_client_ip
    self.server_ip_raw = None
    self.textClientIP.setText(self._mask_ip_for_gui(selected_client_ip))
    self.textServerIP.setText(self._masked_server_hint_from_client(selected_client_ip))
    self.Log('info','setting client ip '+self._mask_ip_for_gui(self.conn_params["clientIP"]))

  def spin_msg_type(self):
    self.textMsgType.setText(self.msg_types[self.msg_type_spin.value()-1])
    self.im_msg_type = self.msg_types[self.msg_type_spin.value()-1]

  def get_node_name(self):
    self.name = self.textNodeName.text().strip()
    if self.name=='':
      self.name='natnet_ros2'
      self.Log('warn','setting natnet_ros2 name to world as no input provided')
    elif not self._is_valid_node_name(self.name):
      self.Log('error', f'Invalid node name: {self.name}. Use pattern [A-Za-z_][A-Za-z0-9_]*')
      self.node.get_logger().error(f'Invalid node name: {self.name}. Use pattern [A-Za-z_][A-Za-z0-9_]*')
      self.error_pass = False
    else:
      self.Log('info','setting name '+self.name)
#----------------------------------------------------------------------------------------
# PARAM CHECK RELATED

  def chek_conn_params(self):
    self.error_pass = True
    self.get_server_ip()
    if self.conn_params["serverIP"]==None or self.conn_params["serverIP"]=='':
      self.Log('error','No server ip provided. Can not connect.')
      self.error_pass=False
    self.get_client_ip()
    if self.conn_params["clientIP"]==None or self.conn_params["clientIP"]=='':
      self.Log('error','No client ip provided. Can not connect.')
      self.error_pass=False
    self.get_server_type()
    self.get_command_port()
    if self.conn_params["serverCommandPort"]==None or self.conn_params["serverCommandPort"]=='':
      self.Log('error','No command port provided. Can not connect.')
      self.error_pass=False
    self.get_data_port()
    if self.conn_params["serverDataPort"]==None or self.conn_params["serverDataPort"]=='':
      self.Log('error','No data port provided. Can not connect.')
      self.error_pass=False
    self.get_world_frame()
    self.get_node_name()

  def set_conn_params(self,node_name:str):
    self.chek_conn_params()
    if self.error_pass:
      try:
        if node_name == 'marker_poses_server' and not self.node.marker_service_ready(timeout_sec=0.1):
          self.error_pass = False
          self.Log('warn', ' Marker service unavailable; cannot update connection parameters right now.')
          self.node.get_logger().warn('Marker service unavailable; cannot update connection parameters right now.')
          return
        self.node.call_set_parameters(node_name,self.conn_params)
      except RuntimeError as e:
        self.error_pass = False
        self.Log('error', f' Failed to set parameters on {node_name}: {e}')
        self.node.get_logger().error(f'Failed to set parameters on {node_name}: {e}')
      except Exception as e:
        self.error_pass = False
        self._log_exception(f'Unexpected error while setting parameters on {node_name}', e)

  def check_all_params(self):
    self.chek_conn_params()
    self.pub_im_setting()
    self.pub_rb_setting()
    self.pub_rbm_setting()
    self.pub_pc_setting()
    self.pub_immt_setting()
    self.pub_params["pub_rigid_body_wmc"] = False
    self.log_frames_setting()
    self.log_internal_setting()
    self.log_latencies_setting()

#----------------------------------------------------------------------------------------
# MARKER POSE SERVER RELATED

  def set_lcds(self,num_of_markers,x_position,y_position,z_position):
    x_len = len(x_position)
    y_len = len(y_position)
    z_len = len(z_position)
    if not (x_len == y_len == z_len == num_of_markers):
      self.Log('error', f' Length mismatch for marker payload. expected={num_of_markers}, x={x_len}, y={y_len}, z={z_len}.')
      self.node.get_logger().error(f'Length mismatch for marker payload. expected={num_of_markers}, x={x_len}, y={y_len}, z={z_len}.')
      self._clear_marker_display()
      self.num_of_markers = 0
      self.x_position = None
      self.y_position = None
      self.z_position = None
      return

    visible_count = min(40, num_of_markers)
    for index in range(1, 41):
      visible = index <= visible_count
      getattr(self, f'name_{index}').setEnabled(visible)
      for axis, values in (('X', x_position), ('Y', y_position), ('Z', z_position)):
        label = self.marker_value_labels[index, axis]
        label.setText(f'{values[index - 1]:.3f}' if visible else '—')
        label.setToolTip(str(values[index - 1]) if visible else '')

    self.num_of_markers = num_of_markers
    self.marker_count_label.setText(
      f'{num_of_markers} markers' if num_of_markers <= 40 else f'Showing 40 of {num_of_markers} markers')
    self.push_ok.setEnabled(visible_count > 0)
    self.x_position = x_position
    self.y_position = y_position
    self.z_position = z_position

  def yaml_dump(self):
    self.im_msg_type = self.msg_types[self.msg_type_spin.value()-1]
    object_names={'object_names':[]}
    if self.config_file is None:
      self.Log('error', ' Configuration file path is not initialized. Cannot save marker configuration.')
      self.node.get_logger().error('Configuration file path is not initialized. Cannot save marker configuration.')
      return

    if self.num_of_markers!=0:
      for i in range(min(40,self.num_of_markers)):
        name_widget = getattr(self, f'name_{i+1}', None)
        if name_widget is None:
          continue
        marker_name = name_widget.text().strip()
        if marker_name == '':
          continue

        object_names['object_names'].append(marker_name)
        object_names[marker_name] = {
          'marker_config': 0,
          'pose': {
            'position': [self.x_position[i], self.y_position[i], self.z_position[i]],
            'orientation': [0, 0, 0],
          },
        }

      self.natnet_params = object_names
      try:
        if os.path.exists(self.config_file):
          os.remove(self.config_file)
      except OSError as e:
        self.Log('warn', f' Could not remove existing config file: {e}')
        self.node.get_logger().warn(f'Could not remove existing config file: {e}')

      with open(self.config_file,'w') as f:
        yaml.dump({self.name: {'ros__parameters':self.natnet_params}},f,indent=2,default_flow_style=False)
        f.close()
      self.Log('info', f' Saved {len(object_names["object_names"])} marker names.')
    else:
      self.Log('error','Number of markers are not recieved. Something went wrong.')

  def call_MarkerPoses_srv(self):
    if self.marker_refresh_thread is not None:
      return
    if not self.node.marker_service_ready(timeout_sec=0.1):
      self.marker_service_available = False
      self.push_refresh.setEnabled(False)
      self.push_refresh.setToolTip('Marker service unavailable. Start marker_poses_server to enable refresh.')
      self.Log('warn', ' Marker service is unavailable. Refresh request skipped.')
      self.node.get_logger().warn('Marker service is unavailable. Refresh request skipped.')
      return

    self.set_conn_params('marker_poses_server')
    if self.error_pass:
      self.marker_service_check_timer.stop()
      self.push_refresh.setEnabled(False)
      self.marker_service_label.setText('Refreshing…')
      self._style_marker_service('accent')
      self.marker_refresh_thread = MarkerRefreshThread(self.node)
      self.marker_refresh_thread.result_signal.connect(self._on_marker_refresh_result)
      self.marker_refresh_thread.error_signal.connect(self._on_marker_refresh_error)
      self.marker_refresh_thread.finished.connect(self._finish_marker_refresh)
      self.marker_refresh_thread.start()

  def _on_marker_refresh_result(self, response):
    self.set_lcds(response.num_of_markers, response.x_position, response.y_position, response.z_position)

  def _on_marker_refresh_error(self, message):
    self._clear_marker_display()
    self.num_of_markers = 0
    self.x_position = None
    self.y_position = None
    self.z_position = None
    self.Log('error', 'Service call failed: ' + message)
    self.node.get_logger().error('Service call failed: ' + message)

  def _finish_marker_refresh(self):
    self.marker_refresh_thread = None
    self._refresh_marker_service_status()
    self.marker_service_check_timer.start()

#----------------------------------------------------------------------------------------
# LOGGING RELATED

  def log_frames_setting(self):
    if self.log_frames.isChecked():
      self.log_params["log_frames"] = True
      self.Log('info','Enabled logging frames in terminal')
    else:
      self.log_params["log_frames"] = False

  def log_internal_setting(self):
    if self.log_internal.isChecked():
      self.log_params["log_internals"] = True
      self.Log('info','Enabled logging internal in terminal')
    else:
      self.log_params["log_internals"] = False

  def log_latencies_setting(self):
    if self.log_latencies.isChecked():
      self.log_params["log_latencies"] = True
      self.Log('info','Enabled logging latencies in terminal')
    else:
      self.log_params["log_latencies"] = False


def main(args=None):
    rclpy.init(args=args)

    node = HelperNode()
    window = None

    app = QtWidgets.QApplication(sys.argv)
    window = PyQt5Widget(node)
    window.show()

    try:
      sys.exit(app.exec_())
    finally:
      if window is not None:
        window.shutdown_launch_process('Application shutdown')
      # Clean up ROS when the application is closed
      node.destroy_node()
      rclpy.shutdown()

if __name__ == '__main__':
    main()
