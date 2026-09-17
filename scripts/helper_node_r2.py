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
import signal
import threading
import time
import traceback
import yaml

from PyQt5 import QtWidgets, uic
import sys

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal

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

class PyQt5Widget(QtWidgets.QMainWindow):
  def __init__(self,node:Node):
    super(PyQt5Widget, self).__init__()
    self.node = node
    ui_file = os.path.join(PKG_PATH, 'ui', 'helper.ui')
    print('')
    print(ui_file)
    print('')
    uic.loadUi(ui_file, self)
    self.setWindowIcon(QtGui.QIcon(os.path.join(PKG_PATH,'ui','logo.png')))

    self.ros_dist = os.environ['ROS_DISTRO']
    self.pwd = os.getcwd()
    self.msg_types = ["PoseStamped","PointStamped"]
    self.im_msg_type = self.msg_types[0]
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
    
    self.outputBox.append("""<div style="color: #ff8c00">[NOTE]</div>""")
    self.outputBox.append('- This prompt does not show all the errors. For full error, check the terminal where this node has been excecuted')
    self.outputBox.append('- It is required to launch the node from your main workspace folder where this package has been built')
    self.outputBox.append('- Stop only affects the launch process started by this GUI session.')
    self.Log('info',' If your WIFI or LAN IP is not detected in the list of the network, You can use the remote IP option and add the ip address manually.')
    self.Log('info',' rosmaster will be selected initially based on the environment variables. You can change it anytime. Provided rosmaster will be used to run the main natnet node.')
    #self.Log('info',' ')
    IP_data = subprocess.check_output(['lshw','-c','network']).decode('utf-8')
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

  def Log(self,type:str="",msg=''):
    html_msg = msg
    if type=="info":
      html_msg = """<div style="color: #656565">[INFO]</div>""" +msg
    if type=="error":
      html_msg = """<div style="color:red">[ERROR]</div>""" +msg
    if type=="warn":
      html_msg = """<div style="color: #ffae42">[WARN]</div>""" +msg
    if type=="block":
      html_msg = """<div style="color: #656565">--------------------</div>"""
    self.outputBox.append(html_msg)
    self.outputBox.moveCursor(QtGui.QTextCursor.End)
    self.outputBox.ensureCursorVisible()
    self.outputBox.verticalScrollBar().setValue(self.outputBox.verticalScrollBar().maximum())

  def _is_valid_ipv4(self, ip_text: str) -> bool:
    octets = str(ip_text).strip().split('.')
    if len(octets) != 4:
      return False
    try:
      return all(0 <= int(octet) <= 255 for octet in octets)
    except ValueError:
      return False

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

  def _clear_launch_state(self):
    self.launch_proc = None
    self.launch_pgid = None
    self.is_running = False
    self.start_node_thread = None

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

      self.start_node_thread = WorkerThread(command, env=env, cwd=self.pwd)
      self.start_node_thread.started_signal.connect(self._on_launch_started)
      self.start_node_thread.log_line_signal.connect(self._on_launch_log_line)
      self.start_node_thread.finished_signal.connect(self._on_launch_finished)
      self.is_running = True
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
      self.conn_params["serverCommandPort"] = int(port_text)
      self.Log('info','setting command port '+str(self.conn_params["serverCommandPort"]))
    except Exception as e:
      self.conn_params["serverCommandPort"] = None
      self.error_pass = False
      self._log_exception(f'Invalid command port value: {port_text}', e)

  def get_data_port(self):
    port_text = self.textDataPort.text()
    try:
      self.conn_params["serverDataPort"] = int(port_text)
      self.Log('info','setting data port '+str(self.conn_params["serverDataPort"]))
    except Exception as e:
      self.conn_params["serverDataPort"] = None
      self.error_pass = False
      self._log_exception(f'Invalid data port value: {port_text}', e)

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
    self.name = self.textNodeName.text()
    if self.name=='':
      self.name='natnet_ros2'
      self.Log('warn','setting natnet_ros2 name to world as no input provided')
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
      self.node.call_set_parameters(node_name,self.conn_params)

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
    if (len(x_position) or len(y_position) or len(z_position))!=num_of_markers:
      self.Log('error',' Length of positions and number of detected markers are not matching, Press refresh to retry.')
    else:
      for i in range(min(40,num_of_markers)):
        eval('self.markerBox_'+str(i+1)+'.setEnabled(True)')
        eval('self.X_'+str(i+1)+'.display(x_position[i])')
        eval('self.Y_'+str(i+1)+'.display(y_position[i])')
        eval('self.Z_'+str(i+1)+'.display(z_position[i])')
      self.num_of_markers = num_of_markers
      self.x_position = x_position
      self.y_position = y_position
      self.z_position = z_position

  def yaml_dump(self):
    self.im_msg_type = self.msg_types[self.msg_type_spin.value()-1]
    empty=0
    object_names={'object_names':[]}
    if self.num_of_markers!=0:
      for i in range(min(40,self.num_of_markers)):
        if eval('self.name_'+str(i+1)+'.text()') == '': empty+=1
        else:
          object_names['object_names'].append(eval('self.name_'+str(i+1)+'.text()'))
          object_names[object_names['object_names'][i-empty]]={'marker_config':0,
                                                    'pose':
                                                    {'position':[self.x_position[i],self.y_position[i],self.z_position[i]],
                                                    'orientation':[0,0,0]}}
      self.natnet_params = object_names
      try:
        os.remove(os.path.join(self.config_file))
      except OSError as e:
        self.Log('warn', f' Could not remove existing config file: {e}')
        self.node.get_logger().warn(f'Could not remove existing config file: {e}')

      with open(self.config_file,'w') as f:
        yaml.dump({self.name: {'ros__parameters':self.natnet_params}},f,indent=2,default_flow_style=False)
        f.close()
    else:
      self.Log('error','Number of markers are not recieved. Something went wrong.')

  def call_MarkerPoses_srv(self):
    self.set_conn_params('marker_poses_server')
    if self.error_pass:
      try:
        res = self.node.request_markerposes()
        self.set_lcds(res.num_of_markers,res.x_position,res.y_position,res.z_position)
      except RuntimeError as e:
        self.Log('error','Service call failed: '+str(e))
        self.node.get_logger().error('Service call failed: '+str(e))
      except Exception as e:
        self._log_exception('Unexpected error during marker pose service call', e)

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
