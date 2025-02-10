#!/usr/bin/env python3

import sys
import os
import configparser
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QComboBox, QLabel, QCheckBox, QPushButton,
                           QScrollArea, QGroupBox, QMessageBox, QLineEdit,
                           QMenuBar, QMenu, QAction, QTabWidget)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
import subprocess
import re
import threading
import time
import signal

class IniEditorThread(threading.Thread):
    def __init__(self, ini_path):
        super().__init__()
        self.ini_path = ini_path
        self.daemon = True

    def run(self):
        try:
            subprocess.run(['gedit', self.ini_path])
        except Exception as e:
            print(f"Error opening ini file: {e}")

class ProcessMonitor(threading.Thread):
    def __init__(self, command, section, led, parent=None):
        super().__init__()
        self.command = command
        self.section = section
        self.led = led
        self.parent = parent
        self.daemon = True
        self.process = None
        self.should_stop = False
        self.terminal_process = None
        self.pgid = None  # Store process group ID

    def run(self):
        try:
            # Start process with pipe for stderr
            self.process = subprocess.Popen(
                ['bash', '-c', self.command],
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )
            
            # Monitor stderr
            while not self.should_stop:
                line = self.process.stderr.readline()
                if not line and self.process.poll() is not None:
                    break
                if line:
                    # Check for common error patterns
                    if any(error_pattern in line.lower() for error_pattern in 
                          ['error', 'exception', 'failed', 'fatal', 'killed']):
                        self.led.set_status('error')
                        break

            # Check return code
            return_code = self.process.wait()
            if return_code != 0 and not self.should_stop:
                self.led.set_status('error')
                
        except Exception as e:
            self.led.set_status('error')
            if self.parent:
                QMessageBox.critical(self.parent, 'Error', f'Error occurred while running process: {str(e)}')

    def stop(self):
        self.should_stop = True
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
        
        if self.terminal_process and self.pgid:
            try:
                # Kill only the specific process group
                os.killpg(self.pgid, signal.SIGTERM)
                time.sleep(0.1)  # Give some time for the process to terminate
                os.killpg(self.pgid, signal.SIGKILL)  # Force kill if still running
            except ProcessLookupError:
                pass  # Process group already terminated
            except Exception as e:
                print(f"Error killing process group: {e}")

class LEDIndicator(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(16, 16)
        self.setStyleSheet("""
            QLabel {
                border: 2px solid gray;
                border-radius: 8px;
                background-color: #404040;
            }
        """)
        
    def set_status(self, status):
        if status == 'running':
            color = '#00FF00'  # Green
        elif status == 'stopped':
            color = '#006400'  # Dark Green
        elif status == 'error':
            color = '#FF0000'  # Red
        else:
            color = '#404040'  # Gray
            
        self.setStyleSheet(f"""
            QLabel {{
                border: 2px solid gray;
                border-radius: 8px;
                background-color: {color};
            }}
        """)

class LaunchManagerGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        # Get the absolute path of the launch_manager directory
        self.launch_manager_dir = os.path.dirname(os.path.abspath(__file__))
        
        self.running_processes = {}  # Store running processes and their monitors
        self.init_ui()
        self.load_config()
        
        # Start status update timer
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)  # Update every second
        
    def init_ui(self):
        self.setWindowTitle('AutoHYU Launch Manager')
        self.setGeometry(100, 100, 600, 800)
        
        # Create menu bar
        menubar = self.menuBar()
        file_menu = menubar.addMenu('File')
        
        # Add menu actions
        open_ini_action = QAction('Open Config', self)
        open_ini_action.triggered.connect(self.open_ini_file)
        file_menu.addAction(open_ini_action)
        
        reload_ini_action = QAction('Load Config', self)
        reload_ini_action.triggered.connect(self.reload_config)
        file_menu.addAction(reload_ini_action)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        
        # Working directory display
        path_group = QGroupBox('Working Directory')
        path_layout = QHBoxLayout()
        path_label = QLabel('Path:')
        path_label.setFixedWidth(50)
        
        # Create text box for path
        self.path_edit = QLineEdit()
        self.path_edit.editingFinished.connect(self.update_working_directory)
        self.path_edit.setFocusPolicy(Qt.NoFocus)
        
        # Create Apply button
        apply_btn = QPushButton('Apply')
        apply_btn.setFixedWidth(80)
        apply_btn.clicked.connect(self.update_working_directory)
        
        path_layout.addWidget(path_label)
        path_layout.addWidget(self.path_edit)
        path_layout.addWidget(apply_btn)
        path_group.setLayout(path_layout)
        main_layout.addWidget(path_group)
        
        # Environment settings group
        self.env_group = QGroupBox('Environment Settings')
        self.env_group.setObjectName('env_group')
        self.env_layout = QVBoxLayout()
        self.env_layout.setSpacing(5)
        
        # Will be populated by load_config()
        self.env_combos = {}
        
        self.env_group.setLayout(self.env_layout)
        main_layout.addWidget(self.env_group)
        
        # Launch options group
        launch_group = QGroupBox('Launch Options')
        launch_layout = QVBoxLayout()
        launch_layout.setSpacing(5)
        
        # Create a scroll area for launch options
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.launch_checkboxes_layout = QVBoxLayout(scroll_widget)
        self.launch_checkboxes_layout.setSpacing(5)
        scroll.setWidget(scroll_widget)
        
        # Will be populated by load_config()
        self.launch_checkboxes = {}
        
        launch_layout.addWidget(scroll)
        launch_group.setLayout(launch_layout)
        main_layout.addWidget(launch_group)
        
        # Control buttons
        button_layout = QHBoxLayout()
        button_layout.setSpacing(10)
        
        # Create buttons with fixed width
        select_all_btn = QPushButton('Select All')
        select_all_btn.setFixedWidth(120)
        select_all_btn.clicked.connect(self.select_all)
        button_layout.addWidget(select_all_btn)
        
        deselect_all_btn = QPushButton('Deselect All')
        deselect_all_btn.setFixedWidth(120)
        deselect_all_btn.clicked.connect(self.deselect_all)
        button_layout.addWidget(deselect_all_btn)
        
        launch_btn = QPushButton('Launch Selected')
        launch_btn.setFixedWidth(120)
        launch_btn.clicked.connect(self.launch_selected)
        button_layout.addWidget(launch_btn)
        
        kill_btn = QPushButton('Kill Selected')
        kill_btn.setFixedWidth(120)
        kill_btn.clicked.connect(self.kill_selected)
        button_layout.addWidget(kill_btn)
        
        main_layout.addLayout(button_layout)

    def open_ini_file(self):
        config_path = os.path.join(self.launch_manager_dir, 'launch_config.ini')
        editor_thread = IniEditorThread(config_path)
        editor_thread.start()

    def reload_config(self):
        # Check for running processes
        if self.running_processes:
            reply = QMessageBox.question(self, 'Running Programs',
                                       'There are running programs. Do you want to terminate all and reload the configuration?',
                                       QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                # Kill all running processes
                for section in list(self.running_processes.keys()):
                    self.kill_single(section)
            else:
                return
        
        # Store current window geometry
        geometry = self.geometry()
        
        # Clear environment layout
        while self.env_layout.count():
            item = self.env_layout.takeAt(0)
            if item.layout():
                while item.layout().count():
                    subitem = item.layout().takeAt(0)
                    if subitem.widget():
                        subitem.widget().deleteLater()
                item.layout().deleteLater()
            elif item.widget():
                item.widget().deleteLater()
                
        # Clear launch checkboxes layout
        while self.launch_checkboxes_layout.count():
            item = self.launch_checkboxes_layout.takeAt(0)
            if item.layout():
                while item.layout().count():
                    subitem = item.layout().takeAt(0)
                    if subitem.widget():
                        subitem.widget().deleteLater()
                item.layout().deleteLater()
            elif item.widget():
                item.widget().deleteLater()
        
        # Clear dictionaries
        self.env_combos.clear()
        self.launch_checkboxes.clear()
        
        # Reload configuration
        self.load_config()
        
        # Restore window geometry
        self.setGeometry(geometry)
        
        QMessageBox.information(self, 'Success', 'Configuration file has been successfully reloaded.')

    def get_default_commands(self, section):
        if 'default_commands' not in self.config[section]:
            return []
            
        command_indices = [int(x.strip()) for x in self.config[section]['default_commands'].split(',')]
        commands = []
        
        for idx in command_indices:
            command_key = f'command_{idx}'
            if command_key in self.config['Default Command']:
                commands.append(self.config['Default Command'][command_key])
                
        return commands

    def launch_single(self, section):
        if section in self.launch_checkboxes:
            command = self.launch_checkboxes[section]['command']
            
            # Check if the process is already running
            if section in self.running_processes and self.running_processes[section].is_alive():
                QMessageBox.warning(self, 'Warning', 
                                 f'{section} is already running.')
                return
            
            # Get environment variables
            env_vars = {
                key: combo.currentText()
                for key, combo in self.env_combos.items()
            }
            
            # Replace environment variables in command
            for key, value in env_vars.items():
                command = command.replace(f'${{{key}}}', value)
            
            # Get default commands for this section
            setup_commands = [
                "conda deactivate 2>/dev/null || true",  # Deactivate conda if it exists
                "export PATH=/usr/bin:/usr/local/bin:$PATH",  # Ensure system paths are first
                "unset PYTHONPATH",  # Unset PYTHONPATH to avoid conflicts
                "unset CONDA_PREFIX",  # Unset conda environment
                "unset CONDA_DEFAULT_ENV",  # Unset conda environment
                "unset CONDA_EXE",  # Unset conda executable
            ]
            
            # Add default commands based on section's configuration
            setup_commands.extend(self.get_default_commands(section))
            
            setup_commands.append(command)
            full_command = " && ".join(setup_commands)
            
            try:
                # Create terminal command with error redirection
                term_command = f'{full_command} 2> >(tee /tmp/launch_manager_{section}.err)'
                
                # Create new terminal with clean environment
                env = os.environ.copy()
                # Remove conda and Python related environment variables
                env_vars_to_remove = [
                    'PYTHONPATH',
                    'CONDA_PREFIX',
                    'CONDA_DEFAULT_ENV',
                    'CONDA_EXE',
                    'CONDA_SHLVL',
                    'CONDA_PYTHON_EXE',
                    'CONDA_PROMPT_MODIFIER'
                ]
                for var in env_vars_to_remove:
                    env.pop(var, None)
                
                # Set clean Python environment
                env['PATH'] = '/usr/bin:/usr/local/bin:' + env.get('PATH', '')
                
                # Set working directory from config
                cwd = self.working_directory
                
                def create_process_group():
                    # Create a new session ID for the child process
                    os.setsid()
                
                try:
                    # Try xterm if gnome-terminal is not available
                    print("Try xterm")
                    terminal_process = subprocess.Popen(
                        ['xterm', '-T', section, '-e', f'cd {cwd} && {term_command}; exec bash'],
                        env=env,
                        preexec_fn=create_process_group
                    )
                    
                except FileNotFoundError:
                    try:
                        # Fallback to x-terminal-emulator
                        print("Try x-terminal-emulator")
                        terminal_process = subprocess.Popen(
                            ['x-terminal-emulator', '-T', section, '-e', f'cd {cwd} && {term_command}; exec bash'],
                            env=env,
                            preexec_fn=create_process_group
                        )
                    except FileNotFoundError:
                        print("Try gnome-terminal")
                        # Try gnome-terminal first with process group
                        terminal_process = subprocess.Popen(
                            ['gnome-terminal', '--working-directory', cwd, '--title', section,
                            '--', 'bash', '-c', f'{term_command}; exec bash'],
                            env=env,
                            preexec_fn=create_process_group
                        )
                
                # Store the process group ID
                monitor = ProcessMonitor(
                    command=f'tail -f /tmp/launch_manager_{section}.err',
                    section=section,
                    led=self.launch_checkboxes[section]['led'],
                    parent=self
                )
                monitor.terminal_process = terminal_process
                monitor.pgid = os.getpgid(terminal_process.pid)  # Store the process group ID
                self.running_processes[section] = monitor
                monitor.start()
                
                # Set initial status
                self.launch_checkboxes[section]['led'].set_status('running')
                
            except Exception as e:
                self.launch_checkboxes[section]['led'].set_status('error')
                QMessageBox.critical(self, 'Error', f'Error occurred while running process: {str(e)}')
    
    def select_all(self):
        for item in self.launch_checkboxes.values():
            item['checkbox'].setChecked(True)
    
    def deselect_all(self):
        for item in self.launch_checkboxes.values():
            item['checkbox'].setChecked(False)
    
    def kill_single(self, section):
        if section in self.launch_checkboxes:
            if section in self.running_processes:
                try:
                    self.running_processes[section].stop()
                except Exception as e:
                    print(f"Error stopping process {section}: {e}")
                finally:
                    del self.running_processes[section]
            
            # Try to kill any remaining ROS nodes for this section
            try:
                command = self.launch_checkboxes[section]['command']
                # Extract launch/node name from command
                match = re.search(r'ros(?:launch|run)\s+(?:--screen\s+)?(?:[\w/]+/)?(\w+)(?:\.launch)?', command)
                if match:
                    target = match.group(1)
                    # Try to kill any matching ROS nodes
                    subprocess.run(['rosnode', 'kill', f'/{target}'], stderr=subprocess.DEVNULL)
            except:
                pass
            
            # Update LED status to stopped
            self.launch_checkboxes[section]['led'].set_status('stopped')
    
    def kill_selected(self):
        for section, item in self.launch_checkboxes.items():
            if item['checkbox'].isChecked():
                self.kill_single(section)
    
    def launch_selected(self):
        for section, item in self.launch_checkboxes.items():
            if item['checkbox'].isChecked():
                self.launch_single(section)

    def closeEvent(self, event):
        # Kill all running processes when closing the application
        for section in list(self.running_processes.keys()):
            self.kill_single(section)
        event.accept()

    def update_working_directory(self):
        new_path = self.path_edit.text()
        # Expand ~ to home directory if present
        new_path = os.path.expanduser(new_path)
        
        # Verify the path exists
        if not os.path.exists(new_path):
            QMessageBox.warning(self, 'Warning', 'The specified path does not exist.')
            # Reset to current working directory
            self.path_edit.setText(self.working_directory)
            return
            
        # Update working directory
        self.working_directory = new_path
        
        # Update config file
        config_path = os.path.join(self.launch_manager_dir, 'launch_config.ini')
        try:
            # Replace ~ with actual home directory in stored path
            relative_path = self.working_directory.replace(os.path.expanduser('~'), '~')
            self.config['Default Path']['working_directory'] = relative_path
            
            with open(config_path, 'w', encoding='utf-8') as f:
                self.config.write(f)
                
            QMessageBox.information(self, 'Success', 'Working directory has been successfully updated.')
        except Exception as e:
            QMessageBox.critical(self, 'Error', f'Error occurred while updating config file: {str(e)}')
            # Reset to current working directory
            self.path_edit.setText(self.working_directory)

    def load_config(self):
        # Use absolute path based on script location
        config_path = os.path.join(self.launch_manager_dir, 'launch_config.ini')
        self.config = configparser.ConfigParser()
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                self.config.read_file(f)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load config file: {e}")
            sys.exit(1)
            
        print(f"Config sections: {self.config.sections()}")
        
        # Load default working directory
        if 'Default Path' in self.config:
            working_dir = self.config['Default Path']['working_directory']
            # Expand ~ to home directory
            self.working_directory = os.path.expanduser(working_dir)
        else:
            # Fallback to project root directory
            self.working_directory = os.path.dirname(self.launch_manager_dir)
        
        # Update path edit box
        self.path_edit.setText(self.working_directory)
        
        # Load environment settings
        if 'Environment' in self.config:
            print(f"Environment options: {dict(self.config['Environment'])}")
            for key in self.config['Environment']:
                key_upper = key.upper()
                # Create label and options
                label = key_upper
                options = [opt.strip() for opt in self.config['Environment'][key].split(',')]
                print(f"Adding {label} with options: {options}")
                
                row_layout = QHBoxLayout()
                # Add label with fixed width
                label_widget = QLabel(f'{label}:')
                label_widget.setFixedWidth(50)
                row_layout.addWidget(label_widget)
                
                combo = QComboBox()
                combo.addItems(options)
                row_layout.addWidget(combo)
                
                self.env_combos[label] = combo
                self.env_layout.addLayout(row_layout)
        else:
            print("No Environment section found in config")
        
        # Skip special sections when loading launch options
        skip_sections = ['Environment', 'Default Path', 'Default Command']
        
        # Load launch options
        for section in self.config.sections():
            if section not in skip_sections:
                try:
                    # Create horizontal layout for checkbox and buttons
                    item_layout = QHBoxLayout()
                    
                    # Create LED indicator (leftmost)
                    led = LEDIndicator()
                    item_layout.addWidget(led)
                    
                    # Add spacing after LED
                    item_layout.addSpacing(5)
                    
                    # Create checkbox
                    checkbox = QCheckBox(section)
                    checkbox.setFixedWidth(200)
                    item_layout.addWidget(checkbox)
                    
                    # Add stretch to push buttons to the right
                    item_layout.addStretch()
                    
                    # Create Launch button
                    launch_btn = QPushButton('Launch')
                    launch_btn.setFixedWidth(80)
                    launch_btn.clicked.connect(lambda checked, s=section: self.launch_single(s))
                    item_layout.addWidget(launch_btn)
                    
                    # Add spacing between buttons
                    item_layout.addSpacing(5)
                    
                    # Create Kill button
                    kill_btn = QPushButton('Kill')
                    kill_btn.setFixedWidth(80)
                    kill_btn.clicked.connect(lambda checked, s=section: self.kill_single(s))
                    item_layout.addWidget(kill_btn)
                    
                    # Store checkbox, LED, and command
                    self.launch_checkboxes[section] = {
                        'checkbox': checkbox,
                        'led': led,
                        'command': self.config[section]['command'],
                        'launch_btn': launch_btn,
                        'kill_btn': kill_btn
                    }
                    
                    # Add to layout
                    self.launch_checkboxes_layout.addLayout(item_layout)
                except KeyError as e:
                    print(f"Warning: Skipping section {section} due to missing key: {e}")
                except Exception as e:
                    print(f"Warning: Error processing section {section}: {e}")
        
        # Update working directory display
        self.path_edit.setText(self.working_directory)
    
    def update_status(self):
        for section, item in self.launch_checkboxes.items():
            if section in self.running_processes:
                monitor = self.running_processes[section]
                
                # Check if terminal process is still alive
                terminal_alive = False
                if monitor.terminal_process:
                    try:
                        # Poll returns None if process is running, not None if process has terminated
                        terminal_alive = monitor.terminal_process.poll() is None
                    except:
                        terminal_alive = False
                
                # If terminal is not alive, clean up the process
                if not terminal_alive:
                    monitor.stop()
                    del self.running_processes[section]
                    item['led'].set_status('stopped')
                else:
                    item['led'].set_status('running')
            else:
                item['led'].set_status('stopped')

def main():
    app = QApplication(sys.argv)
    gui = LaunchManagerGUI()
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main() 