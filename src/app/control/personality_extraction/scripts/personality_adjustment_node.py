#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import configparser
import os
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import threading
import time
import math

PARAMS = {
    'ax_max': {'min': 0.9904, 'max': 3.004},
    'ay_max':  {'min': 0.8742, 'max': 3.298},
    'jx_max':  {'min': 1.490, 'max': 4.288},
    'jx_min': {'min': -1.374, 'max': -4.026},
    'jy_max':   {'min': 1.2267, 'max': 4.889},
    'thw': {'min': 4.889, 'max': 0.5}
}

# Weight target only on "d_w"
WEIGHT_PARAMS = {
    'd_w': {'min': 200.0, 'max': 10000.0}
}
NEGATIVE_AGGRESIVE_PARAMS = ['jx_min', 'thw', 'd_w']

# Path to personality.ini file
personality_ini_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../config/personality.ini'))
weight_ini_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../acados_scc_control/scripts/MPC/weights.ini'))
data_lock = threading.Lock()

# Parameters
personality_apply_target_controller = 'acados_scc_control'
apply_personality_pattern = True

# Current personality
current_personality = 50
current_personality_values = {}
last_updated_personality_values = {}

internal_update = False

def read_ini(ini_file_path):
    if not os.path.exists(ini_file_path):
        return None
    with open(ini_file_path, 'r', encoding='utf-8-sig') as f:
        ini_data = f.read()
    if ini_data.startswith('\ufeff'):
        ini_data = ini_data.lstrip('\ufeff')
    config = configparser.ConfigParser()
    config.read_string(ini_data)
    return config

def write_ini(config, ini_file_path):
    global internal_update
    internal_update = True
    with open(ini_file_path, 'w', encoding='utf-8-sig') as configfile:
        config.write(configfile)
    rospy.loginfo("ini file has been updated successfully.: %s", ini_file_path)

def read_personality_from_ini():
    config = read_ini(personality_ini_path)
    if config is None or 'PersonalityExtraction' not in config:
        rospy.logerr("No 'PersonalityExtraction' section. Default=50.")
        return 50, {}

    try:
        actual_values = {param: float(config.get('PersonalityExtraction', param)) for param in PARAMS}
    except (configparser.NoOptionError, ValueError):
        rospy.logerr("Error reading parameters. Default=50.")
        return 50, {}

    numerator = 0.0
    denominator = 0.0
    # Determine personality value from "jy_max"
    min_of_jy_max = PARAMS['jy_max']['min']
    max_of_jy_max = PARAMS['jy_max']['max']
    a_i = (max_of_jy_max - min_of_jy_max) / 100.0
    b_i = min_of_jy_max
    actual = actual_values['jy_max']
    numerator += a_i * (actual - b_i)
    denominator += a_i**2

    if denominator == 0:
        rospy.logerr("Denominator=0. Default=50.")
        return 50, actual_values

    estimated_personality = numerator / denominator
    estimated_personality = max(0, min(estimated_personality, 100))

    return estimated_personality, actual_values

# Update ini file with input values
def update_control_ini(personality_values, slider_value):
    config = read_ini(personality_ini_path)
    if config is None:
        config = configparser.ConfigParser()
    if 'PersonalityExtraction' not in config.sections():
        config.add_section('PersonalityExtraction')

    config.set('PersonalityExtraction', 'value', str(slider_value))

    for param, value in personality_values.items():
        config.set('PersonalityExtraction', param, str(value))

    write_ini(config, personality_ini_path)
    
def update_controller_ini(personality_values, personality_weight_values):
    config = read_ini(weight_ini_path)
    if config is None:
        config = configparser.ConfigParser()
        
    if 'constraints' not in config.sections():
        config.add_section('constraints')
    if 'weights_e' not in config.sections():
        config.add_section('weights')

    # Update constraints
    for param, value in personality_values.items():
        config.set('constraints', param, str(value))
    
    # Update weights
    for param, value in personality_weight_values.items():
        config.set('weights', param, str(value))

    write_ini(config, weight_ini_path)

def update_bar_plot(ax, bars, personality_values, fig):
    # Clear previous texts
    for txt in ax.texts:
        txt.remove()

    # Update bars and their text labels
    bars[0].set_width(personality_values['ax_max'])
    bars[1].set_width(abs(personality_values['jx_min']))
    bars[2].set_width(personality_values['jx_max'])
    bars[3].set_width(personality_values['ay_max'])
    bars[4].set_width(personality_values['jy_max'])
    bars[5].set_width(personality_values['thw'])

    # Update text values
    for idx, bar in enumerate(bars):
        width = bar.get_width()
        ax.text(0.6, bar.get_y() + bar.get_height()/2, 
                f'{width:.2f}', 
                va='center',
                fontsize=8,
                color='white',
                fontweight='bold')

    fig.canvas.draw_idle()

# Update with slider change
def update(val):
    with data_lock:
        slider_value = slider.val
        personality_values = {}
        personality_weight_values = {}
        
        # Calculate Personality Value each parameter: "ax_max", "jx_max", "jx_min", "ay_max", "jy_max", "thw"
        for param, limits in PARAMS.items():
            # Get min and max value of each parameter
            min_value = limits['min']
            max_value = limits['max']

            # Apply personality pattern or predefined value
            if apply_personality_pattern and param != 'thw':
                # Calculate Personality Value by mapping (min_personality_value to last_updated_personality_values) to (min_value to slider_value)
                personal_value = (last_updated_personality_values[param] - min_value) * 100.0 / last_updated_personality_values['value'] + min_value
            else:
                personal_value = limits['max']

            # Invert personality parameter if it is NEGATIVE_AGGRESIVE_PARAMS
            if param in NEGATIVE_AGGRESIVE_PARAMS:
                min_value = -min_value
                max_value = -max_value
                personal_value = -personal_value
            
            # Calculate new personality value from slider value
            calc_val = min_value + (personal_value - min_value) * (slider_value / 100.0)
            
            # Saturation from limits
            if calc_val > max_value:
                calc_val = max_value
            elif calc_val < min_value:
                calc_val = min_value
            
            # Revert personality parameter if it is NEGATIVE_AGGRESIVE_PARAMS
            if param in NEGATIVE_AGGRESIVE_PARAMS:
                calc_val = -calc_val

            # Set personality value
            personality_values[param] = calc_val
            
            print ("param: ", param, " value: ", calc_val)
            
        # Determine Weight from Personality Value
        for param, limits in WEIGHT_PARAMS.items(): # However, only "d_w" is used for weight
            min_value = limits['min']
            max_value = limits['max']
            
            # Invert personality parameter if it is NEGATIVE_AGGRESIVE_PARAMS
            if param in NEGATIVE_AGGRESIVE_PARAMS:
                slider_value = 100 - slider_value
            
            # Calculate personality weight value with exponential scale
            min_log_value = math.log(min_value)
            max_log_value = math.log(max_value)
            personality_weight_log_value = min_log_value + (max_log_value - min_log_value) * (slider_value / 100.0)
            personality_weight_value = math.exp(personality_weight_log_value)
            
            # Saturation from limits
            if personality_weight_value > max_value:
                personality_weight_value = max_value
            elif personality_weight_value < min_value:
                personality_weight_value = min_value
            
            # Revert personality NEGATIVE_AGGRESIVE_PARAMS
            if param in NEGATIVE_AGGRESIVE_PARAMS:
                slider_value = 100 - slider_value
            
            # Set personality weight value
            personality_weight_values[param] = personality_weight_value
            print (f"param: {param}, value: {personality_weight_value}")

        # Update bar plot
        update_bar_plot(ax, bars, personality_values, fig)

        # Slider 변경 시 ini 기록 (내부 업데이트)
        update_control_ini(personality_values, slider_value)
        
        # Update controller ini file
        update_controller_ini(personality_values, personality_weight_values)

        global current_personality_values
        current_personality_values = personality_values.copy()
        pub.publish(slider_value)

class IniFileEventHandler(FileSystemEventHandler):
    def __init__(self):
        self.last_modified = 0
        self.debounce_seconds = 0.3  # 300ms debounce time

    # Event handler for external modification of ini file
    def on_modified(self, event):
        global internal_update
        current_time = time.time()
        
        # Debounce check
        if current_time - self.last_modified < self.debounce_seconds:
            return
        self.last_modified = current_time
        
        # Check if this is the file we're interested in
        if event.src_path == personality_ini_path:
            if internal_update:  # Internal update event, ignore
                internal_update = False
                return

            time.sleep(0.1)  # Wait for the file to be fully written

            # Read personality from ini file
            estimated_personality, actual_values = read_personality_from_ini()
            global current_personality_values, last_updated_personality_values
            current_personality_values = actual_values.copy()
            last_updated_personality_values = actual_values.copy()
            last_updated_personality_values['value'] = estimated_personality
            
            # Calculate Weight fron Personality Value
            
            # UI update
            with data_lock:
                slider.eventson = False
                slider.set_val(estimated_personality)  # Update slider
                slider.eventson = True

                # Update bar plot
                update_bar_plot(ax, bars, current_personality_values, fig)

def personality_adjustment_node():
    global pub
    rospy.init_node('personality_adjustment', anonymous=False)
    
    # Read parameters from launch file
    global personality_apply_target_controller, apply_personality_pattern
    personality_apply_target_controller = rospy.get_param('personality_adjustment/personality_apply_target_controller', 'acados_scc_control')
    apply_personality_pattern = rospy.get_param('personality_adjustment/apply_personality_pattern', True)
    
    # Read weight from ini file
    global weight_ini_path
    weight_ini_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../' + personality_apply_target_controller + '/scripts/MPC/weights.ini'))
    weight_config = read_ini(weight_ini_path)
    if weight_config is None:
        rospy.logerr("weight ini file not found: %s", weight_ini_path)
        return

    # Publish slider value
    pub = rospy.Publisher('slider_value', Float32, queue_size=10)
    rospy.loginfo("personality_adjustment node initialized.")

if __name__ == '__main__':
    try:
        # Initialize personality_adjustment_node ros node
        personality_adjustment_node()

        # Initialize ini file observer
        event_handler = IniFileEventHandler()
        observer = Observer()
        observer.schedule(event_handler, path=os.path.dirname(personality_ini_path), recursive=False)
        observer.start()

        # Read personality from ini file
        current_personality, current_personality_values = read_personality_from_ini()
        last_updated_personality_values = current_personality_values.copy()
        last_updated_personality_values['value'] = current_personality
        # Initialize matplotlib figure
        fig, ax = plt.subplots()
        plt.subplots_adjust(left=0.25, bottom=0.25)

        # Set parameter labels and initial values
        param_labels = ['Max Long Accel [m/s^2]', 'Max Long Jerk [m/s^3]', 'Min Long Jerk [m/s^3]', 'Max Lat Accel [m/s^2]', 'Max Lat Jerk [m/s^3]', 'Time Headway [s]']
        initial_values = [
            current_personality_values.get('ax_max', 0),
            current_personality_values.get('jx_max', 0),
            abs(current_personality_values.get('jx_min', 0)),
            current_personality_values.get('ay_max', 0),
            current_personality_values.get('jy_max', 0),
            current_personality_values.get('thw', 0),
        ]
        bar_colors = ['blue', 'orange', 'green', 'red', 'purple', 'gray']
        bars = plt.barh(param_labels, initial_values, color=bar_colors)
        
        # Add initial text values on bars
        for bar in bars:
            width = bar.get_width()
            ax.text(0.6, bar.get_y() + bar.get_height()/2, 
                   f'{width:.2f}', 
                   va='center',
                   fontsize=8,
                   color='white',
                   fontweight='bold')
                   
        ax.set_xlim(0.5, 4.5)

        # Initialize slider
        slider_ax = plt.axes([0.25, 0.05, 0.65, 0.03])
        slider = Slider(slider_ax, 'Driving Personality', 0, 100, valinit=current_personality)

        # Event when slider is released
        slider.ax.figure.canvas.mpl_connect('button_release_event', update)

        # Show figure. Node will be terminated when figure is closed.
        plt.show()

        # Stop observer
        observer.stop()
        observer.join()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        observer.stop()
        observer.join()
