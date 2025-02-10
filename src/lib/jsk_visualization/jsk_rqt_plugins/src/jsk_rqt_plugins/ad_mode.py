#!/usr/bin/env python

# Junhee Lee -- 2024.09.10

from distutils.version import LooseVersion
import os
import rospkg
import rospy
import rosservice

import python_qt_binding
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin

# from std_msgs.msg import Header
from autohyu_msgs.msg import ADModeInput

# For both qt4 and qt5
if LooseVersion(python_qt_binding.QT_BINDING_VERSION).version[0] >= 5:
    from python_qt_binding.QtWidgets import QWidget, QCheckBox, QVBoxLayout
else:
    from python_qt_binding.QtGui import QWidget, QCheckBox, QVBoxLayout


class ADModeWidget(QWidget):
    def __init__(self):
        super(ADModeWidget, self).__init__()
        rp = rospkg.RosPack()
        # ui_file = os.path.join(
        #     rp.get_path('jsk_rqt_plugins'), 'resource', 'ad_mode.ui')
        # loadUi(ui_file, self)

        self.setObjectName('ADModeUi')
        

        # Setup QCheckBox UI elements
        self.ready_check = QCheckBox('Ready', self)
        self.lateral_check = QCheckBox('Lateral Auto', self)
        self.longitudinal_check = QCheckBox('Longitudinal Auto', self)
        self.manual_check = QCheckBox('Manual', self)

        # Initialize the initial state
        self.lateral_check.setEnabled(False)
        self.longitudinal_check.setEnabled(False)

        # Setup the layout
        layout = QVBoxLayout()
        layout.addWidget(self.ready_check)
        layout.addWidget(self.lateral_check)
        layout.addWidget(self.longitudinal_check)
        layout.addWidget(self.manual_check)
        self.setLayout(layout)

        # Connect QCheckBox state changes to the update_modes method
        self.ready_check.stateChanged.connect(self.update_modes)
        self.lateral_check.stateChanged.connect(self.update_modes)
        self.longitudinal_check.stateChanged.connect(self.update_modes)
        self.manual_check.stateChanged.connect(self.update_modes)

        self.pub = rospy.Publisher(
            "hmi/ad_mode_input", ADModeInput, queue_size=10)
        self.ad_mode_msg = ADModeInput()
        # self.ad_mode_msg.header = Header()

        # Init status
        self.ad_mode_msg.ready_mode = False
        self.ad_mode_msg.lateral_mode = False
        self.ad_mode_msg.longitudinal_mode = False
        self.ad_mode_msg.manual_mode = True

    def update_modes(self):
        # Handle Manual mode first since it overrides the other modes
        if self.manual_check.isChecked():
            # If Manual mode is checked, uncheck Ready, Lateral, and Longitudinal
            self.ready_check.setChecked(False)
            self.lateral_check.setChecked(False)
            self.longitudinal_check.setChecked(False)
        else:
            # If Ready mode is checked, enable Lateral and Longitudinal checkboxes
            if self.ready_check.isChecked():
                self.lateral_check.setEnabled(True)
                self.longitudinal_check.setEnabled(True)
            else:
                # If Ready is not checked, disable Lateral and Longitudinal checkboxes
                self.lateral_check.setEnabled(False)
                self.lateral_check.setChecked(False)
                self.longitudinal_check.setEnabled(False)
                self.longitudinal_check.setChecked(False)

        # Create and publish the message
        msg = ADModeInput()
        msg.header.stamp = rospy.Time.now()
        msg.ready_mode = self.ready_check.isChecked()
        msg.lateral_mode = self.lateral_check.isChecked()
        msg.longitudinal_mode = self.longitudinal_check.isChecked()
        msg.manual_mode = self.manual_check.isChecked()

        self.pub.publish(msg)

    def __del__(self):
        self.pub.shutdown()


class ADMode(Plugin):
    def __init__(self, context):
        super(ADMode, self).__init__(context)
        self.setObjectName('ADMode')
        self._widget = ADModeWidget()
        context.add_widget(self._widget)

