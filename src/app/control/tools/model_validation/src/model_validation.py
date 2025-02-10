#!/usr/bin/env python3

# -----------------------------------------------------------------------------
# @file           model_validation.py
# @brief          ROS-based model validation tool
#
# @authors        Seounghoon Park
#
# @date           2025-07           Created by Seounghoon Park
#                 2025-01-02        Fix bug by Seounghoon Park
# -----------------------------------------------------------------------------

# system headers
import os
import time
import configparser
import numpy as np
import pandas as pd
import casadi as cs

# ROS headers
import rospy

# algorithm header


from result_plot import ValidationPlot

# message headers
from std_msgs.msg import Float32
from autohyu_msgs.msg import VehicleState

from model.cartesian_kinematic_stm_rear import VehicleModel as KinematicSTMRear
from model.cartesian_dynamic_stm import VehicleModel as DynamicSTMCG

class ModelValidation(object):
    def __init__(self):
        rospy.init_node('model_validation', anonymous=True)
        self.loop_period = 0.01     # same as longitudinal control
        
        ## Publisher

        ## Subscriber
        self.s_vehicle_state = rospy.Subscriber(
            "app/loc/vehicle_state", VehicleState, self.callback_vehicle_state)
        self.s_command_trq   = rospy.Subscriber(
            "app/con/command_torque", Float32, self.callback_command_trq)
        self.s_command_steer = rospy.Subscriber(
            "app/con/command_steer", Float32, self.callback_command_steer)
    
        ## Input
        self.i_vehicle_state        = VehicleState()
        self.i_command_trq_nm       = Float32()
        self.i_command_steer_deg    = Float32()

        ## Output

        ## Parameters
        # vehicle params
        self.params = {}
        self.params['lf']                       = rospy.get_param('model_validation/vehicle/vehicle_front_to_cg')
        self.params['lr']                       = rospy.get_param('model_validation/vehicle/vehicle_rear_to_cg')
        self.params['wheelbase']                = self.params['lf'] + self.params['lr']
        self.params['mass']                     = rospy.get_param('model_validation/vehicle/mass')
        self.params['wheel_radius']             = rospy.get_param('model_validation/vehicle/wheel_radius')
        self.params['Af']                       = rospy.get_param('model_validation/vehicle/front_area')
        self.params['Cd']                       = rospy.get_param('model_validation/vehicle/air_drag_coeff')
        self.params['rolling_resistance']       = rospy.get_param('model_validation/vehicle/rolling_resistance')
        self.params['rho']                      = rospy.get_param('model_validation/air_density')
        self.params['g']                        = rospy.get_param('model_validation/gravity_coeff')
        self.params['inertia']                  = rospy.get_param('model_validation/inertia')

        # pacejka params
        self.params['Bf']               = rospy.get_param("model_validation/Bf")
        self.params['Cf']               = rospy.get_param("model_validation/Cf")
        self.params['Df']               = rospy.get_param("model_validation/Df")
        self.params['Ef']               = rospy.get_param("model_validation/Ef")
        self.params['Br']               = rospy.get_param("model_validation/Br")
        self.params['Cr']               = rospy.get_param("model_validation/Cr")
        self.params['Dr']               = rospy.get_param("model_validation/Dr")
        self.params['Er']               = rospy.get_param("model_validation/Er")
        
        self.params['dt']               = self.loop_period

        ## Variables
        self.model = None
        self.validation_on   = False
        self.is_vehicle_state = False
        self.is_command_trq = False
        self.is_command_steer = False
        
        # GPT : it's faster than np.append
        self.gt_trajectory  = []
        self.dr_trajectory  = []
        self.input_forces   = []
        self.input_steer    = []

        ## Utils
        self.cfg = configparser.ConfigParser()
        self.ini_dir = os.path.dirname(os.path.abspath(__file__)) + "/../../../../../../config/control_tools.ini"

        self.ProcessINI()

        if self.model_type == 0:
            self.model_class = KinematicSTMRear
        elif self.model_type == 1:
            self.model_class = DynamicSTMCG
        else:
            self.model_class = None

        


    def destroy(self):
        self.s_vehicle_state.unregister()
        self.s_command_trq.unregister()
        self.s_command_steer.unregister()



    def callback_vehicle_state(self, data):
        self.i_vehicle_state = data
        self.is_vehicle_state = True

    def callback_command_trq(self, data):
        self.i_command_trq_nm = data
        self.is_command_trq = True

    def callback_command_steer(self, data):
        self.i_command_steer_deg = data
        self.is_command_steer = True

        

    def ProcessINI(self):
        self.cfg.read(self.ini_dir)

        self.model_type          = int(self.cfg.get('Model Validation', 'model_type'))
        self.validation_on       = self.cfg.getboolean('Model Validation', 'validation_on')
        


    def Run(self):
        rate = rospy.Rate(1.0 / self.loop_period)

        while not rospy.is_shutdown():            
            if not self.is_vehicle_state:
                # print(f"{time.time():.1f}", " waiting for vehicle state")
                continue
            if not self.is_command_trq:
                # print(f"{time.time():.1f}", " waiting for command torque")
                continue
            if not self.is_command_steer:
                # print(f"{time.time():.1f}", " waiting for command steer")
                continue

            self.ProcessINI()

        
            if self.validation_on == True:
                if self.model is None:
                    self.model = self.model_class(self.i_vehicle_state, self.params)
                    print(self.model.model_name)
                    print("start logging states & prediction")
                
                # using sensor output for model validation
                force = self.i_vehicle_state.vehicle_can.longitudinal_accel*self.params['mass']


                """
                !!!!!! CHECK RAD OR DEG (PARSER DEPENDENT) !!!!!!
                """
                # steer_rad = np.deg2rad(self.i_vehicle_state.vehicle_can.steering_tire_angle)
                steer_rad = self.i_vehicle_state.vehicle_can.steering_tire_angle
                # steer_rad = self.i_vehicle_state.vehicle_can.steering_wheel_angle / 16.0
                
                x, y, yawrate = self.model.NextState(force, steer_rad, self.params)

                self.gt_trajectory.append([self.i_vehicle_state.x, self.i_vehicle_state.y])
                self.dr_trajectory.append([x, y, yawrate])
                
                # for visualization
                self.input_forces.append(force)
                self.input_steer.append(steer_rad)
                
                



            elif self.validation_on == False:
                if len(self.gt_trajectory) != 0 or len(self.dr_trajectory) != 0 :
                    print("validation end!!")

                    # plot
                    # print(self.dr_trajectory)
                    ValidationPlot(self.model.model_name, self.gt_trajectory, self.dr_trajectory, self.input_forces, self.input_steer, self.params)
                    

                    self.gt_trajectory  = []
                    self.dr_trajectory  = []
                    self.input_trq      = []
                    self.input_steer    = []
                    self.model = None
                # else:
                #     # print("waiting for ")

            rate.sleep()



def main():
    """
    main function
    """
    model_validation = ModelValidation()
    try:
        model_validation.Run()
    finally:
        if model_validation is not None:
            model_validation.destroy()


if __name__ == '__main__':
    main()

                
                

        
