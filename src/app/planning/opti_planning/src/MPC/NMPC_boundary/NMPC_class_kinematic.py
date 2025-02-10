"""
Created on Wed Jun  7 13:07:00 2023

@author: Baha Zarrouki (baha.zarrouki@tum.de)
"""

import numpy as np
import casadi as cs
import yaml
import os
import scipy
from typing import Tuple
import time
from scipy.interpolate import CubicSpline
from MPC.NMPC_boundary.NMPC_STM_acados_settings_kinematic import acados_settings      

from utils.controller_utils import *

'''
7. MPC
    - initialisation: 
        + loading MPC params
        + loading Prediciton model params 
        + loading tire model Params
        + loading constraint, model, acados_solver, costfunction_type
        + setting the initial state / constraints
    - step:
        + set current x_next_sim_disturbed (initial state / constraints)
        + set current reference trajectory
        + solve
        + get MPC solution: x0,u0
        + extract current MPC predictions
        + get SolverDebug Stats
'''

class Nonlinear_Model_Predictive_Controller:

    def __init__(
            self,
            config_path,
            controller_params,
            MPC_params
        ):
        ## --- MPC cost function params ---
        self.MPC_params = MPC_params
        self.Tp      = controller_params['Tp']     # prediction horizon [s]
        self.Ts_MPC  = controller_params['Ts_MPC'] # MPC prediction discretization step [s]
        self.N       = int(self.Tp / self.Ts_MPC)          # number of discretizaion steps MPC
        ## --- Model params ---     
        veh_params_file_MPC         = controller_params['veh_params_file_MPC']
        tire_params_file_MPC        = controller_params['tire_params_file_MPC']
        self.tire_params_full_path  = config_path+tire_params_file_MPC
        self.veh_params_full_path   = config_path+veh_params_file_MPC

        # load vehicle params
        with open(self.veh_params_full_path , 'r') as file:
            veh_params = yaml.load(file, Loader=yaml.FullLoader)
        self.m   = veh_params['m']   # vehicle mass [kg]  MASS
        self.g = 9.81  #[m/s^2]
        # Source: Dr. M. Gerdts, The single track model, Universität Bayreuth, SS 2003
        self.ro  = veh_params['ro']              # air density 
        self.S   = veh_params['S']            # frontal area
        self.Cd  = veh_params['Cd']           # drag coeff
        self.fr0 = 0.009         # friction coefficient factors
        self.fr1 = 0.002         # friction coefficient factors
        self.fr4 = 0.0003        # friction coefficient factors
        self.cW_F = -1 * -0.522  # Aero lift coefficient at front axle
        self.cW_R = -1 * -1.034  # Aero lift coefficient at rear axle


        ## --- configure MPC cost function ---
        # scaling factors
        s_lon           = self.MPC_params['s_lon']
        s_lat           = self.MPC_params['s_lat']
        s_yaw           = self.MPC_params['s_yaw']
        s_vel           = self.MPC_params['s_vel']
        s_jerk          = self.MPC_params['s_jerk']
        s_steering_rate = self.MPC_params['s_steering_rate']
        # weights
        q_lon           = self.MPC_params['q_lon']
        q_lat           = self.MPC_params['q_lat']
        q_yaw           = self.MPC_params['q_yaw']
        q_vel           = self.MPC_params['q_vel']
        r_jerk          = self.MPC_params['r_jerk']
        r_steering_rate = self.MPC_params['r_steering_rate']
        
        
        self.L1_pen = self.MPC_params['L1_pen']
        self.L2_pen = self.MPC_params['L2_pen']

        self.Q   = np.diag([q_lon* (1/s_lon**2), q_lat* (1/s_lat**2), q_yaw * (1/s_yaw**2)])   # pos_x, pos_y, yaw
        self.Qv  = np.diag([q_vel * (1/s_vel**2)])   #  v
        self.R   = np.diag([r_jerk * (1/s_jerk**2), r_steering_rate * (1/s_steering_rate**2)])
        self.kin_dyn_speed = 7.0
        X0_MPC_kin = np.array([0.0, 0.0,  0.0, 1.0, 0.0, 0.0, 0.0, 0.0] )       
        X0_MPC_dyn = np.array([0.0, 0.0,  0.0, self.kin_dyn_speed, 0.0, 0.0, 0.0, 0.0] )       

        solver_generate_C_code  = self.MPC_params['solver_generate_C_code']
        solver_build            = self.MPC_params['solver_build']
        ## --- configure MPC constraints ---
        # load acceleration limits 
        lookuptable_gg_limits_file  = self.MPC_params['lookuptable_gg_limits']
        self.combined_acc_limits    = self.MPC_params['combined_acc_limits']
        # generate interpoltaed local ax & ay limits
        self.ax_lim, self.ay_lim = calculate_velvar_latlon_acc(config_path + lookuptable_gg_limits_file)

        ## --- Load MPC formulation ---
        # self.acados_solver_dyn = acados_settings(
        #     self.Tp, self.N, X0_MPC_dyn, self.Q, self.R, "dynamic", self.L1_pen,
        #     self.L2_pen, self.ax_lim, self.ay_lim, self.combined_acc_limits,
        #     self.veh_params_full_path, self.tire_params_full_path,
        #     solver_generate_C_code=solver_generate_C_code,
        #     solver_build=solver_build
        # )

        self.acados_solver_kin = acados_settings(
            self.Tp, self.N, X0_MPC_kin, self.Q, self.R,  self.Qv, self.L1_pen,
            self.L2_pen, self.ax_lim, self.ay_lim, self.combined_acc_limits,
            self.veh_params_full_path, self.tire_params_full_path,
            solver_generate_C_code=solver_generate_C_code,
            solver_build=solver_build
        )

        self.nx     = np.size(X0_MPC_kin)
        self.stats  = np.zeros(6)
        self.pred_X = np.zeros((self.N , self.nx)) # initialize empty array with shape (0,6)
        self.kappa_pred = np.zeros((self.N+1))
        self.is_MPC_init = True


        return

    def solve(self, acados_solver, ego_trajectory, x0, boundary, objects):
        solve_time_start = time.time()      
    
        self.set_initial_state(acados_solver, x0)
    
        lb_closest_points, rb_closest_points = find_closest_points(self.pred_X,boundary[0],boundary[1] )

        target_objects = np.zeros((3, self.N , 2))
        target_objects[:, :, 0] = 100

        num_objects_to_copy = min(target_objects.shape[0], objects.shape[0])
        num_timesteps_to_copy = min(target_objects.shape[1], objects.shape[1])
        target_objects[:num_objects_to_copy, :num_timesteps_to_copy, :] = objects[:num_objects_to_copy, :num_timesteps_to_copy, :]
        # # update MPC reference
        p = np.zeros(7)
        for j in range(self.N-1):
            lb_x, lb_y, lb_yaw = lb_closest_points[j]
            rb_x, rb_y, rb_yaw = rb_closest_points[j]
            
            # Calculate slopes using yaw values
            m1 = np.tan(lb_yaw)
            m2 = np.tan(rb_yaw)

            
            # Shift the left boundary line to the right (positive direction)
            lb_x_shifted, lb_y_shifted = shift_line(lb_x, lb_y, lb_yaw, -2)  # +1 for 1m right

            # Shift the right boundary line to the left (negative direction)
            rb_x_shifted, rb_y_shifted = shift_line(rb_x, rb_y, rb_yaw, 2) # -1 for 1m left

            # Calculate intercepts using the shifted points
            c1 = lb_y_shifted - m1 * lb_x_shifted
            c2 = rb_y_shifted - m2 * rb_x_shifted
            # yL = m1*self.pred_X[j,0] + c1
            # yR = m2*self.pred_X[j,0]+ c2
            # bound = -(self.pred_X[j,1] - yL)*(self.pred_X[j,1] - yR)

            obj1x = target_objects[0, j, 0]
            obj1y = target_objects[0, j, 1]
            
            obj2x = target_objects[1, j, 0]
            obj2y = target_objects[1, j, 1]
            
            obj3x = target_objects[2, j, 0]
            obj3y = target_objects[2, j, 1]
            
            # dist1 = np.sqrt((self.pred_X[j,0]-obj1x)**2 + (self.pred_X[j,1]-obj1y)**2)
            # dist2 = np.sqrt((self.pred_X[j,0]-obj2x)**2 + (self.pred_X[j,1]-obj2y)**2) 
            # dist3 = np.sqrt((self.pred_X[j,0]-obj3x)**2 + (self.pred_X[j,1]-obj3y)**2)

            # collision_dist = 300 - (dist1 + dist2 + dist3)
            # print(obj1x, obj1y,"\t" , obj2x, obj2y,"\t" , obj3x,obj3y ,"\t")
            p = np.array([ego_trajectory[j,1], ego_trajectory[j,2], ego_trajectory[j,3], 
                          m1,c1,
                          m2,c2,
                          obj1x,obj1y,
                          obj2x,obj2y,
                          obj3x,obj3y
                          ])
            acados_solver.set(j, "p", p)  
        acados_solver.set(self.N-1,"p", p)

        p_N = np.array([ego_trajectory[self.N-1,1], ego_trajectory[self.N-1,2], ego_trajectory[self.N-1,3], 
                          lb_closest_points[self.N-1,0],lb_closest_points[self.N-1,1],
                          rb_closest_points[self.N-1,0],rb_closest_points[self.N-1,1],
                          target_objects[0, -1, 0],target_objects[0, -1, 1],
                          target_objects[1, -1, 0],target_objects[1, -1, 1],
                          target_objects[2, -1, 0],target_objects[2, -1, 1]
                        ])
        # print(ego_target_state[0], ego_target_state[1], ego_target_state[2]*180/3.14)
        acados_solver.set(self.N,"p", p_N)
        
        # solve MPC problem
        status = acados_solver.solve()
        # hpipm_status: 
            # 0: SUCCESS, // found solution satisfying accuracy tolerance
            # 1: MAX_ITER, // maximum iteration number reached
            # 2: MIN_STEP, // minimum step length reached
            # 3: NAN_SOL, // NaN in solution detected
            # 4: INCONS_EQ, // unconsistent equality constraints

        # get MPC solution
        pred_X = np.zeros((self.N , self.nx)) # initialize empty array with shape (0,6)
        if status == 0: 
            for j in range(self.N):
                x       = acados_solver.get(j,"x")
                x       = np.array(x) 
                pred_X[j, :] = x
        
        self.stats[0] = acados_solver.get_cost()


        self.stats[1] = 1000.0*acados_solver.get_stats('time_tot')
        self.stats[2] = acados_solver.get_stats('sqp_iter')
        self.stats[3] = np.max(acados_solver.get_stats('qp_iter'))
        self.stats[4] = status
        solve_time_end = time.time()
        self.stats[5] = 1000.0*(solve_time_end-solve_time_start)

        return pred_X, self.stats

    def set_initial_state(self, acados_solver, x0):
        self.x0 = x0        
        acados_solver.constraints_set(0,"lbx", self.x0)
        acados_solver.constraints_set(0,"ubx", self.x0)

        return
    
    def reset(self, acados_solver ,x0):
        acados_solver.reset()
        self.set_initial_state(acados_solver, x0)
        for i in range(self.N+1):
            acados_solver.set(i, 'x', self.x0)

    def reintialize_solver(self, vehicle_state, solver_generate_C_code=False, solver_build=False):
        # [t, x, y, yaw, speed]
        X0_MPC = np.array([0.0, 0.0, 0.0, vehicle_state.vx, 0.0, 0.0, 0.0, 0.0] )     
        # self.acados_solver_dyn = acados_settings(
        #     self.Tp, self.N, X0_MPC, self.Q, self.R, "dynamic", self.L1_pen,
        #     self.L2_pen, self.ax_lim, self.ay_lim, self.combined_acc_limits,
        #     self.veh_params_full_path, self.tire_params_full_path,
        #     solver_generate_C_code=solver_generate_C_code,
        #     solver_build=solver_build
        # )

        self.acados_solver_kin = acados_settings(
            self.Tp, self.N, X0_MPC, self.Q, self.R,  self.Qv, self.L1_pen,
            self.L2_pen, self.ax_lim, self.ay_lim, self.combined_acc_limits,
            self.veh_params_full_path, self.tire_params_full_path,
            solver_generate_C_code=solver_generate_C_code,
            solver_build=solver_build
        )

        return
    

    def calculate_optimal(self, ref_boundary, ego_trajectory, vehicle_state, MPC_params, objects):

        if self.is_MPC_init:
            X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , vehicle_state.vy, vehicle_state.yaw_vel, 0.0, 0.0]) 
            print("MPC get init")
        else:
            X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , vehicle_state.vy, vehicle_state.yaw_vel, self.prev_delta , self.prev_acc ])  

        # if(vehicle_state.vx > self.kin_dyn_speed):
        #     acados_solver = self.acados_solver_dyn
        # else:
        acados_solver = self.acados_solver_kin
        # self.param_update(acados_solver,MPC_params)

        self.pred_X, MPC_stats = self.solve(acados_solver, ego_trajectory, X0, ref_boundary, objects)
        if MPC_stats[4] != 0:
            print("acados returned status {}".format(MPC_stats[4]), "-> reintialize_solver")
            # self.reset(acados_solver, X0) 
            self.reintialize_solver(vehicle_state)
            self.pred_X, MPC_stats = self.solve(acados_solver, ego_trajectory, X0, ref_boundary, objects)
            print("MPC_STATE",MPC_stats)
            self.is_MPC_init = True
        else:
            self.is_MPC_init = False

        time = np.linspace(0, (self.N - 1) * self.Ts_MPC, self.N) # fill t

        tdelta = CubicSpline(time, self.pred_X[:, 6], extrapolate=True)
        tacc = CubicSpline(time, self.pred_X[:, 7], extrapolate=True)
        self.prev_delta = tdelta(0.02)
        self.prev_acc = tacc(0.02)

        Faero  = 0.5 * self.ro * self.S * self.Cd * vehicle_state.vx**2 
        Fslope = self.m*self.g*math.sin(-vehicle_state.pitch) 
        target_torque = (tacc(0.1)*self.m + Fslope + Faero)*(0.41)
        if vehicle_state.vx > 1.0:
            target_input = [np.rad2deg(tdelta(0.1)),target_torque]
        else:
            target_input = [np.rad2deg(self.pred_X[1,6]),self.pred_X[1,7]*(2300.0)*(0.41)]
        # self.prev_delta = pred_X[1,6]
        # self.prev_acc = pred_X[1,7]
        # target_input = [np.rad2deg(pred_X[1,6]),pred_X[1,7]*(2300.0)*(0.321)]
        return self.pred_X, target_input, MPC_stats


    def param_update(self,acados_solver, MPC_params):
        s_lon           = MPC_params['s_lon']
        s_lat           = MPC_params['s_lat']
        s_yaw           = MPC_params['s_yaw']
        s_vel           = MPC_params['s_vel']
        s_jerk          = MPC_params['s_jerk']
        s_steering_rate = MPC_params['s_steering_rate']
        # weights
        q_lon           = MPC_params['q_lon']
        q_lat           = MPC_params['q_lat']
        q_yaw           = MPC_params['q_yaw']
        q_vel           = MPC_params['q_vel']
        r_jerk          = MPC_params['r_jerk']
        r_steering_rate = MPC_params['r_steering_rate']
        
        self.Q   = np.diag([q_lon* (1/s_lon**2), q_lat* (1/s_lat**2), q_yaw * (1/s_yaw**2), q_vel * (1/s_vel**2), 0, 0, 0, 0])   # pos_x, pos_y, yaw, v
        self.R   = np.diag([r_jerk * (1/s_jerk**2), r_steering_rate * (1/s_steering_rate**2)])
        self.Qe  = self.Q      # np.diag([10.0, 20.0, 10.0, 1.0])   # terminal weight

        # W = scipy.linalg.block_diag(self.Q, self.R)
        # # print("W\n", W)
        # We = self.Qe

        # # update cost function weights
        # for i in range(self.N):
        #     acados_solver.cost_set(i, 'W', W)
        # acados_solver.cost_set(self.N, 'W', We)


import pandas as pd
# calculate interpolated lateral and longitudinal acceleration limits based on current velocity 
def calculate_velvar_latlon_acc(lookuptable_gg_limits_file):
    # Read the lookup table from a CSV file
    # table = np.genfromtxt(lookuptable_gg_limits_file, delimiter=',', skip_header=1)
    data = pd.read_csv(lookuptable_gg_limits_file)
    # Extract the velocity, max lateral acceleration, and max longitudinal acceleration data
    velocity = np.array(data['vel_max_mps'])
    ax_max_mps2 = np.array(data['ax_max_mps2'])
    ay_max_mps2 = np.array(data['ay_max_mps2'])

    # Create interpolation functions for max lateral acceleration and max longitudinal acceleration using casadi.interpolant
    ax_max_interpolant = cs.interpolant('ax_max_interpolant', 'linear', [velocity], ax_max_mps2)
    ay_max_interpolant = cs.interpolant('ay_max_interpolant', 'linear', [velocity], ay_max_mps2)

    return ax_max_interpolant, ay_max_interpolant



def calculate_yaw(points):
    # Calculate yaw based on the direction between consecutive points
    yaws = np.arctan2(np.diff(points[:, 1]), np.diff(points[:, 0]))
    yaws = np.append(yaws, yaws[-1])  # Append the last yaw to match the length of points
    return yaws

def line_equation(yaw, point):
    # Calculate the slope of the line perpendicular to the yaw
    perpendicular_yaw = yaw + np.pi / 2
    m = np.tan(perpendicular_yaw)
    b = point[1] - m * point[0]
    return m, b


def shift_line(x, y, yaw, distance):
    # Calculate the normal vector for the line
    normal_vector = np.array([-np.sin(yaw), np.cos(yaw)])
    # Normalize the normal vector and scale by the distance
    shift_vector = distance * normal_vector
    # Calculate the new shifted line's intercept
    shifted_x = x + shift_vector[0]
    shifted_y = y + shift_vector[1]
    return shifted_x, shifted_y