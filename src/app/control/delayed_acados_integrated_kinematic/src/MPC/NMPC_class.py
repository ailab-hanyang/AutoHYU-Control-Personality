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
            model_name,
            model_costtype,
            config_path,
            controller_params,
            MPC_params,
            reference_trajectory
        ):
        ## --- MPC cost function params ---
        self.model_name = model_name
        self.model_costtype = model_costtype
        self.MPC_params = MPC_params
        self.Tp      = controller_params['Tp']     # prediction horizon [s]
        self.Ts_MPC  = controller_params['Ts_MPC'] # MPC prediction discretization step [s]
        self.N       = int(self.Tp / self.Ts_MPC)  # number of discretizaion steps MPC
        ## --- Model params ---     
        veh_params_file_MPC         = controller_params['veh_params_file_MPC']
        tire_params_file_MPC        = controller_params['tire_params_file_MPC']
        self.tire_params_full_path  = config_path+tire_params_file_MPC
        self.veh_params_full_path   = config_path+veh_params_file_MPC
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

        self.Q   = np.diag([q_lon* (1/s_lon**2), q_lat* (1/s_lat**2), q_yaw * (1/s_yaw**2), q_vel * (1/s_vel**2)]) 
        self.R   = np.diag([r_jerk * (1/s_jerk**2), r_steering_rate * (1/s_steering_rate**2)])
        self.Qe  = self.Q      # np.diag([10.0, 20.0, 10.0, 1.0])   # terminal weight

        X0_MPC = np.array([0.0, 0.0, 0.0, reference_trajectory[0, 4], # x y yaw v
                           0.0, 0.0,                # vlat, yaw_rate,
                           0.0, 0.0, 0.0, 0.0, 0.0, # delta_l, delta_2, delta_1, delta_0, delta
                           0.0, 0.0, 0.0, 0.0, 0.0] )    # a_l, a_2, a_1, a_0, a

        solver_generate_C_code  = self.MPC_params['solver_generate_C_code']
        solver_build            = self.MPC_params['solver_build']
        ## --- configure MPC constraints ---
        # load acceleration limits 
        lookuptable_gg_limits_file  = self.MPC_params['lookuptable_gg_limits']
        self.combined_acc_limits    = self.MPC_params['combined_acc_limits']
        # generate interpoltaed local ax & ay limits
        self.ax_lim, self.ay_lim = calculate_velvar_latlon_acc(config_path + lookuptable_gg_limits_file)

        ## --- Load MPC formulation ---
        costfunction_type = self.MPC_params['costfunction_type']
        from MPC.NMPC_STM_acados_settings import acados_settings
        (
            self.constraint,
            self.model,
            self.acados_solver,
            self.ocp    
        ) = acados_settings(
            self.Tp, self.N, X0_MPC, self.Q, self.R, self.Qe, self.L1_pen,
            self.L2_pen, self.ax_lim, self.ay_lim, self.combined_acc_limits,
            self.model_name, self.model_costtype,
            self.veh_params_full_path, self.tire_params_full_path,
            solver_generate_C_code=solver_generate_C_code,
            solver_build=solver_build
        )

        self.costfunction_type = self.ocp.cost.cost_type
        self.nx     = self.model.x.size()[0]
        # self.x0     = X0_MPC
        # self.acados_solver.constraints_set(0, "lbx", self.x0)
        # self.acados_solver.constraints_set(0, "ubx", self.x0)
        self.stats  = np.zeros(6)
        self.pred_X = np.empty((self.N , self.nx)) # initialize empty array with shape (0,6)
        self.is_MPC_init = True
        self.is_model_changed = True
        # Get the number of non linear constraints in the stage- and terminal cost
        # self.nh = self.ocp.model.con_h_expr.shape[0]
        # self.nh_e = self.ocp.model.con_h_expr_e.shape[0]   

        return


    def solve(
            self,
            current_ref_traj: np.ndarray, # [t, x, y, yaw, speed]
        ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        solve_time_start = time.time()       

        # update MPC reference
        for j in range(self.N):

            if self.costfunction_type == 'LINEAR_LS' or self.costfunction_type == 'NONLINEAR_LS':
                yref = np.array([current_ref_traj[j, 1], current_ref_traj[j, 2], 
                                 current_ref_traj[j, 3], current_ref_traj[j, 4],
                                 0, 0])         # steering_rate, jerk
                self.acados_solver.set(j, "yref", yref)
                self.acados_solver.set(j, "p", current_ref_traj[j, 5])  # reference curvature
                
            elif self.costfunction_type == 'EXTERNAL':
                yref = np.array([current_ref_traj[j, 1], current_ref_traj[j, 2],
                                 current_ref_traj[j, 3], current_ref_traj[j, 4]])
                self.acados_solver.set(j, "p", yref)


        yref_N = np.array([current_ref_traj[self.N, 1], current_ref_traj[self.N, 2], 
                           current_ref_traj[self.N, 3], current_ref_traj[self.N, 4]]) 

        if self.costfunction_type == 'LINEAR_LS' or self.costfunction_type == 'NONLINEAR_LS':
            self.acados_solver.set(self.N, "yref", yref_N)
            self.acados_solver.set(self.N, "p", current_ref_traj[self.N, 5])  # reference curvature

        elif self.costfunction_type == 'EXTERNAL':
            self.acados_solver.set(self.N, "p", yref_N)


        # solve MPC problem
        status = self.acados_solver.solve()
        # hpipm_status: 
            # 0: SUCCESS, // found solution satisfying accuracy tolerance
            # 1: MAX_ITER, // maximum iteration number reached
            # 2: MIN_STEP, // minimum step length reached
            # 3: NAN_SOL, // NaN in solution detected
            # 4: INCONS_EQ, // unconsistent equality constraints

        # get MPC solution
        # x0 = self.acados_solver.get(0, "x")
        u0 = self.acados_solver.get(0, "u")
        # u0 = acados_solver.solve_for_x0(x_next)
        if status == 0: 
            pred_X = np.empty((0, self.nx)) # initialize empty array with shape (0,6)
            for j in range(self.N):
                x       = self.acados_solver.get(j,"x")
                x       = np.array(x) 
                pred_X  = np.concatenate((pred_X, x.reshape(1, -1)), axis=0)
            self.pred_X = pred_X
        self.stats[0] = self.acados_solver.get_cost()
        self.stats[1] = 1000.0*self.acados_solver.get_stats('time_tot')
        self.stats[2] = self.acados_solver.get_stats('sqp_iter')
        self.stats[3] = np.max(self.acados_solver.get_stats('qp_iter'))
        self.stats[4] = status
        solve_time_end = time.time()
        self.stats[5] = 1000.0*(solve_time_end-solve_time_start)

        return u0, self.pred_X, self.stats

    def set_initial_state(self, x0):
        self.x0 = x0        
        self.acados_solver.constraints_set(0, "lbx", self.x0)
        self.acados_solver.constraints_set(0, "ubx", self.x0)

        return
    
    def reset(self, x0):
        self.acados_solver.reset()
        self.set_initial_state(x0)
        for i in range(self.N+1):
            self.acados_solver.set(i, 'x', self.x0)


    def reintialize_solver(self, X_ref, solver_generate_C_code=False, solver_build=False):
        # [t, x, y, yaw, speed]
        x0 = np.array([0.0, 0.0, 0.0, X_ref[0, 4], 
                       0.0, 0.0,            # vlat, yaw_rate,
                       0.0, 0.0, 0.0, 0.0, 0.0,  # delta_l, delta_2, delta_1, delta_0
                       0.0, 0.0, 0.0, 0.0, 0.0])  # a_l, a_2, a_1, a_0

        ## --- Load MPC formulation ---
        from MPC.NMPC_STM_acados_settings import acados_settings
        (
            self.constraint,
            self.model,
            self.acados_solver,
            self.ocp
        ) = acados_settings(
            self.Tp, self.N, x0,  self.Q, self.R, self.Qe, self.L1_pen,
            self.L2_pen, self.ax_lim, self.ay_lim, self.combined_acc_limits,
            self.model_name, self.model_costtype,
            self.veh_params_full_path, self.tire_params_full_path,
            solver_generate_C_code = solver_generate_C_code,
            solver_build = solver_build
        )
        self.costfunction_type = self.ocp.cost.cost_type
        self.set_initial_state(x0)

        return
    

    def calculate_optimal(self, X_ref, vehicle_state, MPC_params, another_model_state = None):
        self.param_update(MPC_params)
        x0 = self.model.init_state(vehicle_state, self.pred_X, self.is_MPC_init, another_model_state)
        # print(x0)
        self.set_initial_state(x0)

        """
        if(self.is_MPC_init == True):
            X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , 0.0 , 0.0, 0.0, 0.0]) 
            print("MPC get init")
        else:
            # X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , 0.0, vehicle_state.yaw_rate, self.pred_X[1,6], self.pred_X[1,7]]) 
            X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , vehicle_state.vy, vehicle_state.yaw_rate, self.pred_X[1,6], self.pred_X[1,7]]) 

        if self.is_model_changed == True:
            X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , vehicle_state.vy, vehicle_state.yaw_rate, before_pred[1,4], before_pred[1,5]]) 
        """ 

        u0, pred_X, MPC_stats = self.solve(X_ref)
        
        if MPC_stats[4] != 0:
            print("acados returned status {}.".format(MPC_stats[4]))
            self.reintialize_solver(X_ref) 
            self.is_MPC_init = True
        else:
            self.is_MPC_init = False

        
        # target_pred = np.hstack((self.pred_X[:, 0:4], self.pred_X[:, 6:8]))
        target_pred = np.hstack((self.pred_X[:, 0:4], 
                                 self.pred_X[:, 10][:, np.newaxis],
                                 self.pred_X[:, 15][:, np.newaxis]))
        print(pred_X[1,15])
        target_input = [np.rad2deg(pred_X[1,10]),pred_X[1,15]*(2300.0)*(0.321)]
        return target_pred , target_input, MPC_stats


    def param_update(self, MPC_params):
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

        # if self.is_model_changed:
        #     r_jerk          = 0.001
        #     r_steering_rate = 0.001
    
        
        self.Q   = np.diag([q_lon* (1/s_lon**2), q_lat* (1/s_lat**2), q_yaw * (1/s_yaw**2), q_vel * (1/s_vel**2)])   
        self.R   = np.diag([r_jerk * (1/s_jerk**2), r_steering_rate * (1/s_steering_rate**2)])
        self.Qe  = self.Q      # np.diag([10.0, 20.0, 10.0, 1.0])   # terminal weight

        W = scipy.linalg.block_diag(self.Q, self.R)
        We = self.Qe

        # update cost function weights
        for i in range(self.N):
            self.acados_solver.cost_set(i, 'W', W)
        self.acados_solver.cost_set(self.N, 'W', We)




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