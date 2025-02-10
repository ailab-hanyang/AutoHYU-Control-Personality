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

class Nonlinear_Model_Predictive_Controller:

    def __init__(self):
        
        ## --- Load MPC formulation ---
        self.acados_solver = acados_settings()
        self.is_MPC_init = True

        return

    

    def calculate_optimal(self, X_ref, frenet_n, vehicle_state, weights):

        if self.is_MPC_init:
            X0 = np.array([0.0, frenet_n, -X_ref[0, 3], vehicle_state.vx , 0.0, 0.0]) 
            print("MPC get init")
        else:
            X0 = np.array([0.0, frenet_n, -X_ref[0, 3], vehicle_state.vx , self.prev_delta , self.prev_acc ])  
        

        self.acados_solver.constraints_set(0,"lbx", X0)
        self.acados_solver.constraints_set(0,"ubx", X0)


        
        ## 5차 다항식 피팅
        num_points = X_ref.shape[0]
        fit_weights = np.linspace(1.0, 0.1, num_points)  # 선형적으로 감소하는 가중치
        if num_points >= 6:
            s_values = X_ref[:, 6]  # s values
            kappa_values = X_ref[:, 5]  # curvature values
            coefficients = np.polyfit(s_values, kappa_values, 5, w=fit_weights)

        for i in range(num_points):
            x_guess = np.array([X_ref[i, 6], 0.0, 0.0, X_ref[i, 4], 0.0, 0.0])
            self.acados_solver.set(i, "x", x_guess)
            p = np.array([X_ref[i, 6], X_ref[i, 4], 
                        weights['s_w'], weights['d_w'], weights['mu_w'], weights['speed_w'], 
                        weights['delta_w'], weights['accel_w'], weights['ddelta_w'], weights['jerk_w'],
                        coefficients[0], coefficients[1], coefficients[2], coefficients[3], coefficients[4], coefficients[5]])
            self.acados_solver.set(i, "p", p)
            
        solve_time_start = time.time()      

        status = self.acados_solver.solve()
        

        pred_X = np.zeros(( num_points , 6)) # initialize empty array with shape (0,6)
        pred_U = np.zeros(( num_points , 2)) 
        if status == 0:
            self.is_MPC_init = False
            for j in range(num_points):
                x = self.acados_solver.get(j,"x")
                x = np.array(x)
                pred_X[j, :] = x
                if j != num_points-1:
                    u = self.acados_solver.get(j,"u") 
                    u = np.array(u)
                    pred_U[j, :] = u
                else:
                    pred_U[j, :] = pred_U[j-1, :]


        stats  = np.zeros(6)

        stats[0] = self.acados_solver.get_cost()


        stats[1] = 1000.0*self.acados_solver.get_stats('time_tot')
        stats[2] = self.acados_solver.get_stats('nlp_iter')
        stats[3] = np.max(self.acados_solver.get_stats('qp_iter'))
        stats[4] = status
        solve_time_end = time.time()
        stats[5] = 1000.0*(solve_time_end-solve_time_start)

        target_cart, self.kappa_pred = frenet_to_cartesian(X_ref, pred_X)
        target_cart = np.hstack((target_cart.T, pred_X[:, 3:5]))
        target_pred = np.hstack((pred_X, pred_U)) 
        # sdelta = CubicSpline(pred_X[:, 0], pred_X[:, 4], extrapolate=True)
        # sacc = CubicSpline(pred_X[:, 0], pred_X[:, 5], extrapolate=True)
        # self.prev_delta = sdelta(vehicle_state.vx*0.02)
        # self.prev_acc = sacc(vehicle_state.vx*0.02)

        # Faero  = 0.5 * self.ro * self.S * self.Cd * vehicle_state.vx**2 
        # Fslope = self.m*self.g*math.sin(-vehicle_state.pitch) 
        # target_torque = (sacc(vehicle_state.vx*0.1)*self.m + Fslope + Faero)*(0.41)
        # if vehicle_state.vx > 1.0:
        #     target_input = [np.rad2deg(sdelta(vehicle_state.vx*0.1)),target_torque]
        # else:
        #     target_input = [np.rad2deg(pred_X[1,6]),pred_X[1,7]*(2300.0)*(0.41)]
        self.prev_delta = pred_X[1,4]
        self.prev_acc = pred_X[1,5]
        target_input = [np.rad2deg(pred_X[1,4]),pred_X[1,5]*(2300.0)*(0.321)]
        return target_cart, target_pred, target_input, stats



