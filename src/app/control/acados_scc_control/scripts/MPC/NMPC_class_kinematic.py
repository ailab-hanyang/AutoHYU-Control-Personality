import numpy as np
import casadi as cs
import yaml
import configparser
import os
import scipy
from typing import Tuple
import time
import math
from scipy.interpolate import CubicSpline
from MPC.NMPC_STM_acados_settings_kinematic import acados_settings      
import matplotlib.pyplot as plt

from utils.trajectory_process import *

class Nonlinear_Model_Predictive_Controller:

    def __init__(self):
        
        ## --- Load MPC formulation ---
        self.acados_solver = acados_settings()
        self.is_MPC_init = True
        self.loader = WeightLoader("weights.ini")
        self.prev_delta = None
        self.prev_acc = None
        self.prev_steers = None
        self.prev_accels = None
        self.last_operation_time = None
        self.pred_X = None
        self.pred_U = None
        self.is_vis_init = False
        # self.viz_init()
        return

    def calculate_optimal(self, road_map,
                                vehicle_state, # x,y,yaw,speed,s,n,mu
                                objects,       # x,y,yaw,id,length,width,height,valid,s,n
                                point,         # x,y,yaw,speed,s,n,mu
                                right_boundary, # x,y,yaw,s,n,mu
                                left_boundary, # x,y,yaw,s,n,mu
                                measured_state
                            ):
        IDX_VS_X = 0
        IDX_VS_Y = 1
        IDX_VS_YAW = 2
        IDX_VS_SPEED = 3
        IDX_VS_S = 4
        IDX_VS_N = 5
        IDX_VS_MU = 6
        IDX_MPC_S = 0
        IDX_MPC_N = 1
        IDX_MPC_MU = 2
        IDX_MPC_SPEED = 3
        IDX_MPC_DELTA = 4
        IDX_MPC_ACC = 5
        # Load weights
        weights, weights_e, constraints, general = self.loader.get_weights_ini()
        
        # MPC 파라미터 설정
        num_step = point.shape[0]
        if self.pred_X is None:
            self.pred_X = np.zeros(( num_step , 6)) # initialize empty array with shape (N,6)
            self.pred_U = np.zeros(( num_step , 2))

        # MPC 초기 상태 설정
        if self.is_MPC_init:
            # X0 = [s, n, mu, v, delta, acc]
            v  = vehicle_state[0, IDX_VS_SPEED] if vehicle_state[0, IDX_VS_SPEED] > 0 else 0.01
            X0 = np.array([vehicle_state[0,IDX_VS_S], vehicle_state[0,IDX_VS_N], vehicle_state[0,IDX_VS_MU], v, 0.0, 0.0])
            self.prev_s = np.linspace(vehicle_state[0, IDX_VS_S], vehicle_state[0, IDX_VS_S]+1.0, np.shape(point)[0])
            self.pred_X[:,0] = self.prev_s
            print("MPC get init")
            self.last_operation_time = time.time()
        else:
            # X0 = [s, n, mu, v, delta, acc]
            current_time = time.time()
            delta_time = current_time - self.last_operation_time
            self.last_operation_time = current_time
            MPC_dt = 0.1
            delta_index = np.max([1, np.min([int(delta_time / MPC_dt), num_step-1])])
            v  = vehicle_state[0, IDX_VS_SPEED] if vehicle_state[0, IDX_VS_SPEED] > 0 else 0.01
            X0 = np.array([vehicle_state[0,IDX_VS_S], vehicle_state[0,IDX_VS_N], vehicle_state[0,IDX_VS_MU], v, self.prev_steers[delta_index] , self.prev_accels[delta_index] ])  
            self.prev_delta = self.prev_steers[delta_index]
            self.prev_acc = self.prev_accels[delta_index]
            
        # print("X0")
        # print(f"s: {X0[0]:.3f}, \tn: {X0[1]:.3f}, \tmu: {X0[2]:.3f}, \tv: {X0[3]:.3f}, \tdelta: {X0[4]:.3f}, \tacc: {X0[5]:.3f}")

        # 차선 길이 계산
        length_list = np.arange(0, np.floor(road_map.GetTotalLength()) + 1, 1).tolist()
        # length_list의 요소 수를 200개로 맞춤
        length_list = np.linspace(0, np.floor(road_map.GetTotalLength()), 200).tolist()
        
        # 차선 곡률 계산
        kappa_values = road_map.GetCurvature(length_list)

        ## 좌우 차선 접선 계산
        # 가까운 lb_sn과 rb_sn 찾기
        lb_sn_closest = find_closest(left_boundary[:,3:], self.prev_s)
        rb_sn_closest = find_closest(right_boundary[:,3:], self.prev_s)

        # 접선 계산
        m1, c1 = compute_tangent_lines(lb_sn_closest, constraints['lane_safety_gap'], "left")
        m2, c2 = compute_tangent_lines(rb_sn_closest, constraints['lane_safety_gap'], "right")

        # 선행차량 위치 계산
        bumpers = compute_object_gap_positions(objects, extra_gap = constraints['bumper_extra_gap'])

        # 제약조건 설정 [Velocity, Steering, Acceleration]
        lbx = np.array([constraints['min_speed'],-constraints['max_delta']])
        ubx = np.array([constraints['max_speed'], constraints['max_delta']])
        
        lbu = np.array([-constraints['max_ddelta'], -constraints['jx_max']])
        ubu = np.array([+constraints['max_ddelta'], +constraints['jx_max']])

        # 비선형 제약조건 설정 [lane_boundary, ay, jy, scc_rear_bumper, scc_centor, scc_front_bumper, friction_circle_constraint]
        relative_distance = vehicle_state[0,3]*constraints['thw'] # Vehicle speed * THW (Time Headway)
        lh = np.array([
            0.0, # lane_boundary
            -0.5, # ay
            -constraints['jy_max'], # jy
            relative_distance, # scc_rear_bumper
            relative_distance, # scc_centor
            relative_distance, # scc_front_bumper
            -1.0 # friction_circle_constraint
            ]) 
        uh = np.array([
            100000.0, # lane_boundary
            constraints['ay_max'], # ay
            constraints['jy_max'], # jy
            1e9, # scc_rear_bumper
            1e9, # scc_centor
            1e9, # scc_front_bumper
            1.0 # friction_circle_constraint
            ]) 
        lh_e = np.array([
            0.0, # lane_boundary
            -0.5, # ay
            relative_distance, # scc_rear_bumper
            relative_distance, # scc_centor
            relative_distance, # scc_front_bumper
            -1.0 # friction_circle_constraint
            ]) # 최종 제약조건은 jy 조건 없음
        uh_e = np.array([
            100000.0, # lane_boundary
            constraints['ay_max'], # ay
            1e9, # scc_rear_bumper
            1e9, # scc_centor
            1e9, # scc_front_bumper
            1.0 # friction_circle_constraint
            ])  # 최종 제약조건은 jy 조건 없음

        # ---- Solver 파라미터 설정 ----
        # self.acados_solver.reset()
        if self.prev_delta is None:
            self.prev_delta = np.zeros(num_step)
            self.prev_acc = np.zeros(num_step)
        for i in range(num_step):
            delta_w = weights['delta_w']
            accel_w = weights['accel_w']
            if i == 0: # 초기 상태 설정
                p = np.array([point[i, IDX_VS_S], point[i, IDX_VS_N], point[i, IDX_VS_MU], point[i, IDX_VS_SPEED],
                              constraints['max_speed'], constraints['ax_max'], constraints['ax_min'], constraints['ay_max'], constraints['super_ellipse_exponent'],
                              self.pred_X[i+1,IDX_VS_S],self.pred_X[i+1,IDX_VS_N],
                              weights['s_w'], weights['d_w'], weights['mu_w'], weights['speed_w'], 
                              delta_w, accel_w, weights['ddelta_w'], weights['jerk_w'], weights['obj_w'],# weights['per_w'],
                              m1[i], c1[i], m2[i], c2[i],
                              *bumpers[:, i, :].flatten(),
                              *kappa_values
                             ])
                # 초기 상태 설정
                self.acados_solver.constraints_set(i, "lbx", X0) 
                self.acados_solver.constraints_set(i, "ubx", X0) 
                self.acados_solver.constraints_set(i, "lbu", lbu) 
                self.acados_solver.constraints_set(i, "ubu", ubu) 

            elif i == num_step-1: # 마지막 상태 설정
                p = np.array([vehicle_state[0,IDX_VS_S], point[i, IDX_VS_N], point[i, IDX_VS_MU], point[i, IDX_VS_SPEED],
                              constraints['max_speed'], constraints['ax_max'], constraints['ax_min'], constraints['ay_max'], constraints['super_ellipse_exponent'],
                              self.pred_X[i,IDX_VS_S],self.pred_X[i,IDX_VS_N],
                              weights_e['s_w'], weights_e['d_w'], weights_e['mu_w'], weights_e['speed_w'], 
                              weights_e['delta_w'], weights_e['accel_w'], weights_e['ddelta_w'], weights_e['jerk_w'], weights_e['obj_w'],# weights_e['per_w'],
                              m1[i], c1[i], m2[i], c2[i],
                              *bumpers[:, i, :].flatten(),
                              *kappa_values
                             ])
                
                self.acados_solver.constraints_set(i, "lbx", lbx)
                self.acados_solver.constraints_set(i, "ubx", ubx)
                self.acados_solver.constraints_set(i, "lh", lh_e)
                self.acados_solver.constraints_set(i, "uh", uh_e)
            else: # 중간 상태 설정
                p = np.array([point[i, IDX_VS_S], point[i, IDX_VS_N], point[i, IDX_VS_MU], point[i, IDX_VS_SPEED],
                              constraints['max_speed'], constraints['ax_max'], constraints['ax_min'], constraints['ay_max'], constraints['super_ellipse_exponent'],
                              self.pred_X[i,IDX_VS_S],self.pred_X[i,IDX_VS_N],
                              weights['s_w'], weights['d_w'], weights['mu_w'], weights['speed_w'], 
                              delta_w, accel_w, weights['ddelta_w'], weights['jerk_w'], weights['obj_w'],# weights['per_w'],
                              m1[i], c1[i], m2[i], c2[i],
                              *bumpers[:, i, :].flatten(),
                              *kappa_values
                             ])
                self.acados_solver.constraints_set(i, "lbx", lbx)
                self.acados_solver.constraints_set(i, "ubx", ubx)
                self.acados_solver.constraints_set(i, "lh", lh)
                self.acados_solver.constraints_set(i, "uh", uh)
                self.acados_solver.constraints_set(i, "lbu", lbu) 
                self.acados_solver.constraints_set(i, "ubu", ubu) 
            self.acados_solver.set(i, "p", p)

        solve_time_start = time.time()      

        status = self.acados_solver.solve()
        
        ## Solver status
        # 0: Success (ACADOS_SUCCESS)
        # 1: NaN detected (ACADOS_NAN_DETECTED)
        # 2: Maximum number of iterations reached (ACADOS_MAXITER)
        # 3: Minimum step size reached (ACADOS_MINSTEP)
        # 4: QP solver failed (ACADOS_QP_FAILURE)
        # 5: Solver created (ACADOS_READY)
        # 6: Problem unbounded (ACADOS_UNBOUNDED)
        # 7: Solver timeout (ACADOS_TIMEOUT)
        status_str = "Success" if (status == 0) else "NaN detected" if (status == 1) else "Maximum number of iterations reached" if (status == 2) else "Minimum step size reached" if (status == 3) else "QP solver failed" if (status == 4) else "Solver created" if (status == 5) else "Problem unbounded" if (status == 6) else "Solver timeout"
        if (status == 0 or status == 2 or status == 3):
            self.is_MPC_init = False
            for j in range(num_step):
                x = self.acados_solver.get(j,"x")
                x = np.array(x)
                self.pred_X[j, :] = x
                # Print the predicted state: time: s, n, mu, v, delta, acc
                if j != num_step-1:
                    u = self.acados_solver.get(j,"u") 
                    u = np.array(u)
                    self.pred_U[j, :] = u
                else:
                    self.pred_U[j, :] = self.pred_U[j-1, :]
                if(general['print'] == 1 and j < num_step-1):
                    print(f"t_idx: {j}, \ts: {x[IDX_MPC_S]:.3f}, \tn: {x[IDX_MPC_N]:.3f}, \tmu: {x[IDX_MPC_MU]:.3f}, \tv: {x[IDX_MPC_SPEED]:.3f}, \tdelta: {np.rad2deg(x[IDX_MPC_DELTA]):.3f}, \tacc: {x[IDX_MPC_ACC]:.3f}, \tddelta: {np.rad2deg(u[0]):.3f}, \tjerk: {u[1]:.3f}")
        else:
            self.acados_solver.reset()
            self.is_MPC_init = True
            print("MPC not solved, reset")
            print(f"status: {status_str}")

        # MPC Solver stats: [Cost, Time, Iter, QP Iter, Status, Solve Time]
        stats  = np.zeros(6)
        stats[0] = self.acados_solver.get_cost()
        stats[1] = 1000.0*self.acados_solver.get_stats('time_tot')
        stats[2] = self.acados_solver.get_stats('nlp_iter')
        stats[3] = np.max(self.acados_solver.get_stats('qp_iter'))
        stats[4] = status
        solve_time_end = time.time()
        stats[5] = 1000.0*(solve_time_end-solve_time_start)
        
        # Print stats
        if(general['print'] == 1):
            print(f"stats: Cost: {stats[0]:.3f}, Time: {stats[1]:.3f}ms, Iter: {stats[2]:.3f}, QP Iter: {stats[3]:.3f}, Status: {stats[4]}, Time: {stats[5]:.3f}ms")

        target_cart = frenet_to_cartesian(road_map, self.pred_X)
        target_cart = np.hstack((target_cart.T, self.pred_X[:, IDX_MPC_SPEED:IDX_MPC_ACC]))
        
        target_pred = np.hstack((self.pred_X, self.pred_U)) 

        self.prev_steers = self.pred_X[:, IDX_MPC_DELTA]
        self.prev_accels = self.pred_X[:, IDX_MPC_ACC]
        self.prev_s   = self.pred_X[:, IDX_MPC_S]
        
        mass_of_ioniq5 = 2300.0
        wheel_radius = 0.321
        future_select_idx = int(general['future_select_idx'])
        target_accel = self.pred_X[future_select_idx, IDX_MPC_ACC]
        target_torque = target_accel*(mass_of_ioniq5)*(wheel_radius)
        
        target_steer = general['steering_gain']*np.rad2deg(self.pred_X[future_select_idx, IDX_MPC_DELTA])
        
        target_input = [target_steer, target_torque]

        # ---- 시각화 ----
        if(general['visualize'] == 1):
            self.viz_run(target_pred, point, left_boundary, right_boundary, vehicle_state, measured_state, bumpers, m1, m2, c1, c2, constraints, kappa_values)

        return target_cart, target_pred, target_input, stats
    
    def viz_init(self):
        # 실시간 플롯을 위한 초기 설정
        plt.ion()  # interactive mode 켜기
        self.fig = plt.figure(figsize=(28, 10), facecolor='black')  # 전체 크기와 배경색 설정
        gs = self.fig.add_gridspec(3, 4)  # 3x4 레이아웃 정의

        # 첫 번째 행 전체를 하나의 큰 플롯으로 합치기
        self.ax_frenet = self.fig.add_subplot(gs[0, :], facecolor='black')  # 첫 번째 행 전체 차지

        # 두 번째 행: Frenet Yaw (1,0), Speed (1,1), Ax (1,2), Ay (1,3)
        self.ax_yaw = self.fig.add_subplot(gs[1, 0], facecolor='black')
        self.ax_speed = self.fig.add_subplot(gs[1, 1], facecolor='black')
        self.ax_ax = self.fig.add_subplot(gs[1, 2], facecolor='black')
        self.ax_ay = self.fig.add_subplot(gs[1, 3], facecolor='black')

        # 세 번째 행: Steering (2,0), **Curvature (2,1) 새로 추가**, Jx (2,2), Jy (2,3)
        self.ax_steering = self.fig.add_subplot(gs[2, 0], facecolor='black')
        
        # -------------------- Curvature Plot 추가 부분 -------------------- #
        self.ax_curvature = self.fig.add_subplot(gs[2, 1], facecolor='black')  # 새로운 축
        self.plot_handles = {}
        
        # Frenet Frame Plot
        self.plot_handles['ref_line'], = self.ax_frenet.plot([], [], 'w--', label='Reference') 
        self.plot_handles['pred_line'], = self.ax_frenet.plot([], [], 'm-', label='Prediction')
        self.plot_handles['left_boundary'], = self.ax_frenet.plot([], [], 'b-', label='Left Boundary')
        self.plot_handles['right_boundary'], = self.ax_frenet.plot([], [], 'r-', label='Right Boundary')
        self.plot_handles['left_tangent'], = self.ax_frenet.plot([], [], 'b--', label='Left Tangent')
        self.plot_handles['right_tangent'], = self.ax_frenet.plot([], [], 'r--', label='Right Tangent')
        self.plot_handles['vehicle_state'] = self.ax_frenet.scatter([], [], color='g', marker='x', label='Vehicle State')
        self.plot_handles['obs_front'] = self.ax_frenet.scatter([], [], color='cyan', marker='o', label='Obstacle Front Bumper')
        self.plot_handles['obs_center'] = self.ax_frenet.scatter([], [], color='orange', marker='s', label='Obstacle Center')
        self.plot_handles['obs_rear'] = self.ax_frenet.scatter([], [], color='magenta', marker='^', label='Obstacle Rear Bumper')

        # Frenet Yaw Plot
        self.plot_handles['yaw_ref_line'], = self.ax_yaw.plot([], [], 'w--', label='Reference')
        self.plot_handles['yaw_pred_line'], = self.ax_yaw.plot([], [], 'm-', label='Prediction')
        self.plot_handles['yaw_vehicle_state'] = self.ax_yaw.scatter([], [], color='g', marker='x', label='Vehicle State')

        # Speed Plot
        self.plot_handles['speed_ref_line'], = self.ax_speed.plot([], [], 'w--', label='Reference')
        self.plot_handles['speed_pred_line'], = self.ax_speed.plot([], [], 'm-', label='Predicted')
        self.plot_handles['speed_vehicle_state'] = self.ax_speed.scatter([], [], color='g', marker='x', label='Vehicle State')

        # Ax Plot
        self.plot_handles['ax_max_line'], = self.ax_ax.plot([], [], 'r--', label='Max')
        self.plot_handles['ax_min_line'], = self.ax_ax.plot([], [], 'r--', label='Min')
        self.plot_handles['ax_pred_line'], = self.ax_ax.plot([], [], 'm-', label='Predicted')
        self.plot_handles['ax_vehicle_state'] = self.ax_ax.scatter([], [], color='g', marker='x', label='Vehicle State')

        # Ay Plot
        self.plot_handles['ay_max_line'], = self.ax_ay.plot([], [], 'r--', label='Max')
        self.plot_handles['ay_min_line'], = self.ax_ay.plot([], [], 'r--', label='Min')
        self.plot_handles['ay_pred_line'], = self.ax_ay.plot([], [], 'm-', label='Predicted')
        self.plot_handles['ay_vehicle_state'] = self.ax_ay.scatter([], [], color='g', marker='x', label='Vehicle State')

        # Steering Plot
        self.plot_handles['steer_pred_line'], = self.ax_steering.plot([], [], 'm-', label='Prediction')
        self.plot_handles['steer_vehicle_state'] = self.ax_steering.scatter([], [], color='g', marker='x', label='Vehicle State')

        # Jx (2,2)
        self.ax_jx = self.fig.add_subplot(gs[2, 2], facecolor='black')
        self.plot_handles['jx_max_line'], = self.ax_jx.plot([], [], 'r--', label='Max')
        self.plot_handles['jx_min_line'], = self.ax_jx.plot([], [], 'r--', label='Min')
        self.plot_handles['jx_pred_line'], = self.ax_jx.plot([], [], 'm-', label='Predicted')

        # Jy (2,3)
        self.ax_jy = self.fig.add_subplot(gs[2, 3], facecolor='black')
        self.plot_handles['jy_max_line'], = self.ax_jy.plot([], [], 'r--', label='Max')
        self.plot_handles['jy_min_line'], = self.ax_jy.plot([], [], 'r--', label='Min')
        self.plot_handles['jy_pred_line'], = self.ax_jy.plot([], [], 'm-', label='Predicted')
        
        # -------------------- Curvature Plot 핸들 추가 -------------------- #
        self.plot_handles['curvature_line'], = self.ax_curvature.plot([], [], 'c-', label='Ref Curvature')

        # -------------------- 각종 축 설정 -------------------- #
        # Frenet Frame
        self.ax_frenet.set_title('Frenet Frame', color='white')
        self.ax_frenet.set_ylabel('N-axis', color='white')
        self.ax_frenet.grid(color='gray')
        self.ax_frenet.legend()
        self.ax_frenet.axis('equal')

        # Frenet Yaw
        self.ax_yaw.set_title('Frenet Yaw', color='white')
        self.ax_yaw.set_ylabel('Yaw (deg)', color='white')
        self.ax_yaw.grid(color='gray')
        self.ax_yaw.legend()
        self.ax_yaw.set_ylim([-15, 15])

        # Speed
        self.ax_speed.set_title('Speed', color='white')
        self.ax_speed.set_ylabel('Speed (km/h)', color='white')
        self.ax_speed.grid(color='gray')
        self.ax_speed.legend()
        self.ax_speed.set_ylim([-1, 70])

        # Ax
        self.ax_ax.set_title('Ax', color='white')
        self.ax_ax.set_ylabel('Ax (m/s^2)', color='white')
        self.ax_ax.grid(color='gray')
        self.ax_ax.legend()
        self.ax_ax.set_ylim([-6, 6])

        # Ay
        self.ax_ay.set_title('Ay', color='white')
        self.ax_ay.set_ylabel('Ay (m/s^2)', color='white')
        self.ax_ay.grid(color='gray')
        self.ax_ay.legend()
        self.ax_ay.set_ylim([-3, 3])

        # Steering
        self.ax_steering.set_title('Steering', color='white')
        self.ax_steering.set_xlabel('S-axis', color='white')
        self.ax_steering.set_ylabel('Steer (deg)', color='white')
        self.ax_steering.grid(color='gray')
        self.ax_steering.legend()
        self.ax_steering.set_ylim([-30, 30])

        # Jx
        self.ax_jx.set_title('Jx', color='white')
        self.ax_jx.set_xlabel('S-axis', color='white')
        self.ax_jx.set_ylabel('Jx (m/s^3)', color='white')
        self.ax_jx.grid(color='gray')
        self.ax_jx.legend()
        self.ax_jx.set_ylim([-6, 6])
        
        # Jy
        self.ax_jy.set_title('Jy', color='white')
        self.ax_jy.set_xlabel('S-axis', color='white')
        self.ax_jy.set_ylabel('Jy (m/s^3)', color='white')
        self.ax_jy.grid(color='gray')
        self.ax_jy.legend()
        self.ax_jy.set_ylim([-6, 6])

        # Curvature
        self.ax_curvature.set_title('Curvature', color='white')
        self.ax_curvature.set_xlabel('S-axis', color='white')
        self.ax_curvature.set_ylabel('kappa (1/m)', color='white')
        self.ax_curvature.grid(color='gray')
        self.ax_curvature.legend()
        self.ax_curvature.set_ylim([-0.1, 0.1])  # 필요시 조정

        # 축 눈금 및 스파인 색상 변경
        axes = [self.ax_frenet, self.ax_yaw, self.ax_speed, self.ax_ax, self.ax_ay, 
                self.ax_steering, self.ax_jx, self.ax_jy, self.ax_curvature]
        for ax in axes:
            ax.tick_params(axis='x', colors='white')
            ax.tick_params(axis='y', colors='white')
            for spine in ax.spines.values():
                spine.set_color('white')

        plt.show(block=False)
        self.is_vis_init = True
        return

    def viz_run(self, 
                target_pred, 
                point, 
                left_boundary, 
                right_boundary, 
                vehicle_state, 
                measured_state, 
                bumpers, 
                m1, m2, c1, c2, 
                constraints,
                kappa_values):  # <- polyfit 계수를 추가로 받음
        
        if not self.is_vis_init:
            self.viz_init()
        
        # -------------------- Frenet Frame -------------------- #
        self.plot_handles['pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['pred_line'].set_ydata(target_pred[:, 1])
        self.plot_handles['ref_line'].set_xdata(point[:, 4])
        self.plot_handles['ref_line'].set_ydata(point[:, 5])
        self.plot_handles['left_boundary'].set_xdata(left_boundary[:, 3])
        self.plot_handles['left_boundary'].set_ydata(left_boundary[:, 4])
        self.plot_handles['right_boundary'].set_xdata(right_boundary[:, 3])
        self.plot_handles['right_boundary'].set_ydata(right_boundary[:, 4])

        yL = m1 * self.prev_s + c1
        yR = m2 * self.prev_s + c2
        self.plot_handles['left_tangent'].set_xdata(self.prev_s)
        self.plot_handles['left_tangent'].set_ydata(yL)
        self.plot_handles['right_tangent'].set_xdata(self.prev_s)
        self.plot_handles['right_tangent'].set_ydata(yR)
        self.plot_handles['vehicle_state'].set_offsets([[vehicle_state[0,4], vehicle_state[0,5]]])

        state_index = 0
        bumpers_state = bumpers[:, state_index, :]
        s_ofront = bumpers_state[:, 0]
        n_ofront = bumpers_state[:, 1]
        s_ocenter = bumpers_state[:, 2]
        n_ocenter = bumpers_state[:, 3]
        s_orear = bumpers_state[:, 4]
        n_orear = bumpers_state[:, 5]
        self.plot_handles['obs_front'].set_offsets(np.vstack((s_ofront, n_ofront)).T)
        self.plot_handles['obs_center'].set_offsets(np.vstack((s_ocenter, n_ocenter)).T)
        self.plot_handles['obs_rear'].set_offsets(np.vstack((s_orear, n_orear)).T)

        # -------------------- Yaw Plot -------------------- #
        self.plot_handles['yaw_pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['yaw_pred_line'].set_ydata(np.rad2deg(target_pred[:, 2]))
        self.plot_handles['yaw_ref_line'].set_xdata(point[:, 4])
        self.plot_handles['yaw_ref_line'].set_ydata(np.rad2deg(point[:, 6]))
        self.plot_handles['yaw_vehicle_state'].set_offsets([[vehicle_state[0, 4], np.rad2deg(vehicle_state[0, 6])]])

        # -------------------- Speed Plot -------------------- #
        self.plot_handles['speed_pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['speed_pred_line'].set_ydata(target_pred[:, 3] * 3.6)
        self.plot_handles['speed_ref_line'].set_xdata(point[:, 4])
        self.plot_handles['speed_ref_line'].set_ydata(point[:, 3] * 3.6)
        self.plot_handles['speed_vehicle_state'].set_offsets([[vehicle_state[0, 4], vehicle_state[0, 3] * 3.6]])

        # -------------------- Ax Plot -------------------- #
        self.plot_handles['ax_pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['ax_pred_line'].set_ydata(target_pred[:, 5])
        self.plot_handles['ax_max_line'].set_xdata(point[:, 4])
        self.plot_handles['ax_max_line'].set_ydata(np.ones_like(point[:, 4])*constraints['ax_max'])
        self.plot_handles['ax_min_line'].set_xdata(point[:, 4])
        self.plot_handles['ax_min_line'].set_ydata(np.ones_like(point[:, 4])*constraints['ax_min'])
        self.plot_handles['ax_vehicle_state'].set_offsets([[vehicle_state[0, 4], measured_state[0, 5]]])

        # -------------------- Ay Plot -------------------- #
        # ay = V^2 / L * tan(delta) 정도의 간단한 관성 공식 예시
        self.plot_handles['ay_pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['ay_pred_line'].set_ydata(
            target_pred[:,3]**2 * np.tan(target_pred[:,4]) / 3.0
        )
        self.plot_handles['ay_max_line'].set_xdata(point[:, 4])
        self.plot_handles['ay_max_line'].set_ydata(np.ones_like(point[:, 4])*constraints['ay_max'])
        self.plot_handles['ay_min_line'].set_xdata(point[:, 4])
        self.plot_handles['ay_min_line'].set_ydata(np.ones_like(point[:, 4])*-constraints['ay_max'])
        self.plot_handles['ay_vehicle_state'].set_offsets([[vehicle_state[0, 4], measured_state[0, 6]]])

        # -------------------- Steering Plot -------------------- #
        self.plot_handles['steer_pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['steer_pred_line'].set_ydata(np.rad2deg(target_pred[:, 4]))
        self.plot_handles['steer_vehicle_state'].set_offsets([[vehicle_state[0, 4], np.rad2deg(measured_state[0, 7])]])

        # -------------------- Jx Plot -------------------- #
        self.plot_handles['jx_pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['jx_pred_line'].set_ydata(target_pred[:, 7])
        self.plot_handles['jx_max_line'].set_xdata(point[:, 4])
        self.plot_handles['jx_max_line'].set_ydata(np.ones_like(point[:, 4])*constraints['jx_max'])
        self.plot_handles['jx_min_line'].set_xdata(point[:, 4])
        self.plot_handles['jx_min_line'].set_ydata(np.ones_like(point[:, 4])*-constraints['jx_max'])

        # -------------------- Jy Plot -------------------- #
        jy = 2*target_pred[:,3]*target_pred[:,4]*target_pred[:,5]/3.0 + (target_pred[:,3]**2)*target_pred[:,6]/3.0
        self.plot_handles['jy_pred_line'].set_xdata(target_pred[:, 0])
        self.plot_handles['jy_pred_line'].set_ydata(jy)
        self.plot_handles['jy_max_line'].set_xdata(point[:, 4])
        self.plot_handles['jy_max_line'].set_ydata(np.ones_like(point[:, 4])*constraints['jy_max'])
        self.plot_handles['jy_min_line'].set_xdata(point[:, 4])
        self.plot_handles['jy_min_line'].set_ydata(np.ones_like(point[:, 4])*-constraints['jy_max'])

        # -------------------- Curvature Plot -------------------- #
        # kappa(s) = c0 * s^5 + c1 * s^4 + c2 * s^3 + c3 * s^2 + c4 * s + c5
        s_predict = np.arange(0, len(kappa_values)*0.1, 0.1)
        self.plot_handles['curvature_line'].set_xdata(s_predict)
        self.plot_handles['curvature_line'].set_ydata(kappa_values)

        # -------------------- 각 축 범위 재조정 & 업데이트 -------------------- #
        all_axes = [
            self.ax_frenet, self.ax_yaw, self.ax_speed, self.ax_ax, self.ax_ay, 
            self.ax_steering, self.ax_jx, self.ax_jy, self.ax_curvature
        ]
        for ax in all_axes:
            ax.relim()
            ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        return


class WeightLoader:
    def __init__(self, file_name="weights.ini"):
        # 같은 디렉토리에 있는 파일 경로 설정
        self.ini_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
        self.ini_config = configparser.ConfigParser()
        
        self.last_modified_time = None
        self.weights = {}
        self.weights_e = {}
        self.constraints = {}

        self._load_weights_ini()

    def _load_weights_ini(self):
        # UTF-8-BOM 인코딩으로 파일 읽기
        with open(self.ini_file_path, 'r', encoding='utf-8-sig') as f:
            self.ini_config.read_file(f)
        
        self.general = {}
        self.weights = {}
        self.weights_e = {}
        self.constraints = {}
        
        for key, value in self.ini_config.items('general'):
            self.general[key] = float(value)
        for key, value in self.ini_config.items('weights'):
            self.weights[key] = float(value)
        for key, value in self.ini_config.items('weights_e'):
            self.weights_e[key] = float(value)
        for key, value in self.ini_config.items('constraints'):
            self.constraints[key] = float(value)
        
        self.last_modified_time = os.path.getmtime(self.ini_file_path)

    def get_weights_ini(self):
        current_modified_time = os.path.getmtime(self.ini_file_path)
        if current_modified_time != self.last_modified_time:
            self._load_weights_ini()
            print ("Weights loaded from ini file")
        return self.weights, self.weights_e, self.constraints, self.general
