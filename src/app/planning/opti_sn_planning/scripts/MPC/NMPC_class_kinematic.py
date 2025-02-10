import numpy as np
import casadi as cs
import yaml
import os
import scipy
from typing import Tuple
import time
from scipy.interpolate import CubicSpline
from MPC.NMPC_STM_acados_settings_kinematic import acados_settings      
import matplotlib.pyplot as plt

from utils.trajectory_process import *

class Nonlinear_Model_Predictive_Controller:

    def __init__(self):
        
        ## --- Load MPC formulation ---
        self.acados_solver = acados_settings()
        self.is_MPC_init = True
        self.loader = WeightLoader("weights.yaml")
        self.viz_init()
        return

    def calculate_optimal(self, road_map,
                                vehicle_state, # x,y,yaw,speed,s,n,mu
                                objects,       # x,y,yaw,id,length,width,height,valid,s,n
                                point,         # x,y,yaw,speed,s,n,mu
                                right_boundary, # x,y,yaw,s,n,mu
                                left_boundary, # x,y,yaw,s,n,mu
                                measured_state
                            ):

        weights, weights_0, weights_e, constraints = self.loader.get_weights()
        
        num_step = point.shape[0]
        pred_X = np.zeros(( num_step , 6)) # initialize empty array with shape (0,6)
        pred_U = np.zeros(( num_step , 2)) 

        if self.is_MPC_init:
            X0 = np.array([vehicle_state[0,4], vehicle_state[0,5], vehicle_state[0,6], vehicle_state[0,3], 0.0, 0.0])
            self.prev_s = np.linspace(vehicle_state[0,4], vehicle_state[0,4]+1.0, np.shape(point)[0])
            pred_X[:,0] = self.prev_s
            print("MPC get init")
        else:
            X0 = np.array([vehicle_state[0,4], vehicle_state[0,5], vehicle_state[0,6], vehicle_state[0,3], self.prev_delta , self.prev_acc ])  

        length_list = np.arange(0, np.floor(road_map.GetTotalLength()) + 1, 1).tolist()
        
        kappa_values = road_map.GetCurvature(length_list)
        
        ## 5차 다항식 피팅
        fit_weights = np.linspace(1.0, 0.1, len(length_list))  # 선형적으로 감소하는 가중치
        coefficients = np.polyfit(length_list, kappa_values, 5, w=fit_weights)

        ## 좌우 차선 접선 계산
        # 가까운 lb_sn과 rb_sn 찾기
        lb_sn_closest = find_closest(left_boundary[:,3:], self.prev_s)
        rb_sn_closest = find_closest(right_boundary[:,3:], self.prev_s)

        # 접선 계산
        m1, c1 = compute_tangent_lines(lb_sn_closest, constraints['lane_safety_gap'], "left")
        m2, c2 = compute_tangent_lines(rb_sn_closest, constraints['lane_safety_gap'], "right")

        bumpers = compute_object_gap_positions(objects, extra_gap = constraints['bumper_extra_gap'])
        # # np.set_printoptions(formatter={'float':lambda x: "{0:0.3f}".format(x)})
        # print("vehicle",vehicle_state[0,4] + 1.5*np.cos(vehicle_state[0,6])
        #                ,vehicle_state[0,5] + 1.5*np.sin(vehicle_state[0,6]))
        # print("bumpers",bumpers[:,0,:])

        lbx = np.array([constraints['min_speed'],-constraints['max_delta'], constraints['min_ax']])
        ubx = np.array([constraints['max_speed'], constraints['max_delta'], constraints['max_ax']])

        lh = np.array([ 0.0 , 0.0, 
                            constraints['max_collision_boundary'], constraints['max_collision_boundary'], constraints['max_collision_boundary'],
                            constraints['max_collision_boundary'], constraints['max_collision_boundary'], constraints['max_collision_boundary'], 
                            constraints['max_collision_boundary'], constraints['max_collision_boundary'], constraints['max_collision_boundary'], 
                            constraints['max_collision_boundary'], constraints['max_collision_boundary'], constraints['max_collision_boundary']
                        ]) 
        uh = np.array([ 1000, constraints['max_ay'], 
                            1e9, 1e9, 1e9, 1e9,1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9 
                        ]) 

        for i in range(num_step):
            if i > num_step*3.0/4.0: # do not trust every reference ...
                x_guess = np.array([point[i, 4], point[i, 5], point[i, 6], point[i, 3], 0.0, 0.0])
                self.acados_solver.set(i, "x", x_guess)
            if i == 0:
                p = np.array([point[i, 4], point[i, 5], point[i, 6], point[i, 3],
                              constraints['max_speed'], pred_X[i+1,4],pred_X[i+1,5],
                        weights_0['s_w'], weights_0['d_w'], weights_0['mu_w'], weights_0['speed_w'], 
                        weights_0['delta_w'], weights_0['accel_w'], weights_0['ddelta_w'], weights_0['jerk_w'], weights_0['obj_w'],
                        coefficients[0], coefficients[1], coefficients[2], coefficients[3], coefficients[4], coefficients[5],
                        m1[i], c1[i], m2[i], c2[i],
                        *bumpers[:, i, :].flatten()
                        ])
                
                self.acados_solver.constraints_set(i, "lbx", X0) 
                self.acados_solver.constraints_set(i, "ubx", X0) 

            elif i == num_step-1:
                p = np.array([point[i, 4], point[i, 5], point[i, 6], point[i, 3],
                              constraints['max_speed'], pred_X[i,4],pred_X[i,5],
                        weights_e['s_w'], weights_e['d_w'], weights_e['mu_w'], weights_e['speed_w'], 
                        weights_e['delta_w'], weights_e['accel_w'], weights_e['ddelta_w'], weights_e['jerk_w'], weights_e['obj_w'],
                        coefficients[0], coefficients[1], coefficients[2], coefficients[3], coefficients[4], coefficients[5],
                        m1[i], c1[i], m2[i], c2[i],
                        *bumpers[:, i, :].flatten()
                        ])
                
                self.acados_solver.constraints_set(i, "lbx", lbx)
                self.acados_solver.constraints_set(i, "ubx", ubx)
                self.acados_solver.constraints_set(i, "lh", lh)
                self.acados_solver.constraints_set(i, "uh", uh)
            else:
                delta_w = calculate_weight(weights['delta_w'], 0, i, num_step - 1)
                accel_w = calculate_weight(weights['accel_w'], 0, i, num_step - 1)

                p = np.array([vehicle_state[0,4], point[i, 5], point[i, 6], point[i, 3],
                              constraints['max_speed'], pred_X[i,4],pred_X[i,5],
                        weights['s_w'], weights['d_w'], weights['mu_w'], weights['speed_w'], 
                        delta_w, accel_w, weights['ddelta_w'], weights['jerk_w'], weights['obj_w'],
                        coefficients[0], coefficients[1], coefficients[2], coefficients[3], coefficients[4], coefficients[5],
                        m1[i], c1[i], m2[i], c2[i],
                        *bumpers[:, i, :].flatten()
                        ])
                self.acados_solver.constraints_set(i, "lbx", lbx)
                self.acados_solver.constraints_set(i, "ubx", ubx)
                self.acados_solver.constraints_set(i, "lh", lh)
                self.acados_solver.constraints_set(i, "uh", uh)
            self.acados_solver.set(i, "p", p)


        solve_time_start = time.time()      

        status = self.acados_solver.solve()
        

        if status == 0:
            self.is_MPC_init = False
            for j in range(num_step):
                x = self.acados_solver.get(j,"x")
                x = np.array(x)
                pred_X[j, :] = x
                if j != num_step-1:
                    u = self.acados_solver.get(j,"u") 
                    u = np.array(u)
                    pred_U[j, :] = u
                else:
                    pred_U[j, :] = pred_U[j-1, :]
        else:
            self.acados_solver = acados_settings(build=False)
            self.is_MPC_init = True


        stats  = np.zeros(6)

        stats[0] = self.acados_solver.get_cost()


        stats[1] = 1000.0*self.acados_solver.get_stats('time_tot')
        stats[2] = self.acados_solver.get_stats('nlp_iter')
        stats[3] = np.max(self.acados_solver.get_stats('qp_iter'))
        stats[4] = status
        solve_time_end = time.time()
        stats[5] = 1000.0*(solve_time_end-solve_time_start)

        target_cart = frenet_to_cartesian(road_map, pred_X)
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
        self.prev_s   = pred_X[:,0]
        target_input = [np.rad2deg(pred_X[1,4]),pred_X[1,5]*(2300.0)*(0.321)]


        self.viz_run(pred_X,point,left_boundary,right_boundary,vehicle_state,measured_state,bumpers,m1,m2,c1,c2)

        return target_cart, target_pred, target_input, stats
    
    def viz_init(self):
        # 실시간 플롯을 위한 초기 설정
        plt.ion()  # interactive mode 켜기
        self.fig = plt.figure(figsize=(18, 10), facecolor='black')  # 전체 크기와 배경색 설정
        gs = self.fig.add_gridspec(3, 2)  # 3x2 레이아웃 정의

        # 첫 번째 행 전체를 하나의 큰 플롯으로 합치기
        self.ax_frenet = self.fig.add_subplot(gs[0, :], facecolor='black')  # 첫 번째 행 전체 차지

        # 두 번째 행: Frenet Yaw (1,0)과 Speed (1,1)
        self.ax_yaw = self.fig.add_subplot(gs[1, 0], facecolor='black')
        self.ax_speed = self.fig.add_subplot(gs[1, 1], facecolor='black')

        # 세 번째 행: Steering (2,0)과 Accel (2,1)
        self.ax_steering = self.fig.add_subplot(gs[2, 0], facecolor='black')
        self.ax_accel = self.fig.add_subplot(gs[2, 1], facecolor='black')

        self.plot_handles = {}

        # Frenet Frame Plot
        self.plot_handles['ref_line'], = self.ax_frenet.plot([], [], 'w--', label='Reference')  # 라인 색상 흰색
        self.plot_handles['pred_line'], = self.ax_frenet.plot([], [], 'm-', label='Prediction')
        self.plot_handles['left_boundary'], = self.ax_frenet.plot([], [], 'b-', label='Left Boundary')
        self.plot_handles['right_boundary'], = self.ax_frenet.plot([], [], 'r-', label='Right Boundary')
        self.plot_handles['left_tangent'], = self.ax_frenet.plot([], [], 'b--', label='Left Tangent Line')
        self.plot_handles['right_tangent'], = self.ax_frenet.plot([], [], 'r--', label='Right Tangent Line')
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

        # Steer Plot
        self.plot_handles['steer_pred_line'], = self.ax_steering.plot([], [], 'm-', label='Prediction')
        self.plot_handles['steer_vehicle_state'] = self.ax_steering.scatter([], [], color='g', marker='x', label='Vehicle State')

        # Accel Plot
        self.plot_handles['accel_pred_line'], = self.ax_accel.plot([], [], 'm-', label='Predicted')
        self.plot_handles['accel_vehicle_state'] = self.ax_accel.scatter([], [], color='g', marker='x', label='Vehicle State')

        # 라벨, 그리드, 축 범위 설정
        # Frenet Frame
        self.ax_frenet.set_title('Frenet Frame', color='white')
        self.ax_frenet.set_xlabel('S-axis', color='white')
        self.ax_frenet.set_ylabel('N-axis', color='white')
        self.ax_frenet.grid(color='gray')
        self.ax_frenet.legend()
        self.ax_frenet.axis('equal')

        # Frenet Yaw
        self.ax_yaw.set_title('Frenet Yaw', color='white')
        self.ax_yaw.set_ylabel('Mu (deg)', color='white')
        self.ax_yaw.grid(color='gray')
        self.ax_yaw.legend()
        self.ax_yaw.set_ylim([-15, 15])

        # Speed
        self.ax_speed.set_title('Speed', color='white')
        self.ax_speed.set_ylabel('Speed (km/h)', color='white')
        self.ax_speed.grid(color='gray')
        self.ax_speed.legend()
        self.ax_speed.set_ylim([-1, 70])

        # Steering
        self.ax_steering.set_title('Steering', color='white')
        self.ax_steering.set_xlabel('S-axis', color='white')
        self.ax_steering.set_ylabel('Steer (deg)', color='white')
        self.ax_steering.grid(color='gray')
        self.ax_steering.legend()
        self.ax_steering.set_ylim([-30, 30])

        # Accel
        self.ax_accel.set_title('Accel', color='white')
        self.ax_accel.set_xlabel('S-axis', color='white')
        self.ax_accel.set_ylabel('Accel (m/s2)', color='white')
        self.ax_accel.grid(color='gray')
        self.ax_accel.legend()
        self.ax_accel.set_ylim([-6, 6])

        # 축 눈금 및 스파인 색상 변경
        axes = [self.ax_frenet, self.ax_yaw, self.ax_speed, self.ax_steering, self.ax_accel]
        for ax in axes:
            ax.tick_params(axis='x', colors='white')
            ax.tick_params(axis='y', colors='white')
            for spine in ax.spines.values():
                spine.set_color('white')

        plt.show(block=False)
        return

    def viz_run(self, pred_X, point, left_boundary, right_boundary, vehicle_state, measured_state, bumpers, m1, m2, c1, c2):
        # Frenet Frame 업데이트
        self.plot_handles['pred_line'].set_xdata(pred_X[:, 0])
        self.plot_handles['pred_line'].set_ydata(pred_X[:, 1])
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

        # Frenet Yaw Plot 업데이트
        self.plot_handles['yaw_pred_line'].set_xdata(pred_X[:, 0])
        self.plot_handles['yaw_pred_line'].set_ydata(np.rad2deg(pred_X[:, 2]))
        self.plot_handles['yaw_ref_line'].set_xdata(point[:, 4])
        self.plot_handles['yaw_ref_line'].set_ydata(np.rad2deg(point[:, 6]))
        self.plot_handles['yaw_vehicle_state'].set_offsets([[vehicle_state[0, 4], np.rad2deg(vehicle_state[0, 6])]])

        # Speed Plot 업데이트
        self.plot_handles['speed_pred_line'].set_xdata(pred_X[:, 0])
        self.plot_handles['speed_pred_line'].set_ydata(pred_X[:, 3] * 3.6)
        self.plot_handles['speed_ref_line'].set_xdata(point[:, 4])
        self.plot_handles['speed_ref_line'].set_ydata(point[:, 3] * 3.6)
        self.plot_handles['speed_vehicle_state'].set_offsets([[vehicle_state[0, 4], vehicle_state[0, 3] * 3.6]])

        # Steer Plot 업데이트
        self.plot_handles['steer_pred_line'].set_xdata(pred_X[:, 0])
        self.plot_handles['steer_pred_line'].set_ydata(np.rad2deg(pred_X[:, 4]))
        self.plot_handles['steer_vehicle_state'].set_offsets([[vehicle_state[0, 4], np.rad2deg(measured_state[0, 7])]])

        # Accel Plot 업데이트
        self.plot_handles['accel_pred_line'].set_xdata(pred_X[:, 0])
        self.plot_handles['accel_pred_line'].set_ydata(pred_X[:, 5])
        self.plot_handles['accel_vehicle_state'].set_offsets([[vehicle_state[0, 4], measured_state[0, 5]]])

        # 각 축 범위 재조정
        self.ax_frenet.relim()
        self.ax_frenet.autoscale_view()
        self.ax_yaw.relim()
        self.ax_yaw.autoscale_view()
        self.ax_speed.relim()
        self.ax_speed.autoscale_view()
        self.ax_steering.relim()
        self.ax_steering.autoscale_view()
        self.ax_accel.relim()
        self.ax_accel.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        return

class WeightLoader:
    def __init__(self, file_name="weights.yaml"):
        # 같은 디렉토리에 있는 파일 경로 설정
        self.file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
        self.last_modified_time = None
        self.weights = {}
        self.weights_0 = {}
        self.weights_e = {}
        self.constraints = {}
        self._load_weights()

    def _load_weights(self):
        """YAML 파일에서 weights를 로드합니다."""
        with open(self.file_path, 'r') as file:
            data = yaml.safe_load(file)
        self.weights = data.get('weights', {})
        self.weights_0 = data.get('weights_0', {})
        self.weights_e = data.get('weights_e', {})
        self.constraints = data.get('constraints', {})
        self.last_modified_time = os.path.getmtime(self.file_path)

    def get_weights(self):
        """파일 변경 사항이 있을 경우 weights를 갱신합니다."""
        current_modified_time = os.path.getmtime(self.file_path)
        if current_modified_time != self.last_modified_time:
            self._load_weights()
        return self.weights, self.weights_0, self.weights_e, self.constraints