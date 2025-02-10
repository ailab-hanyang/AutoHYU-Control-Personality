#!/usr/bin/env python3
# https://github.com/bayesian-optimization/BayesianOptimization

import rospy
from personality_msgs.msg import PersonalityTimeWindow, PersonalityScene, PersonalityScenePoint
from std_msgs.msg import Float32MultiArray

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import *
import numpy as np
from bayes_opt import BayesianOptimization
from bayes_opt.util import UtilityFunction
import matplotlib.pyplot as plt
from personality_extraction import libfrenet2 as frenet
from scipy.interpolate import Akima1DInterpolator
from scipy.optimize import minimize
import math

def ode_model() -> AcadosModel:
    model_name = 'integrate_kinematic_bicycle_nmpc'

    # constants
    lf = 1.49    # 전축에서 무게중심까지의 거리 [m]
    lr = 1.51    # 후축에서 무게중심까지의 거리 [m]
    L = lf + lr # 차량의 휠베이스 [m]

    # set up states
    s       = SX.sym('x')
    d       = SX.sym('d')
    mu      = SX.sym('mu')
    v       = SX.sym('v')
    delta   = SX.sym('delta')
    a       = SX.sym('a')
    state = vertcat(s, d, mu, v, delta, a)

    # set up control inputs
    ddelta = SX.sym('ddelta')
    jerk = SX.sym('jerk')
    input = vertcat(ddelta, jerk)

    # xdot
    s_dot     = SX.sym('s_dot')
    d_dot     = SX.sym('d_dot')
    mu_dot    = SX.sym('mu_dot')
    v_dot     = SX.sym('v_dot')
    delta_dot = SX.sym('delta_dot')
    a_dot     = SX.sym('a_dot')

    c = SX.sym("kappa",6)
    kappa = c[0]*s**5 + c[1]*s**4 + c[2]*s**3 + c[3]*s**2 + c[4]*s+ c[5]

    # states derivatives (continuous time dynamics)
    beta = atan2(lr*tan(delta),L)

    s_dot    = (v * cos(mu))/(1.0-d*kappa)
    d_dot    = v * sin(mu) 
    mu_dot   = v * cos(beta) * tan(delta)/L - kappa*s_dot
    v_dot     = a
    delta_dot = ddelta
    a_dot     = jerk
    
    xdot = vertcat(s_dot, d_dot, mu_dot, v_dot, delta_dot, a_dot)


    model = AcadosModel()
    model.f_expl_expr = xdot
    model.x = state
    model.u = input
    model.name = model_name
    model.p = c
    return model

class WeightBayesOpt:
    def __init__(self):
        rospy.init_node('personality_weight_bayesopt', anonymous=True)
        self.period = rospy.get_param('/task_period/period_personality_extraction')

        # Subscriber
        self.s_personality = rospy.Subscriber(
            "/personality/time_window", 
            PersonalityTimeWindow, 
            self.callback_personality_time_window
        )
        
        # 초기화
        self.personality_data = None
        self.acados_solver = None
        self.setup_acados()
        # self.setup_bayesian_optimization()

        self.initial_prediction = None  # 초기 예측 결과 저장
        self.optimal_prediction = None  # 최적 예측 결과 저장
        self.reference_path = None      # 레퍼런스 경로 저장
        self.iteration_infos = []


        # 초기 가중치 설정
        self.initial_weights = {
            's_w': 1.0,
            'd_w': 1.0,
            'mu_w': 1.0,
            'speed_w': 1.0,
            'delta_w': 1.0,
            'accel_w': 1.0,
            'ddelta_w': 1.0,
            'jerk_w': 1.0
        }

    def setup_acados(self):
        """ACADOS MPC 설정"""
        
        ocp = AcadosOcp()
        # set model
        model = ode_model()
        ocp.model = model

        Tf = 4.0
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        N = 80 # number of prediction states (x0 - x39)

        # set prediction horizon
        ocp.solver_options.N_horizon = N-1 # Number of shooting intervals == number of inputs
        ocp.solver_options.tf = Tf

        
        # parameters
        s_ref = SX.sym('s_ref')
        speed_ref = SX.sym('speed_ref')
        s_w = SX.sym('s_w')
        d_w = SX.sym('d_w')
        mu_w = SX.sym('mu_w')
        speed_w = SX.sym('speed_w')
        delta_w = SX.sym('delta_w')
        accel_w = SX.sym('accel_w')
        ddelta_w = SX.sym('ddelta_w')
        jerk_w = SX.sym('jerk_w')

        p = vertcat(s_ref,speed_ref,s_w,d_w,mu_w,speed_w,delta_w,accel_w,ddelta_w,jerk_w,model.p)
        
        ocp.model.p = p
        ocp.parameter_values = np.zeros(16)

        # cost matrices
        Q_mat = 2*np.diag([s_w, d_w, mu_w, speed_w, delta_w, accel_w])
        R_mat = 2*np.diag([ddelta_w, jerk_w])

        # path cost
        x = model.x # [x,y,yaw,speed,delta,accel]
        u = model.u # [ddelta,jerk]

        ocp.cost.cost_type = 'EXTERNAL'

        cost_x_expr = vertcat(x[0]-s_ref, x[1], x[2], x[3] - speed_ref, x[4], x[5])

        ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr)+ 0.5 * u.T @ mtimes(R_mat, u) 
        

        # terminal cost
        ocp.cost.cost_type_e = 'EXTERNAL'
        ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) 


        # # set constraints
        # delta_max = 0.79
        # ocp.constraints.lbx = np.array([-delta_max])
        # ocp.constraints.ubx = np.array([+delta_max])
        # ocp.constraints.idxbx = np.array([4]) 
        
        # ddelta_max = 0.1
        # jerk_max = 5
        # ocp.constraints.lbu = np.array([-ddelta_max,-jerk_max])
        # ocp.constraints.ubu = np.array([+ddelta_max,+jerk_max])
        # ocp.constraints.idxbu = np.array([0, 1])


        ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
        # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
        ocp.solver_options.qp_solver_iter_max = 100 #  Default: 50
        ocp.solver_options.hpipm_mode = "SPEED"
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4        # Runge-Kutta int. stages: (1) RK1, (2) RK2, (4) RK4
        ocp.solver_options.sim_method_num_steps = 3
        # ocp.solver_options.print_level = 1
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP
        # ocp.solver_options.globalization = 'MERIT_BACKTRACKING' # turns on globalization

        self.acados_solver = AcadosOcpSolver(ocp)

    def setup_bayesian_optimization(self):
        """베이지안 최적화 설정"""
        self.optimizer = BayesianOptimization(
            f=self.objective_function,
            pbounds={
                's_w': (0.001, 10),
                'd_w': (0.001, 10),
                'mu_w': (0.001, 10),
                'speed_w': (0.001, 10),
                'delta_w': (0.001, 10),
                'accel_w': (0.001, 10),
                'ddelta_w': (0.001, 10),
                'jerk_w': (0.001, 10)
            }
        
        )
        self.optimizer.set_gp_params(alpha=1e-4)
        self.utility = UtilityFunction(
            kind="ei",    # 'ei', 'ucb', 'poi' 중 선택
            kappa=2.576,  # UCB용 파라미터
            xi=0.1        # EI, POI용 파라미터
        )

    def calculate_trajectory_stats(self, scene_points):
        """주행 궤적의 통계 계산"""
        ax_values = []
        ay_values = []
        jx_values = []
        jy_values = []
        period = 0.05
        # 궤적 포인트들로부터 가속도와 저크 계산
        for i in range(1, len(scene_points)-1):
            # Calculate longitudinal acceleration
            ax = (scene_points[i+1].vel_mps - scene_points[i].vel_mps) / period 
            
            # Calculate lateral acceleration using velocity and curvature (ay = v^2 * κ)
            # κ = tan(δ)/L where δ is steering angle
            vx = scene_points[i].vel_mps
            delta = scene_points[i].delta
            L = 2.7  # 차량 휠베이스
            ay = (vx * vx) * np.tan(delta) / L
            abs_ay = abs(ay)
            # Calculate longitudinal jerk (derivative of ax)
            jx = (scene_points[i+1].vel_mps - 2*scene_points[i].vel_mps + scene_points[i-1].vel_mps) / (period * period)
            
            # Calculate lateral jerk (derivative of ay)
            ay_prev = (scene_points[i].vel_mps * scene_points[i].vel_mps) * np.tan(scene_points[i-1].delta) / L
            jy = (ay - ay_prev) / period

            ax_values.append(ax)
            ay_values.append(abs_ay)
            jx_values.append(jx)
            jy_values.append(jy)

        return {
            'ax': {'mean': np.mean(ax_values), 'std': np.std(ax_values)},
            'ay': {'mean': np.mean(ay_values), 'std': np.std(ay_values)},
            'jx': {'mean': np.mean(jx_values), 'std': np.std(jx_values)},
            'jy': {'mean': np.mean(jy_values), 'std': np.std(jy_values)}
        }

    def kl_divergence_gaussian(self, p_mean, p_std, q_mean, q_std):
        """가우시안 분포 간의 KL divergence 계산"""
        return (np.log(q_std/p_std) + 
                (p_std**2 + (p_mean - q_mean)**2)/(2*q_std**2) - 0.5)

    def objective_function(self, **weights):
        """베이지안 최적화의 목적 함수"""
        if self.personality_data is None:
            return float('-inf')

        # 모든 씬의 예측 결과를 저장할 리스트
        all_ax_values = []
        all_ay_values = []
        all_jx_values = []
        all_jy_values = []
        
        # 모든 씬 타입에 대해 MPC 시뮬레이션 수행
        all_scenes = (self.personality_data.accel_scenes + 
                     self.personality_data.decel_scenes + 
                     self.personality_data.corner_scenes)
        
        for scene in all_scenes:
            # Extract trajectory points
            traj_points = []
            for point in scene.scene_points:
                traj_points.append({
                    'x': point.x,
                    'y': point.y,
                    'speed': point.vel_mps,
                    'yaw': point.yaw,
                    'delta': point.delta,
                    'ax': point.ax
                })
            
            # Create RoadMap instance for Frenet conversion
            road_map = frenet.RoadMap(
                [p['x'] for p in traj_points],
                [p['y'] for p in traj_points]
            )
            
            # Convert trajectory to Frenet coordinates
            frenet_s, frenet_d = road_map.ToFrenet(
                [p['x'] for p in traj_points],
                [p['y'] for p in traj_points]
            )
            
            # Calculate curvature for each point
            curvature = road_map.GetCurvature(frenet_s)
            
            # Add Frenet coordinates and curvature to trajectory points
            for i, point in enumerate(traj_points):
                point['s'] = frenet_s[i]
                point['d'] = frenet_d[i]
                point['kappa'] = curvature[i]

            # MPC로 예측된 궤적 생성
            predicted_stats, _ = self.run_mpc_prediction(traj_points, weights, road_map)
            
            # 각 통계값을 리스트에 추가
            all_ax_values.append(predicted_stats['ax'])
            all_ay_values.append(predicted_stats['ay'])
            all_jx_values.append(predicted_stats['jx'])
            all_jy_values.append(predicted_stats['jy'])
        
        # 전체 씬에 대한 통계 계산
        # Extract mean values from dictionaries
        predicted_ax_mean = np.mean([x['mean'] for x in all_ax_values])
        predicted_ax_std = np.mean([x['std'] for x in all_ax_values])
        predicted_ay_mean = np.mean([x['mean'] for x in all_ay_values]) 
        predicted_ay_std = np.mean([x['std'] for x in all_ay_values])
        predicted_jx_mean = np.mean([x['mean'] for x in all_jx_values])
        predicted_jx_std = np.mean([x['std'] for x in all_jx_values])
        predicted_jy_mean = np.mean([x['mean'] for x in all_jy_values])
        predicted_jy_std = np.mean([x['std'] for x in all_jy_values])
        
        # KL divergence 계산
        div_ax = self.kl_divergence_gaussian(
            predicted_ax_mean, predicted_ax_std,
            self.personality_data.ax.avg, self.personality_data.ax.std_dev
        )
        div_ay = self.kl_divergence_gaussian(
            predicted_ay_mean, predicted_ay_std,
            abs(self.personality_data.ay.avg), self.personality_data.ay.std_dev
        )
        div_jx = self.kl_divergence_gaussian(
            predicted_jx_mean, predicted_jx_std,
            self.personality_data.jx.avg, self.personality_data.jx.std_dev
        )
        div_jy = self.kl_divergence_gaussian(
            predicted_jy_mean, predicted_jy_std,
            self.personality_data.jy.avg, self.personality_data.jy.std_dev
        )
        
        pdf = (predicted_ax_mean, predicted_ax_std,
                            predicted_ay_mean, predicted_ay_std,
                            predicted_jx_mean, predicted_jx_std,
                            predicted_jy_mean, predicted_jy_std)
        div = (div_ax, div_ay, div_jx, div_jy)

        current_infos = {
            'weights': weights,
            'pdf': pdf,
            'div': div
        }
        self.iteration_infos.append(current_infos)
        # print("div_ax", div_ax, "div_ay", div_ay, "div_jx", div_jx, "div_jy", div_jy)
        total_divergence = div_ax + div_ay + div_jx + div_jy
        # 무한대나 너무 큰 값이 있는 경우 처리

        if np.isinf(total_divergence) or total_divergence > 1e10:
            return -1e10
        return -total_divergence  # 최소화 문제로 변환
    


    def run_mpc_prediction(self, traj_points, weights, road_map, visualize=False):
        """주어진 가중치로 MPC 예측 수행"""
        # MPC 예측 로직 구현
        import matplotlib.pyplot as plt

        
        # 초기 상태 설정
        x0 = np.array([0.0, 0.0, 0.0, traj_points[0]['speed'], traj_points[0]['delta'], traj_points[0]['ax']])
        
        self.acados_solver.constraints_set(0,"lbx", x0)
        self.acados_solver.constraints_set(0,"ubx", x0)
    
        ## 5차 다항식 피팅
        num_points = len(traj_points)
        fit_weights = np.linspace(1.0, 0.1, num_points)  # 선형적으로 감소하는 가중치
        if len(traj_points) >= 6:
            s_values = np.array([point['s'] for point in traj_points])
            kappa_values = np.array([point['kappa'] for point in traj_points])
            coefficients = np.polyfit(s_values, kappa_values, 5, w=fit_weights)

        for i in range(len(traj_points)):
            x_guess = np.array([traj_points[i]['s'], 0.0, 0.0, traj_points[i]['speed'], 0.0, 0.0])
            self.acados_solver.set(i, "x", x_guess)
            p = np.array([traj_points[i]['s'], traj_points[i]['speed'], 
                          weights['s_w'], weights['d_w'], weights['mu_w'], weights['speed_w'], 
                          weights['delta_w'], weights['accel_w'], weights['ddelta_w'], weights['jerk_w'],
                          coefficients[0], coefficients[1], coefficients[2], coefficients[3], coefficients[4], coefficients[5]])
            self.acados_solver.set(i, "p", p)

        status = self.acados_solver.solve()
        
        pred_X = np.zeros(( len(traj_points) , 6)) # initialize empty array with shape (0,6)
        predicted_scene = PersonalityScene()
        if status == 0:
            # Create lists for batch conversion
            s_list = []
            d_list = []
            for j in range(len(traj_points)):
                x = self.acados_solver.get(j,"x")
                x = np.array(x)
                pred_X[j, :] = x
                s_list.append(x[0])  # s coordinate
                d_list.append(x[1])  # d coordinate

            # Batch conversion from Frenet to Cartesian
            cartesian_x, cartesian_y = road_map.ToCartesian(s_list, d_list)
            # Get path heading at each s coordinate
            path_yaws = road_map.GetYaw(s_list)
            
            # Get initial yaw from actual scene
            initial_yaw = traj_points[0]['yaw']

            for j in range(len(traj_points)):
                point = PersonalityScenePoint()
                point.x = cartesian_x[j]
                point.y = cartesian_y[j]
                
                # Convert relative heading (mu) to absolute heading (yaw)
                yaw = path_yaws[j] + pred_X[j, 2]
                
                # Unwrap yaw angle starting from the actual scene's initial yaw
                if j == 0:
                    # First point: adjust to match initial yaw
                    yaw = initial_yaw + np.unwrap([initial_yaw, yaw])[1] - initial_yaw
                else:
                    # Subsequent points: maintain continuity
                    prev_yaw = predicted_scene.scene_points[j-1].yaw
                    yaw = prev_yaw + np.unwrap([prev_yaw, yaw])[1] - prev_yaw
                
                point.yaw = yaw
                point.vel_mps = pred_X[j, 3]
                point.delta = pred_X[j, 4]
                point.ax = pred_X[j, 5]
                predicted_scene.scene_points.append(point)


        # 예측 경로 저장
        prediction_path = {
            'x': [point.x for point in predicted_scene.scene_points],
            'y': [point.y for point in predicted_scene.scene_points]
        }        
        if(visualize):
            # Visualization
            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))
            
            # Reference path plot
            ref_x = [point['x'] for point in traj_points]
            ref_y = [point['y'] for point in traj_points]
            ax1.plot(ref_x, ref_y, 'b-', label='Reference Path')
            ax1.plot(ref_x, ref_y, 'bo', markersize=4) # Reference points
            
            # MPC prediction path plot
            pred_x = [point.x for point in predicted_scene.scene_points]
            pred_y = [point.y for point in predicted_scene.scene_points]
            ax1.plot(pred_x, pred_y, 'r--', label='MPC Prediction')
            ax1.plot(pred_x, pred_y, 'r+', markersize=6) # MPC prediction points
            
            ax1.set_title('Reference Path vs MPC Prediction')
            ax1.set_xlabel('X [m]')
            ax1.set_ylabel('Y [m]')
            ax1.legend()
            ax1.grid(True)
            ax1.axis('equal')

            # Yaw comparison plot
            ref_yaw = [point['yaw'] for point in traj_points]
            time_steps = np.arange(len(ref_yaw))
            ax2.plot(time_steps, np.rad2deg(ref_yaw), 'b-', label='Reference Yaw')
            ax2.plot(time_steps, np.rad2deg(ref_yaw), 'bo', markersize=4)
            
            pred_yaw = [point.yaw for point in predicted_scene.scene_points]
            ax2.plot(time_steps, np.rad2deg(pred_yaw), 'r--', label='MPC Prediction Yaw')
            ax2.plot(time_steps, np.rad2deg(pred_yaw), 'r+', markersize=6)
            ax2.set_title('Reference vs Predicted Yaw')
            ax2.set_xlabel('Time step')
            ax2.set_ylabel('Yaw [deg]')
            ax2.legend()
            ax2.grid(True)
            # Velocity comparison plot
            ref_vel = [point['speed'] * 3.6 for point in traj_points]  # m/s to km/h 변환
            pred_vel = pred_X[:, 3] * 3.6  # m/s to km/h 변환
            
            ax3.plot(time_steps, ref_vel, 'b-', label='Reference Velocity')
            ax3.plot(time_steps, ref_vel, 'bo', markersize=4)
            ax3.plot(time_steps, pred_vel, 'r--', label='MPC Prediction Velocity') 
            ax3.plot(time_steps, pred_vel, 'r+', markersize=6)
            ax3.set_title('Reference vs Predicted Velocity')
            ax3.set_xlabel('Time step')
            ax3.set_ylabel('Velocity [km/h]')
            ax3.legend()
            ax3.grid(True)

            plt.tight_layout()
            plt.show()


        predicted_stats = self.calculate_trajectory_stats(predicted_scene.scene_points)
        return predicted_stats, prediction_path

        

    def loop(self):
        rate = rospy.Rate(1.0 / self.period)
        
        while not rospy.is_shutdown():              
            rate.sleep()

    def callback_personality_time_window(self, msg):
        self.personality_data = msg
        if self.personality_data is not None:
            # Unwrap yaw angles for each scene
            for scene in self.personality_data.accel_scenes + self.personality_data.decel_scenes + self.personality_data.corner_scenes:
                yaws = np.array([point.yaw - np.pi for point in scene.scene_points])
                unwrapped_yaws = np.unwrap(yaws)
                for i, point in enumerate(scene.scene_points):
                    point.yaw = unwrapped_yaws[i]


            first_scene = self.personality_data.corner_scenes[0]
            self.reference_path = {
                'x': [p.x for p in first_scene.scene_points],
                'y': [p.y for p in first_scene.scene_points]
            }
            # Extract trajectory points
            traj_points = []
            for point in first_scene.scene_points:
                traj_points.append({
                    'x': point.x,
                    'y': point.y,
                    'speed': point.vel_mps,
                    'yaw': point.yaw,
                    'delta': point.delta,
                    'ax': point.ax
                })
            
            # Create RoadMap instance for Frenet conversion
            road_map = frenet.RoadMap(
                [p['x'] for p in traj_points],
                [p['y'] for p in traj_points]
            )
            
            # Convert trajectory to Frenet coordinates
            frenet_s, frenet_d = road_map.ToFrenet(
                [p['x'] for p in traj_points],
                [p['y'] for p in traj_points]
            )
            
            # Calculate curvature for each point
            curvature = road_map.GetCurvature(frenet_s)
            
            # Add Frenet coordinates and curvature to trajectory points
            for i, point in enumerate(traj_points):
                point['s'] = frenet_s[i]
                point['d'] = frenet_d[i]
                point['kappa'] = curvature[i]

            # optimizer 초기화 대신 새로 생성
            self.setup_bayesian_optimization()  # optimizer를 새로 생성
            # 베이지안 최적화 수행
            self.optimizer.maximize(
                init_points=10,
                n_iter=40,
                acquisition_function=self.utility  # acquisition function 전달
            )
            
            # 초기 정보와 최적 정보 가져오기
            initial_idx = 0
            initial_infos = self.iteration_infos[initial_idx]
            
            # 최적 인덱스 찾기 수정
            optimal_target = self.optimizer.max['target']
            optimal_idx = next((i for i, info in enumerate(self.iteration_infos) 
                            if -sum(info['div']) == optimal_target), 0)
            print("optimal_idx", optimal_idx)
            optimal_infos = self.iteration_infos[optimal_idx]
            print("initial_infos", initial_infos)
            print("optimal_infos", optimal_infos)
            
            
            # 초기 가중치로 예측
            _, self.initial_prediction = self.run_mpc_prediction(
                traj_points, initial_infos['weights'], road_map, visualize=True
            )
            # 최적 가중치로 예측
            _, self.optimal_prediction = self.run_mpc_prediction(
                traj_points, optimal_infos['weights'], road_map, visualize=True
            )
            
            # 경로 시각화
            self.visualize_paths()
            # 분포 시각화
            self.visualize_distributions(initial_infos, optimal_infos, self.personality_data)

            # 최적 가중치 출력
            rospy.loginfo(f"Optimal weights found: {self.optimizer.max}")
            self.iteration_infos = []

    def visualize_paths(self):
        """경로 비교 시각화"""
        if any(x is None for x in [self.initial_prediction, self.optimal_prediction, self.reference_path]):
            return
            
        plt.figure(figsize=(12, 8))
        
        # 레퍼런스 경로
        plt.plot(self.reference_path['x'], self.reference_path['y'], 
                'k-', label='Reference Path', linewidth=2)
        
        # 초기 가중치 예측
        plt.plot(self.initial_prediction['x'], self.initial_prediction['y'], 
                'b--', label='Initial Weights Prediction', linewidth=1.5)
        
        # 최적 가중치 예측
        plt.plot(self.optimal_prediction['x'], self.optimal_prediction['y'], 
                'r--', label='Optimal Weights Prediction', linewidth=1.5)
        
        plt.title('Path Comparison')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        # ROS 패키지 경로 얻기
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('personality_extraction')
        
        # 패키지 내 results 폴더에 저장
        save_path = f"{pkg_path}/results/path_comparison.png"
        
        # 디렉토리가 없으면 생성
        import os
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        plt.savefig(save_path)
        plt.close()
        
        rospy.loginfo(f"Path comparison saved to: {save_path}")
        
    def visualize_distributions(self, initial_infos, optimal_infos, personality_data):
        """가속도와 저크의 분포 비교 시각화"""
        import scipy.stats as stats
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        x_range = np.linspace(-5, 5, 1000)  # 분포 그래프용 x축 범위
        
        # 가중치 정보 문자열 생성
        initial_weights = initial_infos['weights']
        optimal_weights = optimal_infos['weights']
        weight_text = f"Initial weights:\n"
        weight_text += "\n".join([f"{k}: {v:.3f}" for k,v in initial_weights.items()])
        weight_text += f"\n\nOptimal weights:\n" 
        weight_text += "\n".join([f"{k}: {v:.3f}" for k,v in optimal_weights.items()])
        
        # Longitudinal Acceleration (ax) 분포
        ax1.plot(x_range, stats.norm.pdf(x_range, personality_data.ax.avg, personality_data.ax.std_dev),
                'k-', label='Target', linewidth=2)
        ax1.plot(x_range, stats.norm.pdf(x_range, initial_infos['pdf'][0], initial_infos['pdf'][1]),
                'b--', label='Initial', linewidth=1.5)
        ax1.plot(x_range, stats.norm.pdf(x_range, optimal_infos['pdf'][0], optimal_infos['pdf'][1]),
                'r--', label='Optimized', linewidth=1.5)
        ax1.set_title('Longitudinal Acceleration Distribution')
        ax1.set_xlabel('Acceleration (m/s²)')
        ax1.set_ylabel('Density')
        ax1.legend(loc='upper right')
        ax1.grid(True)
        # KL divergence 텍스트 추가
        ax1.text(0.05, 0.95, f'Initial KL: {initial_infos["div"][0]:.3f}\nOptimal KL: {optimal_infos["div"][0]:.3f}', 
                transform=ax1.transAxes, verticalalignment='top')
        
        # Lateral Acceleration (ay) 분포
        ax2.plot(x_range, stats.norm.pdf(x_range, abs(personality_data.ay.avg), personality_data.ay.std_dev),
                'k-', label='Target', linewidth=2)
        ax2.plot(x_range, stats.norm.pdf(x_range, initial_infos['pdf'][2], initial_infos['pdf'][3]),
                'b--', label='Initial', linewidth=1.5)
        ax2.plot(x_range, stats.norm.pdf(x_range, optimal_infos['pdf'][2], optimal_infos['pdf'][3]),
                'r--', label='Optimized', linewidth=1.5)
        ax2.set_title('Lateral Acceleration Distribution')
        ax2.set_xlabel('Acceleration (m/s²)')
        ax2.set_ylabel('Density')
        ax2.legend(loc='upper right')
        ax2.grid(True)
        # KL divergence 텍스트 추가
        ax2.text(0.05, 0.95, f'Initial KL: {initial_infos["div"][1]:.3f}\nOptimal KL: {optimal_infos["div"][1]:.3f}', 
                transform=ax2.transAxes, verticalalignment='top')
        
        # Longitudinal Jerk (jx) 분포
        ax3.plot(x_range, stats.norm.pdf(x_range, personality_data.jx.avg, personality_data.jx.std_dev),
                'k-', label='Target', linewidth=2)
        ax3.plot(x_range, stats.norm.pdf(x_range, initial_infos['pdf'][4], initial_infos['pdf'][5]),
                'b--', label='Initial', linewidth=1.5)
        ax3.plot(x_range, stats.norm.pdf(x_range, optimal_infos['pdf'][4], optimal_infos['pdf'][5]),
                'r--', label='Optimized', linewidth=1.5)
        ax3.set_title('Longitudinal Jerk Distribution')
        ax3.set_xlabel('Jerk (m/s³)')
        ax3.set_ylabel('Density')
        ax3.legend(loc='upper right')
        ax3.grid(True)
        # KL divergence 텍스트 추가
        ax3.text(0.05, 0.95, f'Initial KL: {initial_infos["div"][2]:.3f}\nOptimal KL: {optimal_infos["div"][2]:.3f}', 
                transform=ax3.transAxes, verticalalignment='top')
        
        # Lateral Jerk (jy) 분포
        ax4.plot(x_range, stats.norm.pdf(x_range, personality_data.jy.avg, personality_data.jy.std_dev),
                'k-', label='Target', linewidth=2)
        ax4.plot(x_range, stats.norm.pdf(x_range, initial_infos['pdf'][6], initial_infos['pdf'][7]),
                'b--', label='Initial', linewidth=1.5)
        ax4.plot(x_range, stats.norm.pdf(x_range, optimal_infos['pdf'][6], optimal_infos['pdf'][7]),
                'r--', label='Optimized', linewidth=1.5)
        ax4.set_title('Lateral Jerk Distribution')
        ax4.set_xlabel('Jerk (m/s³)')
        ax4.set_ylabel('Density')
        ax4.legend(loc='upper right')
        ax4.grid(True)
        # KL divergence 텍스트 추가
        ax4.text(0.05, 0.95, f'Initial KL: {initial_infos["div"][3]:.3f}\nOptimal KL: {optimal_infos["div"][3]:.3f}', 
                transform=ax4.transAxes, verticalalignment='top')
        
         # 가중치 정보를 그래프 오른쪽에 추가
        plt.subplots_adjust(right=0.85)  # 오른쪽 여백 확보
        fig.text(0.88, 0.5, weight_text,
                fontsize=8,
                va='center',
                bbox=dict(facecolor='white', 
                        edgecolor='gray', 
                        alpha=0.8))
        
        plt.tight_layout(rect=[0, 0, 0.85, 1])  # 여백 제외한 영역에 대해 tight_layout 적용
        
        # ROS 패키지 경로에 저장
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('personality_extraction')
        save_path = f"{pkg_path}/results/distribution_comparison.png"
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path)
        plt.close()
        
        rospy.loginfo(f"Distribution comparison saved to: {save_path}")
if __name__ == '__main__':
    try:
        weight_optimizer = WeightBayesOpt()
        weight_optimizer.loop()
    except rospy.ROSInterruptException:
        pass