#!/usr/bin/env python3

"""
acados_sn_control node
"""
# ROS Library
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from autohyu_msgs.msg import VehicleState, Trajectory, TrajectoryPoint, ControlInfo
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

# Python Library
import numpy as np
import yaml
import time
from casadi import *
import matplotlib.pyplot as plt
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from utils.controller_utils import *
from utils.trajectory_process import *

# MPC Models
from MPC.NMPC_boundary.NMPC_class_kinematic import Nonlinear_Model_Predictive_Controller as Model_Predictive_Controller

class AcadosSNControl(object):
    """
    Control With Acados
    """

    def __init__(self):
        # init ROS node
        rospy.init_node('acados_sn_control', anonymous=True)
        self.period = rospy.get_param('/task_period/period_lateral_control')

        # Subscriber
        self.s_vehicle_state = rospy.Subscriber(
            "app/loc/vehicle_state", VehicleState, self.callback_vehicle_state)
        self.is_vehicle_state = False

        self.s_trajectory = rospy.Subscriber(
            "app/pla/trajectory", Trajectory, self.callback_trajectory)
        self.is_trajectory = False

        
        # Publisher
        self.p_command_steer = rospy.Publisher(
            "app/con/command_steer", Float32, queue_size=10)
        self.p_command_torque = rospy.Publisher(
            "app/con/command_torque", Float32, queue_size=10)
        self.p_control_path = rospy.Publisher(
            "hmi/con/control_path", MarkerArray, queue_size=10)
        self.p_resample_path = rospy.Publisher(
            "hmi/con/resample_path", MarkerArray, queue_size=10)
        self.p_control_info = rospy.Publisher(
            "hmi/con/control_info", ControlInfo, queue_size=10)
        
        # Add new publishers for sampled_trajectory and target_pred
        self.p_sampled_trajectory = rospy.Publisher(
            "hmi/con/sampled_trajectory", Float32MultiArray, queue_size=10)
        self.p_target_pred = rospy.Publisher(
            "hmi/con/target_pred", Float32MultiArray, queue_size=10)

        # Input Variables
        self.i_vehicle_state = VehicleState()
        self.i_trajectory = Trajectory()

        # Output Variables
        self.o_command_steer_deg = Float32()
        self.o_command_trq_nm = Float32()
        self.o_control_path = MarkerArray()
        self.o_resample_path = MarkerArray()
        self.o_control_info = ControlInfo()
                
                
        # Predictions : Every controller's output must satisfy ... [x,y,yaw,vx,vy,r,delta,trq]
        self.N_MPC = 79
        self.Ts_MPC = 0.05
        self.target_pred = np.zeros((self.N_MPC, 6)) 
        self.sampled_trajectory = np.zeros((self.N_MPC+1, 7)) 
        
        # Initialize markers
        self.initialize_markers()

        self.is_model_kinematic = True
        self.MPC = None

        # 누적 비용을 저장할 변수들 초기화
        self.cumulative_costs = {
            's_error': [],
            'd_error': [],
            'mu_error': [],
            'v_error': [],
            'delta_error': [],
            'a_error': [],
            'ddelta': [],
            'jerk': []
        }
        
        self.cumulative_costs_z = {}

    def loop(self):
        """
        main loop
        """  
        rate = rospy.Rate(float(1. / self.period))        

        while not rospy.is_shutdown():
            total_time_start = time.time()
            
            # Wait for X0 and Xref
            if not self.is_vehicle_state:
                print(f"{time.time():.1f}", " waiting for vehicle state")
                continue
            if not self.is_trajectory or len(self.i_trajectory.point) < 1 :
                print(f"{time.time():.1f}", " waiting for trajectory")
                continue
            
            vehicle_state = self.i_vehicle_state
            trajectory = self.i_trajectory

            ego_trajectory = transform_traj_to_ego(vehicle_state, trajectory)
            frenet_n, sampled_trajectory = get_ref_trajectory_dt(ego_trajectory, self.N_MPC+1, self.Ts_MPC) # [t, x, y, yaw, speed, curvature, s]
            self.sampled_trajectory = sampled_trajectory
            if self.MPC is None:
                self.MPC = Model_Predictive_Controller()

            weights={
                's_w': 10,
                'd_w': 30,
                'mu_w': 1,
                'speed_w': 1,
                'delta_w': 1,
                'accel_w': 0.1,
                'ddelta_w': 1,
                'jerk_w': 0.01
            }

            target_cart, self.target_pred, target_input, states = self.MPC.calculate_optimal(sampled_trajectory, frenet_n, vehicle_state, weights)

            self.o_command_steer_deg.data = np.float32(target_input[0])
            self.o_command_trq_nm.data = np.float32(target_input[1])
            
            self.update_resample_path(sampled_trajectory)
            self.update_control_path(sampled_trajectory, target_cart, self.Ts_MPC)
            total_time_end = time.time()           
            total_time = 1000.0 * (total_time_end - total_time_start)
            self.update_control_info(vehicle_state, sampled_trajectory, frenet_n, total_time, states)
            # # Publish 
            self.publish()
            self.cumulative_costs_z =self.calculate_cost(sampled_trajectory, self.target_pred)
            rate.sleep()

    def destroy(self):
        """
        Destroy all objects
        """
        self.visualize_cost_distributions()


        # 모든 객체 해제
        self.p_command_steer.unregister()
        self.p_control_path.unregister()
        self.p_resample_path.unregister()
        self.p_command_torque.unregister()
        self.p_control_info.unregister()
        self.p_sampled_trajectory.unregister()
        self.p_target_pred.unregister()
        self.s_vehicle_state.unregister()
        self.s_trajectory.unregister()
        
    
    def visualize_cost_distributions(self):
        """
        각 비용의 분포를 시각화
        """
        costs = list(self.cumulative_costs.keys())
        n_costs = len(costs)
        
        # 서브플롯 구성 (2행 4열)
        fig, axes = plt.subplots(2, 4, figsize=(20, 10))
        fig.suptitle('Control Cost Distribution Analysis', fontsize=16)
        
        for idx, cost_name in enumerate(costs):
            row = idx // 4
            col = idx % 4
            ax = axes[row, col]
            
            data = self.cumulative_costs[cost_name]
            
            # Histogram and kernel density estimation
            ax.hist(data, bins=30, density=True, alpha=0.7, color='skyblue')
            
            # Calculate mean and standard deviation
            mean = np.mean(data)
            std = np.std(data)
            
            # Display statistics as text
            stats_text = f'Mean: {mean:.2f}\nStd Dev: {std:.2f}'
            ax.text(0.95, 0.95, stats_text,
                    transform=ax.transAxes,
                    verticalalignment='top',
                    horizontalalignment='right',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
            
            ax.set_title(f'{cost_name} Distribution')
            ax.set_xlabel('Cost Value')
            ax.set_ylabel('Frequency')
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # 그래프를 파일로 저장
        save_path = os.path.join(os.path.dirname(__file__), f'cost_distributions.png')
        plt.savefig(save_path)
        plt.close()
        
        # 통계 데이터를 CSV 파일로 저장
        stats_data = {
            'cost_type': [],
            'mean': [],
            'std': [],
            'min': [],
            'max': [],
            'median': []
        }
        
        for cost_name in costs:
            data = self.cumulative_costs[cost_name]
            stats_data['cost_type'].append(cost_name)
            stats_data['mean'].append(np.mean(data))
            stats_data['std'].append(np.std(data))
            stats_data['min'].append(np.min(data))
            stats_data['max'].append(np.max(data))
            stats_data['median'].append(np.median(data))
        import pandas as pd
        df = pd.DataFrame(stats_data)
        csv_path = os.path.join(os.path.dirname(__file__), f'cost_statistics_.csv')
        df.to_csv(csv_path, index=False)
        
        rospy.loginfo(f"비용 분포 시각화가 저장되었습니다: {save_path}")
        rospy.loginfo(f"비용 통계가 저장되었습니다: {csv_path}")
        
    def callback_vehicle_state(self, data):
        """
        Callback function for /vehicle_state
        """
        self.i_vehicle_state = data
        self.is_vehicle_state = True

    def callback_trajectory(self, data):
        """
        Callback function for /trajectory
        """
        self.i_trajectory = data
        self.is_trajectory = True
    

    def publish(self):
        """
        Publish Control Command and Errors 
        """
        self.p_command_steer.publish(self.o_command_steer_deg)
        self.p_command_torque.publish(self.o_command_trq_nm)
        self.p_control_path.publish(self.o_control_path)
        self.p_resample_path.publish(self.o_resample_path)
        self.p_control_info.publish(self.o_control_info)

        # Publish sampled_trajectory and target_pred
        self.publish_float32_multi_array(self.p_sampled_trajectory, self.sampled_trajectory)
        self.publish_float32_multi_array(self.p_target_pred, self.target_pred)

    def publish_float32_multi_array(self, publisher, array):
        """
        Publish numpy array as Float32MultiArray
        """
        array = np.float32(array)
        msg = Float32MultiArray()
        msg.data = array.flatten().tolist()
        publisher.publish(msg)
    
    def calculate_cost(self, sampled_trajectory, target_pred):
        # sampled_trajectory: [s, d, mu, v, delta, a]
        # target_pred: [x, y, yaw, v, delta, a, ddelta, jerk]
        
        # State error costs
        s_error = sampled_trajectory[:, 6] - target_pred[:, 0] 
        d_error = target_pred[:, 1]
        mu_error = target_pred[:, 2]
        v_error = sampled_trajectory[:, 4] - target_pred[:, 3]
        delta_error = target_pred[:, 4]
        a_error = target_pred[:, 5]
        
        # Input costs
        ddelta = target_pred[:, 6]
        jerk = target_pred[:, 7]

          
        # 현재 주기의 각 오차 누적값 계산
        s_error_sum = np.sum(np.abs(s_error))
        d_error_sum = np.sum(np.abs(d_error))
        mu_error_sum = np.sum(np.abs(mu_error))
        v_error_sum = np.sum(np.abs(v_error))
        delta_error_sum = np.sum(np.abs(delta_error))
        a_error_sum = np.sum(np.abs(a_error))
        ddelta_sum = np.sum(np.abs(ddelta))
        jerk_sum = np.sum(np.abs(jerk))
        
        # 누적 비용 리스트에 현재 값 추가
        self.cumulative_costs['s_error'].append(s_error_sum)
        self.cumulative_costs['d_error'].append(d_error_sum)
        self.cumulative_costs['mu_error'].append(mu_error_sum)
        self.cumulative_costs['v_error'].append(v_error_sum)
        self.cumulative_costs['delta_error'].append(delta_error_sum)
        self.cumulative_costs['a_error'].append(a_error_sum)
        self.cumulative_costs['ddelta'].append(ddelta_sum)
        self.cumulative_costs['jerk'].append(jerk_sum)
        
        # 전체 누적된 비용에 대한 z-score 계산
        current_costs = np.array([
            s_error_sum,
            d_error_sum,
            mu_error_sum,
            v_error_sum,
            delta_error_sum,
            a_error_sum,
            ddelta_sum,
            jerk_sum
        ])
        
        # 누적된 전체 데이터의 평균과 표준편차 계산
        historical_costs = np.array([
            self.cumulative_costs['s_error'],
            self.cumulative_costs['d_error'],
            self.cumulative_costs['mu_error'],
            self.cumulative_costs['v_error'],
            self.cumulative_costs['delta_error'],
            self.cumulative_costs['a_error'],
            self.cumulative_costs['ddelta'],
            self.cumulative_costs['jerk']
        ])
        costs_mean = np.mean(historical_costs, axis=1)
        costs_std = np.std(historical_costs, axis=1)
        z_scores = (current_costs - costs_mean) / (costs_std + 1e-10)

        
        return {
            's_error_z': {z_scores[0], costs_mean[0], costs_std[0]},
            'd_error_z': {z_scores[1], costs_mean[1], costs_std[1]},
            'mu_error_z': {z_scores[2], costs_mean[2], costs_std[2]},
            'v_error_z': {z_scores[3], costs_mean[3], costs_std[3]}, 
            'delta_error_z': {z_scores[4], costs_mean[4], costs_std[4]},
            'a_error_z': {z_scores[5], costs_mean[5], costs_std[5]},
            'ddelta_z': {z_scores[6], costs_mean[6], costs_std[6]},
            'jerk_z': {z_scores[7], costs_mean[7], costs_std[7]}
        }
    def initialize_markers(self):
        ## Messages
        # optimized control path (w/ speed)
        self.speed_marker = Marker()
        self.speed_marker.header.frame_id  = "ego_frame"
        self.speed_marker.ns = "temporal"
        self.speed_marker.id = 0
        self.speed_marker.action = Marker().ADD
        self.speed_marker.type = Marker().LINE_LIST
        self.speed_marker.lifetime = rospy.Duration(0.1)
        self.speed_marker.scale.x = 0.15
        self.speed_marker.color.r = 1.0
        self.speed_marker.color.g = 0.0
        self.speed_marker.color.b = 0.58
        self.speed_marker.color.a = 1.0

        for i in range(self.N_MPC+1): 
            pt1 = Point()
            pt2 = Point()
            self.speed_marker.points.append(pt1)
            self.speed_marker.points.append(pt2)

        self.path_marker = Marker()
        self.path_marker.header.frame_id = "ego_frame"
        self.path_marker.ns = "spartial"
        self.path_marker.id = 1
        self.path_marker.action = Marker().ADD
        self.path_marker.type = Marker().LINE_STRIP
        self.path_marker.lifetime = rospy.Duration(0.1)
        self.path_marker.scale.x = 0.25
        self.path_marker.color.r = 1.0
        self.path_marker.color.g = 0.0
        self.path_marker.color.b = 0.58
        self.path_marker.color.a = 1.0

        for i in range(self.N_MPC+1): 
            pt = Point()
            self.path_marker.points.append(pt)

        # reference trajectory control uses (w/o speed)
        self.resample_marker = MarkerArray()
        for i in range((self.N_MPC+1)*2):
            marker = Marker()
            marker.header.frame_id = "ego_frame"
            marker.action   = Marker().ADD
            marker.type     = Marker().CYLINDER
            marker.lifetime = rospy.Duration(0.1)
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.color.a = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            self.resample_marker.markers.append(marker)



    def update_resample_path(self, sampled_trajectory):
        sampled_x       = sampled_trajectory[:, 1]
        sampled_y       = sampled_trajectory[:, 2]
        sampled_yaw     = sampled_trajectory[:, 3]
        sampled_speed   = sampled_trajectory[:, 4]

        for j in range(len(sampled_x)):
            visual_yaw = sampled_yaw[j] * 7

            # self.resample_marker.markers[j].header.stamp = rospy.Time.now()
            self.resample_marker.markers[j].id = j
            self.resample_marker.markers[j].ns = 'center/yaw'
            
            self.resample_marker.markers[j].pose.position.x = sampled_x[j]
            self.resample_marker.markers[j].pose.position.y = sampled_y[j]
            self.resample_marker.markers[j].pose.position.z = visual_yaw * 0.5
            
            self.resample_marker.markers[j].scale.z = visual_yaw

            self.resample_marker.markers[j].color.r = 0.0
            self.resample_marker.markers[j].color.g = 0.0
            self.resample_marker.markers[j].color.b = 1.0
            self.resample_marker.markers[j].color.a = 0.5
 

        for i in range(len(sampled_x)):
            j = j + 1

            # self.resample_marker.markers[j].header.stamp = rospy.Time.now()
            self.resample_marker.markers[j].id = j
            self.resample_marker.markers[j].ns = 'center/speed'
            
            self.resample_marker.markers[j].pose.position.x = sampled_x[i]
            self.resample_marker.markers[j].pose.position.y = sampled_y[i]
            self.resample_marker.markers[j].pose.position.z = sampled_speed[i] * 0.27778 * 0.5
            
            self.resample_marker.markers[j].scale.z = sampled_speed[i] * 0.27778

            self.resample_marker.markers[j].color.r = 0.0
            self.resample_marker.markers[j].color.g = 1.0
            self.resample_marker.markers[j].color.b = 1.0
            self.resample_marker.markers[j].color.a = 0.5


        self.o_resample_path = self.resample_marker

    def update_control_path(self, X_ref, pred_X, dt):
        predicted_trajectory = Trajectory()
        
        N = pred_X.shape[0]  # pred_X의 첫 번째 차원 크기
        print(N)
             
        for i in range(N):
            point = TrajectoryPoint()
            point.x = pred_X[i, 0]
            point.y = pred_X[i, 1]
            point.yaw = pred_X[i, 2]
            point.speed = pred_X[i, 3] 
            predicted_trajectory.point.append(point)

        marker_array = MarkerArray()
        
        self.speed_marker.header.frame_id = "ego_frame"
        self.speed_marker.header.stamp = predicted_trajectory.header.stamp

        idx = 0
        for i in range(0, len(predicted_trajectory.point) * 2, 2):
            self.speed_marker.points[i].x = predicted_trajectory.point[idx].x
            self.speed_marker.points[i].y = predicted_trajectory.point[idx].y
            self.speed_marker.points[i].z = 0

            self.speed_marker.points[i + 1].x = predicted_trajectory.point[idx].x
            self.speed_marker.points[i + 1].y = predicted_trajectory.point[idx].y
            self.speed_marker.points[i + 1].z = predicted_trajectory.point[idx].speed * 0.27778 # kph2mps
            idx += 1
        marker_array.markers.append(self.speed_marker)

        self.path_marker.header.frame_id = "ego_frame"
        self.path_marker.header.stamp = predicted_trajectory.header.stamp

        for i in range(len(predicted_trajectory.point)):
            self.path_marker.points[i].x = predicted_trajectory.point[i].x
            self.path_marker.points[i].y = predicted_trajectory.point[i].y
            self.path_marker.points[i].z = 0.1
        marker_array.markers.append(self.path_marker)

        self.o_control_path = marker_array




    def update_control_info(self, vehicle_state, sampled_trajectory, cte, total_time, MPC_stats):
        self.o_control_info.current_speed = vehicle_state.vx * 3.6
        self.o_control_info.target_speed = sampled_trajectory[0, 4] * 3.6
        self.o_control_info.speed_error = (sampled_trajectory[0, 4] - vehicle_state.vx) * 3.6
        self.o_control_info.yaw_error = sampled_trajectory[0, 3]
        self.o_control_info.cross_track_error = cte
        self.o_control_info.total_time = total_time
        self.o_control_info.opt_cost = MPC_stats[0]
        self.o_control_info.opt_time = MPC_stats[1]
        self.o_control_info.sqp_iter = MPC_stats[2]
        self.o_control_info.qp_iter = MPC_stats[3]
        self.o_control_info.solve_time = MPC_stats[5]

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    """
    main function
    """
    acados_sn_control = AcadosSNControl()
    try:
        acados_sn_control.loop()
    finally:
        if acados_sn_control is not None:
            acados_sn_control.destroy()

if __name__ == '__main__':
    main()
