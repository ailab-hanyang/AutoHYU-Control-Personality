#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Float32
from autohyu_msgs.msg import ControlInfo
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.cm as cm
import math

class RealtimePlotter:
    def __init__(self, K=5):
        # Initialize ROS node
        rospy.init_node('rt_plotter', anonymous=True)
        
        # Subscribe to topics
        rospy.Subscriber("hmi/con/sampled_trajectory", Float32MultiArray, self.callback_sampled_trajectory)
        rospy.Subscriber("hmi/con/target_pred", Float32MultiArray, self.callback_target_pred)
        rospy.Subscriber("/hmi/con/control_info", ControlInfo, self.callback_control_info)
        
        self.i_cross_track_error = 0.0
        self.i_yaw_error = 0.0
        
        # Publishers for each float32 data field
        self.pub_cross_track_error = rospy.Publisher("/hmi/con/cross_track_error", Float32, queue_size=10)
        self.pub_yaw_error = rospy.Publisher("/hmi/con/yaw_error", Float32, queue_size=10)
        self.pub_speed_error = rospy.Publisher("/hmi/con/speed_error", Float32, queue_size=10)
        self.pub_target_speed = rospy.Publisher("/hmi/con/target_speed", Float32, queue_size=10)
        self.pub_current_speed = rospy.Publisher("/hmi/con/current_speed", Float32, queue_size=10)
        self.pub_opt_cost = rospy.Publisher("/hmi/con/opt_cost", Float32, queue_size=10)
        self.pub_opt_time = rospy.Publisher("/hmi/con/opt_time", Float32, queue_size=10)
        self.pub_solve_time = rospy.Publisher("/hmi/con/solve_time", Float32, queue_size=10)
        self.pub_total_time = rospy.Publisher("/hmi/con/total_time", Float32, queue_size=10)
        self.pub_sqp_iter = rospy.Publisher("/hmi/con/sqp_iter", Float32, queue_size=10)
        self.pub_qp_iter = rospy.Publisher("/hmi/con/qp_iter", Float32, queue_size=10)

        self.sampled_trajectory = np.zeros((1, 7))
        self.target_pred = np.zeros((1, 8))
        self.current_state = [0, 0]  # Placeholder for the current state (x0)
        self.control_info = ControlInfo()

        # Buffer to store the last K frames
        self.K = K
        self.pred_steering_buffer = []
        self.pred_acceleration_buffer = []
        
        self.ten_sec_buf_size = 100
        self.cross_track_error_buffer = [0.0] * self.ten_sec_buf_size
        self.ct_rmse_count = 1
        self.sum_squared_cte = 0.0
        self.max_cte = 0.0
        self.yaw_error_buffer = [0.0] * self.ten_sec_buf_size
        self.yaw_rmse_count = 1
        self.sum_squared_yawerr = 0.0
        self.max_yawerr = 0.0

        # List of plot functions
        self.plot_functions = [
            self.plot_reference_vs_predicted_path, 
            self.plot_reference_vs_predicted_speed, 
            self.plot_reference_vs_predicted_yaw, 
            self.plot_predicted_steering, 
            self.plot_predicted_acceleration, 
            self.plot_alat,
            self.plot_vy,
            self.plot_yawrate,
            self.plot_ref_curvature,
            self.plot_cte,
            self.plot_yaw_error,
        ]

        # Initialize figure and axes
        self.fig, self.axes = self.create_subplots(len(self.plot_functions))

    def create_subplots(self, num_plots):
        if num_plots == 3:
            rows, cols = 3, 1
        elif num_plots in [4, 5, 6]:
            rows, cols = 3, 2
        else:
            rows = math.ceil(math.sqrt(num_plots))
            cols = math.ceil(num_plots / rows)
        fig, axes = plt.subplots(rows, cols, figsize=(10, 25))
        return fig, axes.flatten()  # Flatten to make indexing easier

    def callback_sampled_trajectory(self, msg):
        data = np.array(msg.data).reshape(-1, 7)
        self.sampled_trajectory = data

    def callback_target_pred(self, msg):
        data = np.array(msg.data).reshape(-1, 8)
        self.target_pred = data

        # Update buffers
        if len(self.pred_steering_buffer) >= self.K:
            self.pred_steering_buffer.pop(0)
        if len(self.pred_acceleration_buffer) >= self.K:
            self.pred_acceleration_buffer.pop(0)
        self.pred_steering_buffer.append(np.rad2deg(data[:, 10]))
        self.pred_acceleration_buffer.append(data[:, 15])

        

    def callback_control_info(self, msg):
        # Publish each field as a separate Float32 message
        self.pub_cross_track_error.publish(Float32(msg.cross_track_error))
        self.pub_yaw_error.publish(Float32(msg.yaw_error))
        self.pub_speed_error.publish(Float32(msg.speed_error))
        self.pub_target_speed.publish(Float32(msg.target_speed))
        self.pub_current_speed.publish(Float32(msg.current_speed))
        self.pub_opt_cost.publish(Float32(msg.opt_cost))
        self.pub_opt_time.publish(Float32(msg.opt_time))
        self.pub_solve_time.publish(Float32(msg.solve_time))
        self.pub_total_time.publish(Float32(msg.total_time))
        self.pub_sqp_iter.publish(Float32(msg.sqp_iter))
        self.pub_qp_iter.publish(Float32(msg.qp_iter))
        
        self.i_cross_track_error = msg.cross_track_error
        self.i_yaw_error = msg.yaw_error

    def animate(self, i):
        for ax in self.axes:
            ax.clear()

        for plot_function, ax in zip(self.plot_functions, self.axes):
            plot_function(ax)

    def plot_reference_vs_predicted_path(self, ax):
        if self.sampled_trajectory.size > 0 and self.target_pred.size > 0:
            ref_x = self.sampled_trajectory[:, 1]
            ref_y = self.sampled_trajectory[:, 2]
            pred_x = self.target_pred[:, 0]
            pred_y = self.target_pred[:, 1]

            ax.plot(ref_x, ref_y, '+-', color='black', label='Reference Path', markersize=5)
            ax.plot(pred_x, pred_y, 'x-', color='red', label='Predicted Path', markersize=5)
            ax.scatter(pred_x[0], pred_y[0], color='red', label='Current State')
            ax.set_title('Reference vs Predicted Path')
            ax.legend()
            ax.set_ylim([-15, 15])

    def plot_reference_vs_predicted_speed(self, ax):
        if self.sampled_trajectory.size > 0 and self.target_pred.size > 0:
            ref_speed = self.sampled_trajectory[:, 4] * 3.6
            pred_speed = self.target_pred[:, 3] * 3.6
            ax.plot(ref_speed, '+-', color='black', label='Reference Speed', markersize=5)
            ax.plot(pred_speed, 'x-', color='red', label='Predicted Speed', markersize=5)
            ax.set_title('Reference vs Predicted Speed')
            ax.legend()
            ax.set_ylim([-1, 100])

    def plot_reference_vs_predicted_yaw(self, ax):
        if self.sampled_trajectory.size > 0 and self.target_pred.size > 0:
            ref_yaw = np.rad2deg(self.sampled_trajectory[:, 3])
            pred_yaw = np.rad2deg(self.target_pred[:, 2])
            ax.plot(ref_yaw, '+-', color='black', label='Reference Yaw', markersize=5)
            ax.plot(pred_yaw, 'x-', color='red', label='Predicted Yaw', markersize=5)
            ax.set_title('Reference vs Predicted Yaw')
            ax.legend()
            ax.set_ylim([-180, 180])

    def plot_predicted_steering(self, ax):
        for k in reversed(range(len(self.pred_steering_buffer))):
            alpha = 1.0 - (k / self.K)
            color = cm.Reds(alpha)
            ax.plot(self.pred_steering_buffer[-(k+1)], 'o-', color=color, label=f'Predicted Steering {-k}', markersize=5)
        ax.set_title('Predicted Steering')
        ax.legend()
        # Set ylim if needed, e.g.:
        # ax.set_ylim([-35, 35])

    def plot_predicted_acceleration(self, ax):
        for k in reversed(range(len(self.pred_acceleration_buffer))):
            alpha = 1.0 - (k / self.K)
            color = cm.Reds(alpha)
            ax.plot(self.pred_acceleration_buffer[-(k+1)], 'o-', color=color, label=f'Predicted Acceleration {-k}', markersize=5)
        ax.set_title('Predicted Acceleration')
        ax.legend()
        # Set ylim if needed, e.g.:
        # ax.set_ylim([-5, 5])

    def plot_ref_curvature(self, ax):
        if self.sampled_trajectory.size > 0:
            ref_curv = self.sampled_trajectory[:, 5]
            ax.plot(ref_curv, '+-', color='black', label='Ref curvature', markersize=5)
            ax.set_title('Ref curvature')
            ax.legend()
            ax.set_ylim([-0.05, 0.05])


    def plot_alat(self, ax):
        if self.target_pred.size > 0:
            alat = self.target_pred[:, 3] * self.target_pred[:, 5]
            ax.plot(alat, '+-', color='black', label='Predicted alat', markersize=5)
            ax.set_title('Predicted alat')
            ax.legend()


    def plot_vy(self, ax):
        if self.target_pred.size > 0:
            vy = self.target_pred[:, 4] 
            ax.plot(vy, '+-', color='black', label='Predicted vy', markersize=5)
            ax.set_title('Predicted vy')
            ax.legend()


    def plot_yawrate(self, ax):
        if self.target_pred.size > 0:
            yawrate = self.target_pred[:, 5]
            ax.plot(yawrate, '+-', color='black', label='Predicted yawrate', markersize=5)
            ax.set_title('Predicted yawrate')
            ax.legend()
            
    def plot_cte(self, ax):
        # Update buffers
        self.cross_track_error_buffer.pop(0)
        self.cross_track_error_buffer.append(self.i_cross_track_error)
    
        # Get max
        if abs(self.max_cte) < abs(self.i_cross_track_error):
            self.max_cte = self.i_cross_track_error
    
        # Get rmse
        self.ct_rmse_count += 1
        self.sum_squared_cte += self.i_cross_track_error**2
        ct_rmse = np.sqrt(self.sum_squared_cte / self.ct_rmse_count)
        
        ax.plot(self.cross_track_error_buffer, '+-', color='black', label='Cross Track Error', markersize=5)
        ax.set_title('Cross Track Error [m]')
        ax.legend()
        ax.set_ylim([-0.4, 0.4])
        
        # Add RMSE & max text to the subplot
        ax.text(0.05, 0.35, f'RMSE: {ct_rmse:.4f}', transform=ax.transAxes, fontsize=10, verticalalignment='top')
        ax.text(0.05, 0.25, f'MAX : {self.max_cte:.4f}', transform=ax.transAxes, fontsize=10, verticalalignment='top')
        
    def plot_yaw_error(self, ax):
        # Update buffers
        self.yaw_error_buffer.pop(0)
        self.yaw_error_buffer.append(self.i_yaw_error)
        
        # Get max
        if abs(self.max_yawerr) < abs(self.i_yaw_error):
            self.max_yawerr = self.i_yaw_error
        
        # get rmse
        self.yaw_rmse_count += 1
        self.sum_squared_yawerr += self.i_yaw_error**2
        yaw_rmse = np.sqrt(self.sum_squared_yawerr / self.yaw_rmse_count)
        
        ax.plot(self.yaw_error_buffer, '+-', color='black', label='Yaw Error', markersize=5)
        ax.set_title('Yaw Error [m]')
        ax.legend()
        ax.set_ylim([-5, 5])    
        
        # Add RMSE text to the subplot
        ax.text(0.05, 0.35, f'RMSE: {yaw_rmse:.4f}', transform=ax.transAxes, fontsize=10, verticalalignment='top')
        ax.text(0.05, 0.25, f'MAX : {self.max_yawerr:.4f}', transform=ax.transAxes, fontsize=10, verticalalignment='top')

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.animate, interval=10)
        print(self.target_pred)
        plt.show()

if __name__ == '__main__':
    plotter = RealtimePlotter(K=5)
    try:
        plotter.run()
    except rospy.ROSInterruptException:
        pass
