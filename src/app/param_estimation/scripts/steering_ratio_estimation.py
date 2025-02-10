import rospy

from autohyu_msgs.msg import VehicleState
from autohyu_msgs.msg import VehicleCAN

from morai_msgs.msg import EgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleStatus

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



class SteerRatioEstimation(object):
    def __init__(self):
        rospy.init_node('steer_ratio_estimation', anonymous=True)
        self.loop_period = 0.05
        self.params = {}
        self.params['simul_mode']   = rospy.get_param('param_estimate_mode')
        # self.params['lf']           = 2.168401537942515         # spec from carla cybertruck
        # self.params['lr']           = 1.906848034063998         # spec from carla cybertruck
        # self.params['lf']           = 1.6182414127459879        # spec from carla model3
        # self.params['lr']           = 1.3864081952544254        # spec from carla model3
        # self.params['lf']           = 1.3750395189625095        # spec from carla mini
        # self.params['lr']           = 1.6402329522240393        # spec from carla mini
        self.params['lf']           = 1.49         # spec from ioniq5
        self.params['lr']           = 1.51         # spec from ioniq5

        self.params['wheelbase']    = self.params['lf'] + self.params['lr']
        self.plot_max_y = 100.0

        # pub & sub
        self.s_vehicle_state = rospy.Subscriber('/app/loc/vehicle_state', VehicleState, self.callback_vehicle_state)
        self.s_vehicle_cmd = rospy.Subscriber('/bsw/can/vehicle_cmd', VehicleCAN, self.callback_vehicle_cmd)
        
        # vars
        self.i_vehicle_state = VehicleState()
        self.i_vehicle_cmd = VehicleCAN()
        

        if self.params['simul_mode'] == "morai" :
            print("[param_estimation] Checking morai steering ratio (maybe same)")
            self.i_simul_input = EgoVehicleStatus()
            self.s_simul_input = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_simul_input)
            self.plot_max_x = 50.0
        elif self.params['simul_mode'] == "carla" :
            print("[param_estimation] Checking CARLA steering ratio")
            self.i_simul_input = CarlaEgoVehicleStatus()
            self.s_simul_input = rospy.Subscriber('/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, self.callback_simul_input)
            self.plot_max_x = 1.2
        else:
            print("[param_estimation] wrong mode selection!")


        # setting plot
        self.fig, self.axis = plt.subplots()
        self.axis.grid(True, linestyle='--', linewidth=0.5, color='gray')
        self.colors = []

        self.axis.set_xlim(-self.plot_max_x, self.plot_max_x)
        self.axis.set_ylim(-self.plot_max_y, self.plot_max_y)     
        self.axis.set_title("Steering Ratio Estimation")

        
        self.input_plot_data = []
        self.estimated_delta_plot_data = []
        points_size = 5
        self.sc = self.axis.scatter(self.input_plot_data, self.estimated_delta_plot_data, s=points_size, c=self.colors, cmap='plasma')

        # 피팅된 곡선을 표시할 라인 객체 초기화
        self.line, = self.axis.plot([], [], 'r-', lw=0.5, label='steering ratio function (cubic)')
        self.axis.legend()

        # 다항식 계수를 표시할 텍스트 객체 초기화
        self.coeff_text = self.axis.text(0.05, 0.95, '', transform=self.axis.transAxes, fontsize=12, verticalalignment='top')
    

    def callback_vehicle_state(self, data):
        self.i_vehicle_state = data

    def callback_vehicle_cmd(self, data):
        self.i_vehicle_cmd = data
    
    def callback_simul_input(self, data):
        self.i_simul_input = data
        

    def update_points(self, frame):
        self.loop_idx = 0

        yaw_rate_radps = self.i_vehicle_state.yaw_vel
        vx_mps = self.i_vehicle_state.vx

        if self.params['simul_mode'] == "morai":
            input = self.i_simul_input.wheel_angle     # it's reverse in morai!!!!
        elif self.params['simul_mode'] == "carla":
            input = self.i_simul_input.control.steer
        else:
            input = 0
        
    
        # calculating estimated degree
        side_slip_rad = np.arctan2(self.i_vehicle_state.vy, self.i_vehicle_state.vx)
        if abs(vx_mps) < 0.1 or abs(np.rad2deg(side_slip_rad)) > 50.0:
            print("too low vx or slip angle too large!!!")
            input = 0.0
            delta_deg = 0.0

        else:
            # when vehicle pose is in CG
            lr = self.params['lr']
            lf = self.params['lf']
            l = lr + lf
            delta_rad = np.arctan((l/lr)*np.tan(np.arcsin((lr/vx_mps)*yaw_rate_radps)))
            delta_deg = np.rad2deg(delta_rad)
            # when vehicle pose is in rear
            # delta_deg = np.rad2deg(np.arctan(self.params['wheelbase'] * yaw_rate_radps / vx_mps))
            if input == 0.0 :
                delta_deg = 0.0

        self.input_plot_data.append(input)
        self.estimated_delta_plot_data.append(delta_deg)
        self.colors.append(len(self.input_plot_data) % 100)


        # plot datas collected
        self.sc.set_offsets(np.c_[self.input_plot_data, self.estimated_delta_plot_data])
        self.sc.set_array(np.array(self.colors))
        self.sc.set_clim([0, 100])

        if self.loop_idx % 20 == 0:
            if len(self.input_plot_data) >= 4:
                try:
                    coeffs = np.polyfit(self.input_plot_data, self.estimated_delta_plot_data, 3)
                    x_fit = np.linspace(-self.plot_max_x, self.plot_max_x, 100)
                    y_fit = np.polyval(coeffs, x_fit)

                    # 피팅된 곡선 업데이트
                    self.line.set_data(x_fit, y_fit)

                    # 계수 텍스트 업데이트
                    coeff_text = f'Coefficients:\n{coeffs[0]:.4f} x^3\n{coeffs[1]:.4f} x^2\n{coeffs[2]:.4f} x\n{coeffs[3]:.4f}'
                    self.coeff_text.set_text(coeff_text)
                except np.linalg.LinAlgError:
                    print("SVD did not converge, skipping this update.")
                    self.input_plot_data.clear()
                    self.estimated_delta_plot_data.clear()

        self.loop_idx = self.loop_idx + 1

        return self.sc , self.line, self.coeff_text

        


if __name__ == '__main__':
    node = SteerRatioEstimation()
    ani = FuncAnimation(node.fig, node.update_points, frames=None, interval = node.loop_period*1000, blit = True)
    plt.show()