import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

import rospy

from autohyu_msgs.msg import VehicleState
from autohyu_msgs.msg import VehicleCAN

from morai_msgs.msg import EgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleStatus




class TorqueMapGeneration(object):
    def __init__(self):
        rospy.init_node('torque_map_generation', anonymous=True)
        self.loop_period = 0.05
        self.params = {}
        self.params['simul_mode']   = rospy.get_param('param_estimate_mode')

        self.params['lf']                       = rospy.get_param('param_estimation/vehicle/vehicle_front_to_cg')
        self.params['lr']                       = rospy.get_param('param_estimation/vehicle/vehicle_rear_to_cg')
        self.params['wheelbase']                = self.params['lf'] + self.params['lr']
        self.params['mass']                     = rospy.get_param('param_estimation/vehicle/mass')
        self.params['wheel_radius']             = rospy.get_param('param_estimation/vehicle/wheel_radius')
        self.params['Af']                       = rospy.get_param('param_estimation/vehicle/front_area')
        self.params['Cd']                       = rospy.get_param('param_estimation/vehicle/air_drag_coeff')
        self.params['rolling_resistance']       = rospy.get_param('param_estimation/vehicle/rolling_resistance')
        self.params['rho']                      = rospy.get_param('param_estimation/air_density')
        self.params['g']                        = rospy.get_param('param_estimation/gravity_coeff')


        # pub & sub
        # self.s_vehicle_state = rospy.Subscriber('/app/loc/vehicle_state', VehicleState, self.callback_vehicle_state)
        self.s_vehicle_state = rospy.Subscriber('/app/loc/vehicle_state_filtered', VehicleState, self.callback_vehicle_state)
        self.s_vehicle_cmd = rospy.Subscriber('/bsw/can/vehicle_cmd', VehicleCAN, self.callback_vehicle_cmd)
        
        # vars
        self.i_vehicle_state = VehicleState()
        self.i_vehicle_cmd = VehicleCAN()
        

        if self.params['simul_mode'] == "morai" :
            print("[param_estimation] Checking morai steering ratio")
            self.i_simul_input = EgoVehicleStatus()
            self.s_simul_input = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.callback_simul_input)

        # elif self.params['simul_mode'] == "carla" :
            
        else:
            print("[param_estimation] wrong mode selection!")


        # setting plot
        self.fig = plt.figure()
        self.axis = self.fig.add_subplot(111, projection='3d')

        self.plot_max_x = 7000
        self.plot_max_y = 200.0
        self.plot_max_z = 100
        self.axis.set_xlim(-2000.0, self.plot_max_x)
        self.axis.set_ylim(-10.0, self.plot_max_y)     
        self.axis.set_zlim(0.0, self.plot_max_z)
        self.axis.invert_xaxis()
        self.axis.set_xlabel("Torque [Nm]")
        self.axis.set_ylabel("Speed [kph]")
        self.axis.set_zlabel("Throttle [%]")

        self.axis.set_title("Torque Map")

        # load csv file
        load_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../resources/torque_map/torque_map_output.csv')
        if os.path.exists(load_dir):
            rospy.logwarn("previous file exists!")
            df = pd.read_csv(load_dir)
            self.torque_points_Nm = df['torque_Nm']
            self.speed_points_kph = df['speed_kph']
            self.throttle_points = df['throttle']
            self.sc = self.axis.scatter(self.torque_points_Nm, self.speed_points_kph, self.throttle_points, c='gray', s=10)
        else:
            self.torque_points_Nm = np.array([])
            self.speed_points_kph = np.array([])
            self.throttle_points = np.array([])
            self.sc = self.axis.scatter(self.torque_points_Nm, self.speed_points_kph, self.throttle_points)

        
        self.no_throttle_time = rospy.Time.now()
        self.brake_start_time = rospy.Time.now()
        self.neutral_gear_time = rospy.Time.now()
        self.prev_pitch_vel = 0.0


    def callback_vehicle_state(self, data):
        self.i_vehicle_state = data

    def callback_vehicle_cmd(self, data):
        self.i_vehicle_cmd = data
    
    def callback_simul_input(self, data):
        self.i_simul_input = data
        

    def update_points(self, frame):
        if not rospy.is_shutdown():

            pitch_vel_derivative = (self.i_vehicle_state.pitch_vel - self.prev_pitch_vel) / self.loop_period
            self.prev_pitch_vel = self.i_vehicle_state.pitch_vel
        
            if self.i_vehicle_state.vx < 3 / 3.6 :
                return self.sc, 

            else:
                f_aero = (1/2) * self.params['rho'] * self.params['Cd'] * self.params['Af'] * (self.i_vehicle_state.vx)**2
                rx = self.params['rolling_resistance'] * self.params['mass'] * self.params['g'] * np.cos(self.i_vehicle_state.pitch)
                slope_force = self.params['mass'] * self.params['g'] * np.sin(self.i_vehicle_state.pitch)
                # print(f_aero, rx, slope_force)

                estimated_torque_Nm = (self.params['mass']*self.i_vehicle_state.ax + f_aero + rx + slope_force) * self.params['wheel_radius']
                tire_slip_angle_rad = np.arctan2(self.i_vehicle_state.vy + self.params['lf']*self.i_vehicle_state.yaw_vel , self.i_vehicle_state.vx)


                vx_kph = self.i_vehicle_state.vx * 3.6
                throttle = self.i_vehicle_state.vehicle_can.accel_position
                brake = self.i_vehicle_state.vehicle_can.brake_pressure

                # print("slip_angle_deg :", abs(np.rad2deg(tire_slip_angle_rad)))
                # print("steering_tire_deg :", abs(np.rad2deg(self.i_vehicle_state.vehicle_can.steering_tire_angle)))

                if throttle < 0.0:
                    self.no_throttle_time = rospy.Time.now()

                if brake > 0.00001:
                    self.brake_start_time = rospy.Time.now()
                    return self.sc,

                if self.i_vehicle_state.vehicle_can.gear_select != 5:   # if it's not D gear
                    self.neutral_gear_time = rospy.Time.now()
                    return self.sc,

                
                now_time = rospy.Time.now()
                brake_duration_sec = rospy.Duration.to_sec(now_time - self.brake_start_time)
                n_gear_duration_sec = rospy.Duration.to_sec(now_time - self.neutral_gear_time)
                no_throttle_duration_sec = rospy.Duration.to_sec(now_time - self.no_throttle_time)
                if (brake_duration_sec < 0.5 or
                    n_gear_duration_sec < 0.5 or
                    # abs(self.i_vehicle_state.pitch) < 0.0005 or
                    # abs(self.i_vehicle_state.pitch_vel) > 0.01 or
                    abs(pitch_vel_derivative) > 0.035):
                    # no_throttle_duration_sec < 0.5):
                    return self.sc,
                

                if (brake <= 0.00001
                    and abs(np.rad2deg(tire_slip_angle_rad)) < 1.0
                    and abs(np.rad2deg(self.i_vehicle_state.vehicle_can.steering_tire_angle)) < 1.0
                    and self.i_vehicle_state.ax < 10.0 ):

                    self.torque_points_Nm = np.append(self.torque_points_Nm, estimated_torque_Nm)
                    self.speed_points_kph = np.append(self.speed_points_kph, vx_kph)
                    self.throttle_points = np.append(self.throttle_points, throttle * 100)
                    
                    color = cm.jet(self.speed_points_kph / self.plot_max_y)
                    self.axis.collections.clear()  # remove previous scatters
                    self.sc = self.axis.scatter(self.torque_points_Nm, self.speed_points_kph, self.throttle_points, c=color, s=10)

                return self.sc, 
            
        else:
            rospy.logwarn("generating csv files")
            # print(os.path.dirname(os.path.realpath(__file__)))
            save_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../../../resources/torque_map/torque_map_output.csv')
            data = np.column_stack((self.torque_points_Nm, self.speed_points_kph, self.throttle_points))
            dataframe = pd.DataFrame(data, columns=['torque_Nm', 'speed_kph', 'throttle'])
            dataframe.to_csv(save_dir, index=False)
            rospy.logwarn("csv gen complete!")
            plt.close()
            

        

if __name__ == '__main__':
    node = TorqueMapGeneration()
    ani = FuncAnimation(node.fig, node.update_points, frames=None, interval=node.loop_period*1000, blit = False)
    plt.show()