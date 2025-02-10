# Dynamic single track model with Pacejka's tire model
# Created on 2024-06-28 based on TUM Control

# Author: junhee.lee (998jun@gmail.com)             - Created
#         seounghoon.park (sunghoon8585@gmail.com)  - change to ego, discrete, delayed
#           

from casadi import *
import numpy as np
from pylab import *
import yaml

# TODO: longitudinal tire forces are missing   
# TODO: consider down force!

def pred_stm(veh_params_file, tire_params_file = "pacejka_params_file.yaml"):   
    """
    This function implements a simplified dynamic bicycle model of a vehicle. 
    It uses the CasADi Model to define the states and control inputs, 
    and sets the dynamics of the vehicle using various tire and physical parameters. 
    The function can use two different models for tire behavior: Pacejka 'magic formula' 
    and AT-model. The default tire model is Pacejka 'magic formula'. 
    The function takes two required arguments: 'veh_params_file' and an optional argument
    'model_type' for the tire model. If the tire model is Pacejka, 
    it also takes an optional argument 'tire_params_file', which is set to "pacejka_params_file.yaml" by default.
    """

    # reference point: center of mass
    constraint  = types.SimpleNamespace()
    model       = types.SimpleNamespace()
    params      = types.SimpleNamespace()
    model_name  = "kinematic_stm"
    # load vehicle params
    with open(veh_params_file, 'r') as file:
        veh_params = yaml.load(file, Loader=yaml.FullLoader)
    lf  = veh_params['lf']  # distance from spring mass center of gravity to front axle [m]  LENA
    lr  = veh_params['lr']  # distance from spring mass center of gravity to rear axle [m]  LENB
      
    ## CasADi Model
    # states & control inputs
    posx    = MX.sym("posx")
    posy    = MX.sym("posy")
    yaw     = MX.sym("yaw")
    vlong   = MX.sym("vlong")
    vlat    = MX.sym("vlat")            # side slip angle
    yawrate = MX.sym("yawrate")
    delta_f_l = MX.sym("delta_f_l")
    delta_f_2 = MX.sym("delta_f_2")
    delta_f_1 = MX.sym("delta_f_1")
    delta_f_0 = MX.sym("delta_f_0")
    delta_f = MX.sym("delta_f")         # steering angle
    a_l    = MX.sym("a_l")
    a_2    = MX.sym("a_2")
    a_1    = MX.sym("a_1")
    a_0    = MX.sym("a_0")
    a      = MX.sym("a")               # acceleration

    x = vertcat(posx, posy, yaw, vlong,
                vlat, yawrate,
                delta_f_l, delta_f_2, delta_f_1, delta_f_0, delta_f,
                a_l, a_2, a_1, a_0, a)

    # controls
    jerk                = MX.sym("jerk")
    steering_rate       = MX.sym("steering_rate")
    u                   = vertcat(jerk, steering_rate)

    # xdot
    posx_dot    = MX.sym("posx_dot")
    posy_dot    = MX.sym("posy_dot")
    yaw_dot     = MX.sym("yaw_dot")
    vlong_dot   = MX.sym("vlong_dot")
    vlat_dot    = MX.sym("beta_dot")
    yawrate_dot = MX.sym("yawrate_dot")
    delta_f_l_dot = MX.sym("delta_f_l_dot")
    delta_f_dot = MX.sym("delta_f_dot")
    a_l_dot     = MX.sym("a_l_dot")
    a_dot       = MX.sym("a_dot")


    # algebraic variables
    z = vertcat([])

    # parameters
    curvature = MX.sym("curvature")
    p = vertcat(curvature)
  
    # States Derivatives
    # beta = atan2(lr * tan(delta_f), lf + lr)    # side slip angle

    posx_dot    = vlong * cos(yaw)
    posy_dot    = vlong * sin(yaw)
    yaw_dot     = (1.0 / (lf + lr)) * vlong*tan(delta_f_l)
    vlong_dot   = a_l
    vlat_dot    = 0.0
    yawrate_dot = 0.0
    delta_f_l_dot = (-delta_f_l + delta_f_2) / 0.12
    delta_f_dot = steering_rate 
    a_l_dot = (-a_l + a_2) / 0.12
    a_dot = jerk
    
    # Discretized Model (Forward Euler)
    dt = 0.05
    posx_next = posx + posx_dot * dt
    posy_next = posy + posy_dot * dt
    yaw_next = yaw + yaw_dot * dt
    vlong_next = vlong + vlong_dot * dt
    vlat_next = vlat + vlat_dot * dt
    yawrate_next = yawrate + yawrate_dot * dt
    
    delta_f_l_next = delta_f_l + delta_f_l_dot * dt
    delta_f_2_next = delta_f_1
    delta_f_1_next = delta_f_0
    delta_f_0_next = delta_f
    delta_f_next = delta_f + delta_f_dot * dt
    
    a_l_next = a_l + a_l_dot * dt
    a_2_next = a_1
    a_1_next = a_0
    a_0_next = a
    a_next = a + a_dot * dt

    x_next = vertcat(
        posx_next, 
        posy_next, 
        yaw_next, 
        vlong_next, 
        vlat_next, 
        yawrate_next,
        delta_f_l_next,
        delta_f_2_next,
        delta_f_1_next,
        delta_f_0_next,
        delta_f_next,
        a_l_next,
        a_2_next,
        a_1_next,
        a_0_next,
        a_next)
   
    disc_expr = x_next
    
    # xdot = vertcat(posx_dot, posy_dot, yaw_dot, vlong_dot, vlat_dot, yawrate_dot, delta_f_dot, a_dot)

    # f_expl = vertcat(
    #     posx_dot, 
    #     posy_dot, 
    #     yaw_dot, 
    #     vlong_dot,
    #     vlat_dot, 
    #     yawrate_dot,
    #     delta_f_dot,
    #     a_dot,
    #     jerk, 
    #     steering_rate,
    # )

    ## Initial Conditions
    x0 = np.array([0, 0, 0, 0, 
                   0, 0, 
                   0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0])

    ## constraints
    # state bounds
    model.v_min             = 0.0                          # [m/s²]
    model.v_max             = 100.0                        # [m/s²]
    model.a_min             = veh_params['acc_min']        # [m/s²]
    model.a_max             = veh_params['acc_max']        # [m/s²]
    model.jerk_min          = veh_params['jerk_min']       # [m/s³]
    model.jerk_max          = veh_params['jerk_max']       # [m/s³]
    # input bounds
    model.acc_min           = veh_params['acc_min']                 # [m/s²]
    model.acc_max           = veh_params['acc_max']                 # [m/s²]      
    model.delta_f_min       = veh_params['delta_f_min']             # steering angle [rad]
    model.delta_f_max       = veh_params['delta_f_max']             # steering angle [rad]         
    model.delta_f_dot_min   = veh_params['delta_f_dot_min']         # steering angular velocity [rad/s]
    model.delta_f_dot_max   = veh_params['delta_f_dot_max']         # steering angular velocity [rad/s] 

    model.lbx = np.array([model.v_min, model.delta_f_min, model.a_min])
    model.ubx = np.array([model.v_max, model.delta_f_max, model.a_max])
    model.idxbx = np.array([3, 10, 15])
    model.lbu = np.array([model.jerk_min, model.delta_f_dot_min])
    model.ubu = np.array([model.jerk_max, model.delta_f_dot_max])
    model.idxbu = np.array([0, 1])


    def init_state(vehicle_state, prediction, prev_empty, another_model_state = None):
        if(prev_empty == True):
            X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , 
                           0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0]) 
            print("kinematic MPC get init")
        else:
            X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx ,
                           0.0, vehicle_state.yaw_vel, 
                        #    0, 0, 0, 0, prediction[1, 10],
                        #    0, 0, 0, 0, prediction[1, 15]])
                           prediction[1, 6], prediction[1, 7], prediction[1, 8], prediction[1, 9], prediction[1,10], 
                           prediction[1, 11], prediction[1, 12], prediction[1, 13], prediction[1, 14], prediction[1, 15]])

        # if another_model_state is not None:
        #     X0 = np.array([0.0, 0.0, 0.0, vehicle_state.vx , 0.0,
        #                    vehicle_state.yaw_vel, another_model_state[1,4],
        #                    another_model_state[1,5]]) 
        return X0
    
    model.init_state = init_state

    ## CasADi model struct
    # model.f_expl_expr   = f_expl
    model.disc_expr         = disc_expr
    model.x                 = x
    model.u                 = u
    model.z                 = z
    model.p                 = p
    model.name              = model_name
    model.params            = params
    model.x0                = x0
    
    return model, constraint
