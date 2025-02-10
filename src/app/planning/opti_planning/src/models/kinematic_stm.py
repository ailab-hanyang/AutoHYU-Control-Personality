# Dynamic single track model with Pacejka's tire model
# Created on 2024-06-28 based on TUM Control

# Author: junhee.lee (998jun@gmail.com)

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
    delta_f = MX.sym("delta_f")         # steering angle
    a       = MX.sym("a")               # acceleration

    x = vertcat(posx, posy, yaw, vlong, vlat, yawrate, delta_f, a)

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
    delta_f_dot = MX.sym("delta_f_dot")
    a_dot       = MX.sym("a_dot")


    # algebraic variables
    z = vertcat([])

    # parameters
    p = vertcat([])
  
    # States Derivatives
    beta = atan2(lr * tan(delta_f), lf + lr)    # side slip angle

    posx_dot    = vlong * cos(yaw + beta)
    posy_dot    = vlong * sin(yaw + beta)
    yaw_dot     = (1.0 / (lf + lr)) * vlong*cos(beta)*tan(delta_f)
    vlong_dot   = a
    vlat_dot    = 0.0
    yawrate_dot = 0.0
    delta_f_dot = steering_rate 
    a_dot = jerk

   
    
    xdot = vertcat(posx_dot, posy_dot, yaw_dot, vlong_dot, vlat_dot, yawrate_dot, delta_f_dot, a_dot)

    f_expl = vertcat(
        posx_dot, 
        posy_dot, 
        yaw_dot, 
        vlong_dot,
        vlat_dot, 
        yawrate_dot,
        delta_f_dot,
        a_dot,
        jerk, 
        steering_rate,
    )

    ## Initial Conditions
    x0 = np.array([0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0])

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

    ## CasADi model struct
    model.f_expl_expr   = f_expl
    model.x             = x
    model.xdot          = xdot
    model.u             = u
    model.z             = z
    model.p             = p
    model.name          = model_name
    model.params        = params
    model.x0            = x0
    
    return model, constraint
