from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import numpy as np
from casadi import *


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

def acados_settings():
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
    # standardize_cost = ((unstandardize_cost - mean) / (std_dev));
    # standardize_s = ((x[0]-s_ref)**2-1.37)/1.43
    # standardize_n = (x[1]**2-0.34)/0.32
    # standardize_mu = (x[2]**2-0.15)/0.15
    # standardize_v = ((x[3]**2-speed_ref)-13.22)/2.15
    # standardize_a = (x[4]**2-45.86)/63.22
    # standardize_delta = (x[5]**2-3.43)/4.25

    # standardize_ddelta = (x[7]**2-2.18)/2.47
    # standardize_jerk = (x[6]**2-68.23)/83.97

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

    acados_solver = AcadosOcpSolver(ocp)

    return acados_solver

