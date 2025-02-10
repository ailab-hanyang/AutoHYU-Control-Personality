# Created on Tue Dec 06 11:20 2022

# Author: Baha Zarrouki (baha.zarrouki@tum.de)
# based on acados python documentation: https://docs.acados.org/python_interface/index.html

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

import scipy.linalg
import numpy as np
from casadi import vertcat, sqrt, fmod, pi, if_else, MX, interp1d, Function

"""
Source: [1] Zarrouki, Baha, Chenyang Wang, and Johannes Betz. "A stochastic nonlinear model predictive control with an uncertainty propagation horizon for autonomous vehicle motion control." arXiv preprint arXiv:2310.18753 (2023).
"""





def acados_settings(Tf, N, x0, Q, R, Qe, L1_pen, L2_pen, ax_max_interpolant, ay_max_interpolant, combined_acc_limits, model_name, model_costtype, veh_params_file = "veh_params_pred.yaml", tire_params_file= "pacejka_params.yaml", solver_generate_C_code = True, solver_build = True):

    if model_name == 'kinematic_stm':
        from models.kinematic_stm import pred_stm
    # elif model_name == 'dynamic_stm_pacejka':
    #     from models.dynamic_stm_pacejka import pred_stm

        

    # import prediction model
    pred_model, constraints = pred_stm(veh_params_file, tire_params_file)
    ocp = AcadosOcp()

    model               = AcadosModel()
    model.disc_dyn_expr = pred_model.disc_expr
    model.x             = pred_model.x
    # model.xdot          = pred_model.xdot
    model.u             = pred_model.u
    # model.z             = pred_model.z
    model.p             = pred_model.p
    model.name          = pred_model.name
    ocp.model           = model

    nx = pred_model.x.rows() 
    nu = pred_model.u.rows() 
    ny = nx + nu             
    ny_e = nx                

    ocp.dims.N = N

    """ # not used now
    x = ocp.model.x
    u = ocp.model.u
    # # adjust yaw to [0..2*pi]
    # # yaw = x[2]
    # yaw = fmod(x[2], 2*pi)
    # yaw = if_else( yaw < 0,yaw + 2*pi , yaw) # adjust for negative angles
    yaw = fmod(x[2] + pi, 2*pi) - pi

    # compute absolute velocity for cost function (as reference velocity, model states give v_lon and v_lat)
    # vel_abs = MX.sym("vel_abs")
    # vel_abs = sqrt(x[3]**2 + x[4]**2)
    vel_abs = x[3]
    """

    # cost function formulation
    ocp.cost.cost_type      = model_costtype # Cost type at intermediate shooting nodes (1 to N-1)
    ocp.cost.cost_type_e    = model_costtype # Cost type at terminal shooting node (N)

    # if model_costtype == 'LINEAR_LS':
    #     ocp.cost.W      = scipy.linalg.block_diag(Q, R)
    #     ocp.cost.W_e    = Q
        
    #     Vx = np.zeros((ny, nx))
    #     Vx[:nx, :nx] = np.eye(nx)


    #     Vu = np.zeros((ny, nu))
    #     # x y yaw vlon 
    #     # vlat yawrate 
    #     # dl d2 d1 d0 d
    #     # al a2 a1 a0 a
    #     # steerrate jerk
    #     Vu[16, 0] = 1.0
    #     Vu[17, 1] = 1.0

    #     Vx_e = np.zeros((ny_e, nx))
    #     Vx_e[:nx, :nx] = np.eye(nx)

    #     ocp.cost.Vx = Vx
    #     ocp.cost.Vu = Vu
    #     ocp.cost.Vx_e = Vx_e

    #     ocp.cost.yref = np.zeros(ny,)            
    #     ocp.cost.yref_e = np.zeros(ny_e,)


    if model_costtype == 'NONLINEAR_LS':
        unscale =1 # N / Tf
        x = ocp.model.x
        u = ocp.model.u
        p = ocp.model.p
        # # adjust yaw to [0..2*pi]
        # # yaw = x[2]
        # yaw = fmod(x[2], 2*pi)
        # yaw = if_else( yaw < 0,yaw + 2*pi , yaw) # adjust for negative angles
        yaw = fmod(x[2] + pi, 2*pi) - pi

        # compute absolute velocity for cost function (as reference velocity, model states give v_lon and v_lat)
        # vel_abs = MX.sym("vel_abs")
        # vel_abs = sqrt(x[3]**2 + x[4]**2)
        vel_abs = x[3]
        # cost function formulation
        ocp.cost.cost_type      = "NONLINEAR_LS" # Cost type at intermediate shooting nodes (1 to N-1)
        ocp.cost.cost_type_e    = "NONLINEAR_LS" # Cost type at terminal shooting node (N)
        
        ocp.model.cost_y_expr = vertcat(x[:2], yaw, vel_abs,  u) 
        ocp.model.cost_y_expr_e = vertcat(x[:2], yaw, vel_abs)

        # cost function weights
        ocp.cost.W      = 0.01 * unscale * scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e    = 0.01 * Qe / unscale

        # intial references
        ocp.cost.yref   = np.array([0, 0, 0, 0, 0, 0])
        ocp.cost.yref_e = np.array([0, 0, 0, 0])

        # intial params
        ocp.parameter_values = np.array([0])
        
    
    # personality
    # ax_min_val = -1.0
    # ax_max_val = 1.0
    # jx_min_val = -1.0
    # jx_max_val = 1.0
    # ay_max_val = 1.0
    # jy_max_val = 1.0

    # comfort
    min_ax = -21.1246076188100036
    max_ax = 1.73863077335999483
    min_jx = -2.51305210678157254
    max_jx = 1.82315158628187679
    max_ay = 1.51424212095000388
    max_jy = 1.88598123380242133

    # aggressive
    # min_ax = -22.2725038735100016
    # max_ax = 2.90828453876999538
    # min_jx = -3.2337285614179212
    # max_jx = 2.98118366066729967
    # max_ay = 4.12745539751999768
    # max_jy = 3.73689505566259994

    # min_ax = -10
    # max_ax = 1
    # min_jx = -1.5
    # max_jx = 2.98118366066729967
    # max_ay = 0.87
    # max_jy = 10.22


    """
    # comfort (adjusted)
    max_long_accel = 1.7403846153846152
    max_long_decel = -1.7403846153846152
    max_lat_accel = 1.0552884615384615
    max_long_jerk = 1.0552884615384615
    max_lat_jerk = 0.4961538461538461
    # aggressive (adjusted)
    max_long_accel = 3.0
    max_long_decel = -3.0
    max_lat_accel = 2.0
    max_long_jerk = 2.0
    max_lat_jerk = 1.0
    min_ax = max_long_decel
    max_ax = max_long_accel
    min_jx = -max_long_jerk
    max_jx = max_long_jerk
    max_ay = max_lat_accel
    max_jy = max_lat_jerk
    """

    ocp.constraints.lbx = np.array([min_ax])  # ax_min
    ocp.constraints.ubx = np.array([max_ax])   # ax_max
    ocp.constraints.idxbx = np.array([15])
    ocp.constraints.Jsbx = np.eye(1)
    
    ocp.constraints.lbu = np.array([min_jx])   # jx_min
    ocp.constraints.ubu = np.array([max_jx])    # jx_max
    ocp.constraints.idxbu = np.array([0])
    # ocp.constraints.Jsbu = np.eye(1)

    # nonlinear constraints
    # a_lat = x[3]**2*sqrt(p[0]**2)       # vlong^2 * sqrt(cuvature^2)
    # a_lat = sqrt((np.tan(x[6])/3.0)**2) * x[3]**2
    a_lat = sqrt((np.tan(x[6])/3.0)**2) * x[3]**2
    # a_lat = sqrt((np.tan(x[10])/3.0)**2) * x[3]**2
    # jerk_lat = 2*x[3]*x[6]*x[11]/3.0 + x[3]**2*u[1]/3.0  # 2*vlong*deltaf_l*a_l / 3 + vlong^2*steerrate / 3
    # jerk_lat = 2*x[3]*x[10]*x[15]/3.0 + x[3]**2*u[1]/3.0  # 2*vlong*deltaf*a / 3 + vlong^2*steerrate / 3
    jerk_lat = x[3]**2*u[1]/3.0
    ocp.model.con_h_expr = vertcat(a_lat, jerk_lat)
    ocp.constraints.lh = np.array([0.0, -max_jy])
    ocp.constraints.uh = np.array([max_ay, max_jy])
    # ocp.model.con_h_expr = vertcat(jerk_lat)
    # ocp.constraints.lh = np.array([-jy_max_val])
    # ocp.constraints.uh = np.array([jy_max_val])


    nh = ocp.model.con_h_expr.shape[0]
    ocp.constraints.Jsh = np.eye(nh)

    # order : (state, input, slack)
    # shape : nh + jsbx + jsbu = 1 + 1 + 2 = 4
    # z_1 = np.array([1, 1])  # ax (jx..not now) ay jy
    # z_2 = np.array([0.1, 0.1])    
    z_1 = np.array([10, 10, 10])  # ax (jx..not now) ay jy
    z_2 = np.array([10, 1, 1])    
    ocp.cost.Zl = z_2
    ocp.cost.Zu = z_2
    ocp.cost.zl = z_1
    ocp.cost.zu = z_1
    

    """ original settings
    ocp.constraints.lbx = pred_model.lbx
    ocp.constraints.ubx = pred_model.ubx
    ocp.constraints.idxbx = pred_model.idxbx
    ocp.constraints.lbu = pred_model.lbu
    ocp.constraints.ubu = pred_model.ubu
    ocp.constraints.idxbu = pred_model.idxbu
    """

    # set initial condition
    ocp.constraints.x0 = x0
    # set QP solver and integration
    ocp.solver_options.tf = Tf
    # ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_iter_max = 50 #  Default: 50
    ocp.solver_options.hpipm_mode = "SPEED"
    # ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    # ocp.solver_options.nlp_solver_type = "SQP"
    # ocp.solver_options.nlp_solver_max_iter = 150
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    # ocp.solver_options.regularize_method = 'CONVEXIFY'
    ocp.solver_options.integrator_type = "DISCRETE"
    ocp.solver_options.sim_method_num_stages = 4        # Runge-Kutta int. stages: (1) RK1, (2) RK2, (4) RK4
    ocp.solver_options.sim_method_num_steps = 3

    # create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + pred_model.name + ".json", generate=solver_generate_C_code, build=solver_build)#, verbose=False)

    return constraints, pred_model, acados_solver, ocp



# %% Backlog: cost function with external cost: take y_ref as parameter to vary at each shooting node
    posx_ref    = MX.sym("posx_ref")
    posy_ref    = MX.sym("posy_ref")
    yaw_ref     = MX.sym("yaw_ref")
    v_ref   = MX.sym("v_ref")
    p = vertcat(posx_ref, posy_ref, yaw_ref, v_ref)    
    ocp.model.p   = p
    ocp.parameter_values = np.array([0, 0, 0, 0]) 
    # set p in main function at each shooting node: 
    # for j in range(N):
    #     yref = np.array([current_ref_traj['pos_x'][j],current_ref_traj['pos_y'][j],current_ref_traj['ref_yaw'][j],current_ref_traj['ref_v'][j]])
    #     acados_solver.set(j, "p", yref)
    #     # for i in range(len(yref)):
    #     #     acados_solver.set_params_sparse(j, i, yref[i])
    # yref_N = np.array([current_ref_traj['pos_x'][j+1],current_ref_traj['pos_y'][j+1],current_ref_traj['ref_yaw'][j+1],current_ref_traj['ref_v'][j+1]])
    # acados_solver.set(N, "p", yref_N)
    cost_x_expr = vertcat(x[0]-posx_ref, x[1]-posy_ref, yaw - yaw_ref, vel_abs - v_ref)

    ocp.cost.cost_type      = 'EXTERNAL'    # Cost type at intermediate shooting nodes (1 to N-1)
    ocp.cost.cost_type_e    = 'EXTERNAL'    # Cost type at terminal shooting node (N)
    # cost_x_expr = = vertcat(x[:2], yaw, vel_abs,  u) 
    # ocp.model.cost_expr_ext_cost = (cost_x_expr - ocp.model.p).T @ Q @ (cost_x_expr - ocp.model.p) + u.T @ R @ u
    # ocp.model.cost_expr_ext_cost_e = (cost_x_expr - ocp.model.p).T @ Qe @ (cost_x_expr - ocp.model.p)
    ocp.model.cost_expr_ext_cost = 0.5 * cost_x_expr.T @ cs.mtimes(Q, cost_x_expr) + 0.5 * u.T @ cs.mtimes(R, u)
    ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ cs.mtimes(Q, cost_x_expr)
   
