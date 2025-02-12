# Created on Tue Dec 06 11:20 2022

# Author: Baha Zarrouki (baha.zarrouki@tum.de)
# based on acados python documentation: https://docs.acados.org/python_interface/index.html

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import models.kinematic_stm
import numpy as np
from casadi import vertcat, sqrt, fmod, pi, if_else, MX, tan, mtimes, atan2, sin, cos, Function

"""
Source: [1] Zarrouki, Baha, Chenyang Wang, and Johannes Betz. "A stochastic nonlinear model predictive control with an uncertainty propagation horizon for autonomous vehicle motion control." arXiv preprint arXiv:2310.18753 (2023).
"""

def acados_settings(Tf, N, x0, Q, R, Qv, L1_pen, L2_pen, ax_max_interpolant, ay_max_interpolant, combined_acc_limits, veh_params_file = "veh_params_pred.yaml", tire_params_file= "pacejka_params.yaml", solver_generate_C_code = True, solver_build = True):

    ocp = AcadosOcp()

    # Import prediction models
    pred_model, constraints = models.kinematic_stm.pred_stm(veh_params_file, tire_params_file)
   
    
    # define acados ODE
    model               = AcadosModel()
    model.f_expl_expr   = pred_model.xdot
    model.x             = pred_model.x
    # model.xdot          = pred_model.xdot
    model.u             = pred_model.u
    # model.z             = pred_model.z
    # model.p             = pred_model.p
    model.name          = "kinematic_stm"
    ocp.model           = model

    nx = pred_model.x.rows() # 8
    nu = pred_model.u.rows() # 2
    ny = nx + nu             # 10
    ny_e = nx                # 8 


    ocp.dims.N = N
    unscale =1 # N / Tf
    
    # p = pred_model.p
    # ocp.model.p   = p
    # ocp.parameter_values = np.array([0])
    
    
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
    
    
    # cost function formulation
    ocp.cost.cost_type      = "EXTERNAL" # Cost type at intermediate shooting nodes (1 to N-1)
    ocp.cost.cost_type_e    = "EXTERNAL" # Cost type at terminal shooting node (N)

    posx_ref    = MX.sym("posx_ref")
    posy_ref    = MX.sym("posy_ref")
    yaw_ref     = MX.sym("yaw_ref")

    m1 = MX.sym("m1") # Left boundary line equation: y = m1*x + c1
    c1 = MX.sym("c1")
    m2 = MX.sym("m2") # Right boundary line equation: y = m2*x + c2
    c2 = MX.sym("c2")

    obj1x = MX.sym("obj1x") 
    obj1y = MX.sym("obj1y") 
                
    obj2x = MX.sym("obj2x") 
    obj2y = MX.sym("obj2y") 
                
    obj3x = MX.sym("obj3x") 
    obj3y = MX.sym("obj3y") 

    p = vertcat(posx_ref, posy_ref, yaw_ref, m1,c1, m2,c2, obj1x, obj1y, obj2x, obj2y, obj3x, obj3y)    
    ocp.model.p   = p
    ocp.parameter_values = np.array([0, 0, 0, 0, 0, 0, 0, 0,0,0,0,0,0]) 
    cost_x_expr = vertcat(x[0]-posx_ref, x[1]-posy_ref, yaw - yaw_ref)

    collision_dist = 300 - sqrt((x[0]-obj1x)**2 + (x[1]-obj1y)**2) - sqrt((x[0]-obj2x)**2 + (x[1]-obj2y)**2) - sqrt((x[0]-obj3x)**2 + (x[1]-obj3y)**2)

    ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q, cost_x_expr)+ 0.5 * u.T @ mtimes(R, u) #+ Qv*collision_dist
    ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q, cost_x_expr) # + Qv/(vel_cost *vel_cost)
    
    ## Goal vector constraint #
    nh_e = 0
    # dist = sqrt((x[0]-posx_ref)**2 + (x[1]-posy_ref)**2)
    # ocp.model.con_h_expr_e = vertcat(dist)
    # ocp.constraints.lh_e    = np.array([0.0]) 
    # ocp.constraints.uh_e    = np.array([1.0]) 
    # nh_e = ocp.model.con_h_expr_e.shape[0]
    # ocp.constraints.Jsh_e = np.eye(nh_e) 

    
    # # --- Lane boundary constraint    
    yL = m1*x[0] + c1
    yR = m2*x[0] + c2
    bound = -(x[1] - yL)*(x[1] - yR)

    # ocp.model.con_h_expr = vertcat(bound)
    # ocp.constraints.lh    = np.array([0.0]) 
    # ocp.constraints.uh    = np.array([10000.0]) 
    # nh = ocp.model.con_h_expr.shape[0]
    # ocp.constraints.Jsh = np.eye(nh) 

    # # # --- Lateral Acc constraint
    a_lat = x[3]**2*sqrt(x[6]**2)/3.0

    # # --- Obj
    obj1 = sqrt((x[0]-obj1x)**2 + (x[1]-obj1y)**2) 
    obj2 = sqrt((x[0]-obj2x)**2 + (x[1]-obj2y)**2)
    obj3 = sqrt((x[0]-obj3x)**2 + (x[1]-obj3y)**2)


    ocp.model.con_h_expr = vertcat(bound, a_lat, obj1, obj2, obj3)
    ocp.constraints.lh    = np.array([0.0,      0.0, 2.5,       2.5,       2.5  ]) 
    ocp.constraints.uh    = np.array([10000.0,  3.0, 10000.0,   10000.0,   10000.0]) 
    nh = ocp.model.con_h_expr.shape[0]
    ocp.constraints.Jsh = np.eye(nh) 


    ocp.constraints.lbx     = np.array([1, pred_model.delta_f_min, -3]) # lower bounds on x at intermediate shooting nodes (1 to N-1)
    ocp.constraints.ubx     = np.array([100,pred_model.delta_f_max, 1 ]) # upper bounds on x at intermediate shooting nodes (1 to N-1)
    ocp.constraints.idxbx   = np.array([3,6, 7])   # indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    ocp.constraints.Jsbx    = np.eye(3) 


    # Define the penalty for the slack variables
    # z_1 is the linear penalty for the slack variables in the stage cost function
    z_1 = np.array([0.1, 0.1, 0.1, 1.0, 0.1, 100.0, 100.0, 100.0]) * L1_pen
    # z_2 is the quadratic penalty for the slack variables in the stage cost function
    # z_2 = np.ones((nh+3,)) * L2_pen
    z_2 = np.array([ 0.1, 0.1, 0.1, 1.0, 0.1, 100.0, 100.0, 100.0]) * L2_pen
    ocp.cost.Zl = z_2
    # Quadratic penalty to when the constraint is violated in upper bound in the stage cost
    ocp.cost.Zu = z_2
    # Linear penalty to when the constraint is violated in lower bound in the stage cost
    ocp.cost.zl = z_1
    # Linear penalty to when the constraint is violated in upper bound in the stage cost    
    ocp.cost.zu = z_1
    # ocp.constraints.lbx_e     = np.array([0.0])
    # ocp.constraints.ubx_e     = np.array([0.0])
    # ocp.constraints.idxbx_e  = np.array([3])
    # ocp.constraints.Jsbx_e    = np.eye(1) 

    # Define the penalty for the slack variables
    # z_1 is the linear penalty for the slack variables in the terminal cost function
    z_1_e = np.ones((nh_e,)) * L1_pen
    # z_2 is the quadratic penalty for the slack variables in the terminal cost function
    z_2_e = np.ones((nh_e,)) * L2_pen
    # Add the penalty to the cost function
    # Quadratic penalty to when the constraint is violated in lower bound in the terminal cost
    ocp.cost.Zl_e = z_2_e
    # Quadratic penalty to when the constraint is violated in upper bound in the terminal cost
    ocp.cost.Zu_e = z_2_e
    # Linear penalty to when the constraint is violated in lower bound in the terminal cost
    ocp.cost.zl_e = z_1_e
    # Linear penalty to when the constraint is violated in upper bound in the terminal cost    
    ocp.cost.zu_e = z_1_e


    # # --- velocity
    # # a_lat = x[3]**2*p[1]
    # # a_long = x[7]
    # # a_lat = x[3]**2*sqrt(p[0]**2)
    # a_lat = x[3]**2*sqrt(x[6]**2)/3.0

    # # if True:
    # #     ocp.model.con_h_expr = vertcat(dist)
    # #     ocp.constraints.lh      = np.array([0.0])  # lower bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # #     ocp.constraints.uh      = np.array([1.0])  # upper bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # #     ocp.constraints.lh_e    = np.array([0.0]) 
    # #     ocp.constraints.uh_e    = np.array([1.0]) 
    # # # else:
    # ocp.model.con_h_expr = vertcat(a_lat)
    # ocp.constraints.lh      = np.array([0.0])  # lower bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # ocp.constraints.uh      = np.array([4.5])  # upper bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # ocp.constraints.lh_e    = np.array([0.0]) 
    # ocp.constraints.uh_e    = np.array([4.5]) 
    # # ocp.constraints.lh      = np.array([-2.0, -2.1])  # lower bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # ocp.constraints.uh      = np.array([2.0, 2.1])  # upper bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # ocp.constraints.lh_e    = np.array([-2.0, -2.1]) 
    # # ocp.constraints.uh_e    = np.array([2.0, 2.1]) 

    # ocp.model.con_h_expr_e = ocp.model.con_h_expr
    # nh = ocp.model.con_h_expr.shape[0]
    # # Define which one of the nonlinear constraints will be written as slack variables in the stage cost function
    # # In our case, we will use all of them
    # ocp.constraints.Jsh = np.eye(nh) 
    # # Define the penalty for the slack variables
    # # z_1 is the linear penalty for the slack variables in the stage cost function
    # z_1 = np.ones((nh+2,)) * L1_pen
    # # z_2 is the quadratic penalty for the slack variables in the stage cost function
    # z_2 = np.ones((nh+2,)) * L2_pen
    # ocp.cost.Zl = z_2
    # # Quadratic penalty to when the constraint is violated in upper bound in the stage cost
    # ocp.cost.Zu = z_2
    # # Linear penalty to when the constraint is violated in lower bound in the stage cost
    # ocp.cost.zl = z_1
    # # Linear penalty to when the constraint is violated in upper bound in the stage cost    
    # ocp.cost.zu = z_1


    # ocp.constraints.lbx     = np.array([pred_model.v_min, pred_model.delta_f_min, pred_model.a_min]) # lower bounds on x at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.ubx     = np.array([pred_model.v_max, pred_model.delta_f_max, pred_model.a_max]) # upper bounds on x at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.idxbx   = np.array([3, 6, 7])   # indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.lbx     = np.array([1.0,]) # lower bounds on x at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.ubx     = np.array([-1.0]) # upper bounds on x at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.idxbx   = np.array([1])   # indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.lbu     = np.array([pred_model.jerk_min, pred_model.delta_f_dot_min])
    # ocp.constraints.ubu     = np.array([pred_model.jerk_max, pred_model.delta_f_dot_max])
    # ocp.constraints.idxbu   = np.array([0, 1])
    

    # # # --- combined lateral and longitudinal acceleration constraints varying accroding to current velocity
    # # # Source: Wischnewski, Alexander, et al. "A tube-MPC approach to autonomous multi-vehicle racing on high-speed ovals." IEEE Transactions on Intelligent Vehicles (2022).
    # # # compute/extract acceleration constraints
    
    # # a_lat       = MX.sym("a_lat")
    # a_lon       = MX.sym("a_lon")
    # acc_limits_ineq1 = MX.sym("acc_limits_ineq1")
    # acc_limits_ineq2 = MX.sym("acc_limits_ineq2")
    # # extract current acceleration limits based on velocity
    # a_lat       = x[3] * x[5]
    # a_lon       = x[7] 
    # ay_max = ay_max_interpolant(vel_abs)
    # ax_max = ax_max_interpolant(vel_abs)
    # ax_max = if_else( a_lon < 0, -pred_model.acc_min, ax_max)  # ax limits are asymetric: >0 -> acceleration, <0: braking --> change the scaling factor for braking
    # constraints.a_lat = Function("a_lat", [x], [a_lat]) # define function for evaluation
    
    # # # ay_max = constraints.lat_acc_max
    # # # ax_max = pred_model.acc_max 

    # # # if combined_acc_limits == 0:
    # # #     # separated limits
    # # #     a_lon_limits_ineq = a_lon/ax_max
    # # #     a_lat_limits_ineq = a_lat/ay_max
    # # #     # constraints: nonlinear inequalities
    # # #     ocp.model.con_h_expr = vertcat(a_lon_limits_ineq, a_lat_limits_ineq)
    # # #     ocp.model.con_h_expr_e = vertcat(a_lon_limits_ineq, a_lat_limits_ineq)
    # # #     # nonlinear constraints
    # # #     # stage bounds for nonlinear inequalities
    # # #     ocp.constraints.lh      = np.array([-1, -1])  # lower bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # #     ocp.constraints.uh      = np.array([1, 1])  # upper bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # #     # terminal bounds for nonlinear inequalities
    # # #     ocp.constraints.lh_e    = np.array([-1, -1])  # lower bound for nonlinear inequalities at terminal shooting node N
    # # #     ocp.constraints.uh_e    = np.array([1, 1])  # upper bound for nonlinear inequalities at terminal shooting node N
    # # # elif combined_acc_limits == 1:
    # # # Diamond shaped combined lat lon acceleration limits
    # # acc_limits_ineq1 = a_lon/ax_max + a_lat/ay_max
    # # acc_limits_ineq2 = a_lon/ax_max - a_lat/ay_max
    # # # constraints: nonlinear inequalities
    # # ocp.model.con_h_expr = vertcat(acc_limits_ineq1, acc_limits_ineq2)
    # # ocp.model.con_h_expr_e = vertcat(acc_limits_ineq1, acc_limits_ineq2)
    # # # nonlinear constraints
    # # # stage bounds for nonlinear inequalities
    # # ocp.constraints.lh      = np.array([-1, -1])  # lower bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # ocp.constraints.uh      = np.array([1, 1])  # upper bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # # terminal bounds for nonlinear inequalities
    # # ocp.constraints.lh_e    = np.array([-1, -1])  # lower bound for nonlinear inequalities at terminal shooting node N
    # # ocp.constraints.uh_e    = np.array([1, 1])  # upper bound for nonlinear inequalities at terminal shooting node N
    # # else:
    # #     # Circle shaped combined lat lon acceleration limits
    # acc_limits_ineq = (a_lon/ax_max)**2 + (a_lat/ay_max)**2
    # # constraints: nonlinear inequalities
    # ocp.model.con_h_expr = vertcat(acc_limits_ineq)
    # ocp.model.con_h_expr_e = vertcat(acc_limits_ineq)
    # # nonlinear constraints
    # # stage bounds for nonlinear inequalities
    # ocp.constraints.lh      = np.array([0])  # lower bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # ocp.constraints.uh      = np.array([1])  # upper bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # terminal bounds for nonlinear inequalities
    # ocp.constraints.lh_e    = np.array([0])  # lower bound for nonlinear inequalities at terminal shooting node N
    # ocp.constraints.uh_e    = np.array([1])  # upper bound for nonlinear inequalities at terminal shooting node N

    # # # constraints.a_lon = Function("a_lon", [x], [a_lon])

    # # contraints (source reference: https://github.com/acados/acados/blob/master/docs/problem_formulation/problem_formulation_ocp_mex.pdf)
    # # linear constraints
    # # stage bounds on x
    # ocp.constraints.lbx     = np.array([pred_model.delta_f_min]) # lower bounds on x at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.ubx     = np.array([pred_model.delta_f_max]) # upper bounds on x at intermediate shooting nodes (1 to N-1)
    # ocp.constraints.idxbx   = np.array([6])   # indices of bounds on x (defines Jbx) at intermediate shooting nodes (1 to N-1)
    # # terminal bounds on x
    # ocp.constraints.lbx_e   = np.array([pred_model.delta_f_min])   # lower bounds on x at terminal shooting node N
    # ocp.constraints.ubx_e   = np.array([pred_model.delta_f_max])   # upper bounds on x at terminal shooting node N
    # ocp.constraints.idxbx_e = np.array([6])     # Indices for bounds on x at terminal shooting node N (defines Jebx)
    # # stage bounds on u 
    # # ocp.constraints.lbu     = np.array([pred_model.jerk_min, pred_model.delta_f_dot_min])
    # # ocp.constraints.ubu     = np.array([pred_model.jerk_max, pred_model.delta_f_dot_max])
    # # ocp.constraints.idxbu   = np.array([0, 1])
    # ocp.constraints.lbu     = np.array([pred_model.delta_f_dot_min])
    # ocp.constraints.ubu     = np.array([pred_model.delta_f_dot_max])
    # ocp.constraints.idxbu   = np.array([1])

    # # # ocp.constraints.lh      = np.array([constraints.lat_acc_min])  # lower bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # # ocp.constraints.uh      = np.array([constraints.lat_acc_max])  # upper bound for nonlinear inequalities at shooting nodes (0 to N-1)
    # # # # terminal bounds for nonlinear inequalities
    # # # ocp.constraints.lh_e    = np.array([constraints.lat_acc_min])  # lower bound for nonlinear inequalities at terminal shooting node N
    # # # ocp.constraints.uh_e    = np.array([constraints.lat_acc_max])  # upper bound for nonlinear inequalities at terminal shooting node N

    # # soft constraints --> help the solver to find a solution (QP solver sometimes does not find a solution with hard constraints)
    # # source: https://discourse.acados.org/t/infeasibility-issues-when-using-hard-nonlinear-constraints/1021 
    # # Slack variables are useful to relax the constraints and help the solver to find a solution
    # # Define parameters for the slack variables in the cost function
    # # L1 is the linear term and L2 is the quadratic term
    # # Tuning: if it is too small, the constraint will not be satisfied, if it is too big, it will become a hard constraint
    # # and the solver will not find a solution --> tune it to find the right balance.

    # # Slack variables for the stage cost constraints
    # # The main idea is to add a cost term in the cost function that penalizes the violation of the constraints
    # # The slack variables are added to the cost function and the constraints are relaxed (check eq 1 from link below)
    # # https://raw.githubusercontent.com/acados/acados/master/docs/problem_formulation/problem_formulation_ocp_mex.pdf

    # # Get the number of non linear constraints in the stage cost
    # nh = ocp.model.con_h_expr.shape[0]
    # # Define which one of the input constraints will be written as slack variables in the stage cost function
    # # In our case, only one input constraint is defined, so we will use the one
    # ocp.constraints.Jsbu = np.eye(1) 
    # # Define which one of the state constraints will be written as slack variables in the stage cost function
    # # In our case, only one state constraint is defined, so we will use the one
    # ocp.constraints.Jsbx = np.eye(1) 
    # # Define which one of the nonlinear constraints will be written as slack variables in the stage cost function
    # # In our case, we will use all of them
    # ocp.constraints.Jsh = np.eye(nh) 

    # # Define the penalty for the slack variables
    # # z_1 is the linear penalty for the slack variables in the stage cost function
    # z_1 = np.ones((nh + 2,)) * L1_pen
    # # z_2 is the quadratic penalty for the slack variables in the stage cost function
    # z_2 = np.ones((nh + 2,)) * L2_pen
    
    # # Add the penalty to the cost function
    # # Obs: the vectors Zl, Zu, zl, zu, Zl_e, Zu_e, zl_e, zu_e are defined as the weight of the constraints violation
    # # When we have slack variables in input, state and nonlinear constraints, the order that these variables are stacked is
    # # [input_slack, state_slack, nonlinear_slack], according to the link above, last line of page 1.
    # # Quadratic penalty to when the constraint is violated in lower bound in the stage cost
    # ocp.cost.Zl = z_2
    # # Quadratic penalty to when the constraint is violated in upper bound in the stage cost
    # ocp.cost.Zu = z_2
    # # Linear penalty to when the constraint is violated in lower bound in the stage cost
    # ocp.cost.zl = z_1
    # # Linear penalty to when the constraint is violated in upper bound in the stage cost    
    # ocp.cost.zu = z_1

    # # Quadratic penalty to when the constraint is violated in lower bound in the stage cost
    # ocp.cost.Zl_0 = np.ones((nh + 0,)) * L2_pen
    # # Quadratic penalty to when the constraint is violated in upper bound in the stage cost
    # ocp.cost.Zu_0 = np.ones((nh + 0,)) * L2_pen
    # # Linear penalty to when the constraint is violated in lower bound in the stage cost
    # ocp.cost.zl_0 = np.ones((nh + 0,)) * L1_pen
    # # Linear penalty to when the constraint is violated in upper bound in the stage cost    
    # ocp.cost.zu_0 = np.ones((nh + 0,)) * L1_pen

    # # Slack variables for the terminal cost constraints
    # # Get the number of non linear constraints in the terminal cost
    # nh_e = ocp.model.con_h_expr_e.shape[0]    
    # # Define which one of the state constraints will be written as slack variables in the terminal cost function
    # # In our case, only one state constraint is defined, so we will use the one
    # ocp.constraints.Jsbx_e = np.eye(1) 
    # # Define which one of the nonlinear constraints will be written as slack variables in the terminal cost function
    # # In our case, we will use all of them
    # ocp.constraints.Jsh_e = np.eye(nh_e) 

    # # Define the penalty for the slack variables
    # # z_1 is the linear penalty for the slack variables in the terminal cost function
    # z_1_e = np.ones((nh_e + 1,)) * L1_pen
    # # z_2 is the quadratic penalty for the slack variables in the terminal cost function
    # z_2_e = np.ones((nh_e + 1,)) * L2_pen

    # # Add the penalty to the cost function
    # # Quadratic penalty to when the constraint is violated in lower bound in the terminal cost
    # ocp.cost.Zl_e = z_2_e
    # # Quadratic penalty to when the constraint is violated in upper bound in the terminal cost
    # ocp.cost.Zu_e = z_2_e
    # # Linear penalty to when the constraint is violated in lower bound in the terminal cost
    # ocp.cost.zl_e = z_1_e
    # # Linear penalty to when the constraint is violated in upper bound in the terminal cost    
    # ocp.cost.zu_e = z_1_e



    # set initial condition
    ocp.constraints.x0 = x0
    # set QP solver and integration
    ocp.solver_options.tf = Tf
    # ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_iter_max = 50 #  Default: 50
    # ocp.solver_options.hpipm_mode = "SPEED"
    ocp.solver_options.hpipm_mode = "ROBUST"
    # ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    # ocp.solver_options.nlp_solver_type = "SQP"
    # ocp.solver_options.nlp_solver_max_iter = 150
    # ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    # ocp.solver_options.regularize_method = 'CONVEXIFY'
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4        # Runge-Kutta int. stages: (1) RK1, (2) RK2, (4) RK4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.sim_method_jac_reuse = 1

    # create solver
    file_name = "acados_ocp_"+".json"
    acados_solver = AcadosOcpSolver(ocp, json_file=file_name, generate=solver_generate_C_code, build=solver_build)#, verbose=False)
    # acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp_NMPC.json")

    return acados_solver



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
   
