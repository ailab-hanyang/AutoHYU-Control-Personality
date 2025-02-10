from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import numpy as np
from casadi import *
import casadi as cs
from casadi import fabs

# constants
lf = 1.49    # 전축에서 무게중심까지의 거리 [m]
lr = 1.51    # 후축에서 무게중심까지의 거리 [m]
L = lf + lr # 차량의 휠베이스 [m]

IDX_X_S = 0
IDX_X_N = 1
IDX_X_MU = 2
IDX_X_SPEED = 3
IDX_X_DELTA = 4
IDX_X_ACC = 5

IDX_U_DD = 0
IDX_U_JERK = 1

def ode_model() -> AcadosModel:
    model_name = 'integrate_kinematic_bicycle_nmpc'


    # set up states
    s       = SX.sym('s')
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

    # c = SX.sym("kappa",6)
    # Curvature of the road: 5th order polynomial
    # kappa = c[0]*s**5 + c[1]*s**4 + c[2]*s**3 + c[3]*s**2 + c[4]*s+ c[5]
    kappa_list = SX.sym("kappa", 200)
    s_idx_low = cs.floor(s)  # 하한 인덱스
    s_idx_high = s_idx_low + 1  # 상한 인덱스

    # 인덱스 범위 제한
    s_idx_low = cs.fmin(cs.fmax(s_idx_low, 0), 198)
    s_idx_high = cs.fmin(cs.fmax(s_idx_high, 1), 199)

    # 보간 가중치
    alpha = s - s_idx_low

    # 선형 보간된 곡률
    # 각 인덱스에 대한 가중치 벡터 생성
    weights_low = cs.vertcat(*[(1-alpha) * (s_idx_low == i) for i in range(200)])
    weights_high = cs.vertcat(*[alpha * (s_idx_high == i) for i in range(200)])
    
    # 가중치를 적용하여 곡률 계산
    kappa = cs.mtimes(weights_low.T, kappa_list) + cs.mtimes(weights_high.T, kappa_list)
    
    # states derivatives (continuous time dynamics)
    # beta = atan2(lr*tan(delta),L)

    s_dot    = (v * cos(mu))/(1.0-d*kappa)
    d_dot    = v * sin(mu) 
    # mu_dot   = v * cos(beta) * tan(delta)/L - kappa*s_dot
    mu_dot   = v * tan(delta)/L - kappa*s_dot # Rear wheel 기준이므로 Beta 고려 X
    v_dot     = a
    delta_dot = ddelta
    a_dot     = jerk
    
    xdot = vertcat(s_dot, d_dot, mu_dot, v_dot, delta_dot, a_dot)

    model = AcadosModel()
    model.f_expl_expr = xdot
    model.x = state
    model.u = input
    model.name = model_name
    # model.kappa = c
    model.kappa_list = kappa_list
    return model

def acados_settings(build=True):
    ocp = AcadosOcp()
    # set model
    model = ode_model()

    ocp.model = model

    Tf = 4.0
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    N = 40 # number of prediction states (x0 - x39)

    # set prediction horizon
    ocp.solver_options.N_horizon = N-1 # Number of shooting intervals == number of inputs
    ocp.solver_options.tf = Tf

    
    # parameters
    s_ref = SX.sym('s_ref')
    n_ref = SX.sym('n_ref')
    mu_ref = SX.sym('mu_ref')
    speed_ref = SX.sym('speed_ref')

    max_speed = SX.sym('max_speed')
    ax_max = SX.sym('ax_max')
    ax_min = SX.sym('ax_min')
    ay_max = SX.sym('ay_max')
    super_ellipse_exponent = SX.sym('super_ellipse_exponent')

    delta_prev = SX.sym('delta_prev')
    accel_prev = SX.sym('accel_prev')

    s_w = SX.sym('s_w')
    d_w = SX.sym('d_w')
    mu_w = SX.sym('mu_w')
    speed_w = SX.sym('speed_w')
    delta_w = SX.sym('delta_w')
    accel_w = SX.sym('accel_w')
    ddelta_w = SX.sym('ddelta_w')
    jerk_w = SX.sym('jerk_w')
    obj_w = SX.sym('obj_w')

    m1 = SX.sym("m1") # Left boundary line equation: y = m1*x + c1
    c1 = SX.sym("c1")
    m2 = SX.sym("m2") # Right boundary line equation: y = m2*x + c2
    c2 = SX.sym("c2")
    obj0 = SX.sym("obj0",6)

    kappa_list = model.kappa_list
    
    # c = model.kappa
    p = vertcat(s_ref,n_ref,mu_ref,speed_ref,
                max_speed,ax_max,ax_min,ay_max,super_ellipse_exponent,
                delta_prev,accel_prev,
                s_w,d_w,mu_w,speed_w,delta_w,accel_w,ddelta_w,jerk_w,obj_w, #16
                m1,c1, m2,c2, 
                obj0,
                kappa_list
                )
    
    ocp.model.p = p

    # Dynamically determine the number of parameters
    n_params = ocp.model.p.size()[0]
    ocp.parameter_values = np.zeros(n_params)

    # path cost
    x = ocp.model.x # [s, n, mu, v, delta, acc]
    u = ocp.model.u # [ddelta,jerk]
    
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'

    ## Cost function 
    # State cost
    cost_x_expr = vertcat(x[IDX_X_N], x[IDX_X_SPEED]) #, x[IDX_X_MU])#, x[IDX_X_DELTA] - delta_prev, x[IDX_X_ACC]- accel_prev)
    Q_mat = 2*np.diag([d_w, speed_w]) #, mu_w])#, delta_w, accel_w])
    R_mat = 2*np.diag([ddelta_w, jerk_w])

    # State cost with non-linear term
    # Heading angle cost
    sin_mu = sin(x[IDX_X_MU])
    cos_mu = cos(x[IDX_X_MU])
    norm_mu = atan2(sin_mu, cos_mu)
    
    # Cost function apply
    ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) \
                                    + 0.5 * u.T @ mtimes(R_mat, u) \
                                    + mu_w * (norm_mu)**2
    ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) \
                                    + mu_w * (norm_mu)**2 \
                                    + s_w * fabs(Tf*max_speed-x[IDX_X_S])**2
                                    # (s_ref == ego_s) at the end point

    ## set constraints
    # State constraints
    delta_max = 0.79
    ocp.constraints.lbx = np.array([-1.0,-delta_max])
    ocp.constraints.ubx = np.array([14.72,+delta_max])
    ocp.constraints.idxbx = np.array([IDX_X_SPEED, IDX_X_DELTA]) # [v, delta]
    # nbx = ocp.constraints.idxbx.shape[0]
    # ocp.constraints.Jsbx  = np.eye(nbx) # If enable, it will be soft constraint
    
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx 
    ocp.constraints.idxbx_e = ocp.constraints.idxbx 
    # nbx = ocp.constraints.idxbx.shape[0]
    # ocp.constraints.Jsbx_e  = np.eye(nbx) # If enable, it will be soft constraint

    # Control input constraints
    ddelta_max = 0.1
    jerk_max = 5
    ocp.constraints.lbu = np.array([-ddelta_max, -jerk_max])
    ocp.constraints.ubu = np.array([+ddelta_max, jerk_max])
    ocp.constraints.idxbu = np.array([0, 1]) # [ddelta, jerk_x]
    # nbu = ocp.constraints.idxbu.shape[0]
    # ocp.constraints.Jsbu  = np.eye(nbu) # If enable, it will be soft constraint

    ### Nonlinear constraints
    ## Lane boundary constraint
    yL = m1*x[IDX_X_S] + c1
    yR = m2*x[IDX_X_S] + c2
    # bound = -(x[1] - yL)*(x[1] - yR)
    bound = -((x[IDX_X_N] - 1.0) * (x[IDX_X_N] + 1.0))


    ## lateral accel constraint
    # kappa = c[0]*x[0]**5 + c[1]*x[0]**4 + c[2]*x[0]**3 + c[3]*x[0]**2 + c[4]*x[0]+ c[5]
    # a_lat = x[3]**2*sqrt(kappa**2)
    a_lat = x[IDX_X_SPEED]**2*tan(sqrt(x[IDX_X_DELTA]**2))/L
    j_lat = 2*x[IDX_X_SPEED]*x[IDX_X_DELTA]*x[IDX_X_ACC]/L + (x[IDX_X_SPEED]**2)*u[IDX_U_DD]/L

    ## object constraint
    # if you want to follow front vehicle only 
    s_front = x[IDX_X_S] + L*cos(x[IDX_X_MU]) # Ego vehicle front bumper position

    # Distance from front bumper to object[front, center, rear]
    dist_ofront_0  = obj0[0] - s_front 
    dist_ocenter_0 = obj0[2] - s_front 
    dist_orear_0   = obj0[4] - s_front 

    dist_ofront_0 = if_else( dist_ofront_0 <  0, 1e9 , dist_ofront_0)
    dist_ocenter_0 = if_else( dist_ocenter_0 <  0, 1e9 , dist_ocenter_0)
    dist_orear_0 = if_else( dist_orear_0 <  0, 1e9 , dist_orear_0)

    # safety distance
    safe_dist = 3.0

    ## Friction circle constraint
    # Set friction circle constraint with super-ellipse form
    ax = x[IDX_X_ACC]  # Longitudinal acceleration
    ay = x[IDX_X_SPEED]**2*tan(sqrt(x[IDX_X_DELTA]**2))/L  # Lateral acceleration
    
    # Maximum acceleration limit
    ax_max_acc = fabs(ax_max)   # Acceleration limit (m/s^2)
    ax_max_dec = fabs(ax_min)   # Deceleration limit (m/s^2)
    ay_max = fabs(ay_max)       # Maximum lateral acceleration (m/s^2)
    
    # 초타원 지수
    n = fabs(super_ellipse_exponent) # Super-ellipse exponent: 0.6 is astroid, 1.0 is diamond, 2.0 is circle, 4.0 is square

    # ax의 절대값을 최대 가속도로 정규화 (가속/감속에 따라 다르게 적용)
    normalized_ax = if_else(ax >= 0,
                          fabs(ax/ax_max_acc),  # Acceleration
                          fabs(ax/ax_max_dec))  # Deceleration
    normalized_ay = fabs(ay/ay_max)
    
    # 초타원 제약조건
    friction_circle_constraint = fabs(normalized_ax**n + normalized_ay**n) # < 1
    # friction_circle_constraint = 0.1 # < 1

    ### Nonlinear constraints apply
    # Set constraints
    ocp.model.con_h_expr = vertcat(
                                   bound, 
                                   a_lat, j_lat,
                                   dist_ofront_0, dist_ocenter_0, dist_orear_0,
                                   friction_circle_constraint
                                   )
    # Upper bound of constraints
    ocp.constraints.lh    = np.array([
                                    0.0,
                                    -0.5, -10,
                                    safe_dist, safe_dist, safe_dist,
                                    -1.0
                                    ]) 
    # Lower bound of constraints
    ocp.constraints.uh    = np.array([
                                    1e9,
                                    10.5, 10,
                                    1e9, 1e9, 1e9,
                                    1.0
                                      ]) 
    # Set nonlinear constraint as soft constraint
    nh = ocp.model.con_h_expr.shape[0]
    ocp.constraints.Jsh = np.eye(nh)
    
    ## End point constraints
    ocp.model.con_h_expr_e = vertcat(
                                   bound,
                                   a_lat,
                                   dist_ofront_0, dist_ocenter_0, dist_orear_0,
                                   friction_circle_constraint
                                   )
    # Upper bound of constraints
    ocp.constraints.lh_e = np.array([
                                    0.0,
                                    -0.5, 
                                    safe_dist, safe_dist, safe_dist,
                                    -1.0
                                    ]) 
    # Lower bound of constraints
    ocp.constraints.uh_e = np.array([ 
                                    1e9,
                                    10.0,
                                    1e9, 1e9, 1e9,
                                    1.0
                                      ]) 
    # Set nonlinear constraint as soft constraint
    nh_e = ocp.model.con_h_expr_e.shape[0]
    ocp.constraints.Jsh_e = np.eye(nh_e) 

    ## Nonlinear constraint penalty
    z_1 = np.array([
                    1e3, # bound
                    1e2, # a_lat
                    1e1, # j_lat
                    1e3, # dist_ofront_0
                    1e3, # dist_ocenter_0
                    1e3, # dist_orear_0
                    1e3, # friction_circle_constraint
                    ]) 
    z_2 = np.array([
                    0.0, # lane boundary constraint
                    1e2, # a_lat
                    0.0, # j_lat
                    1e3, # dist_ofront_0
                    1e3, # dist_ocenter_0
                    1e3, # dist_orear_0
                    1e3 # friction_circle_constraint
                    ])
    ocp.cost.Zl = z_2
    # Quadratic penalty to when the constraint is violated in upper bound in the stage cost
    ocp.cost.Zu = z_2
    # Linear penalty to when the constraint is violated in lower bound in the stage cost
    ocp.cost.zl = z_1
    # Linear penalty to when the constraint is violated in upper bound in the stage cost    
    ocp.cost.zu = z_1

    ## End point constraints
    z_1_e = np.array([
                    1e4, # lane boundary constraint
                    0.0, # a_lat
                    1e3, # dist_ofront_0
                    1e3, # dist_ocenter_0
                    1e3, # dist_orear_0
                    1e3, # friction_circle_constraint
                    ]) 
    z_2_e = np.array([
                    0.0, # lane boundary constraint
                    0.0, # a_lat
                    1e3, # dist_ofront_0
                    1e3, # dist_ocenter_0
                    1e3, # dist_orear_0
                    1e3, # friction_circle_constraint
                    ])
    ocp.cost.Zl_e = z_2_e
    ocp.cost.Zu_e = z_2_e
    ocp.cost.zl_e = z_1_e
    ocp.cost.zu_e = z_1_e

    ## Initial condition
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    ### Solver options
    ## Solver type
    solver_type = 'SQP'  # Choose from: 'SQP_RTI', 'SQP', 'IPOPT', 'SQPMETHOD'
    
    # Basic options for all solvers
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 4        # Runge-Kutta int. stages: (1) RK1, (2) RK2, (4) RK4
    ocp.solver_options.sim_method_num_steps = 2
    
    # Solver specific options
    if solver_type == 'SQP_RTI':
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver_iter_max = 100
        ocp.solver_options.hpipm_mode = "SPEED"
        
    elif solver_type == 'SQP':
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.nlp_solver_max_iter = 10
        ocp.solver_options.qp_solver_iter_max = 50
        ocp.solver_options.globalization = 'MERIT_BACKTRACKING'
        ocp.solver_options.alpha_reduction = 0.7
        ocp.solver_options.alpha_min = 0.1
        ocp.solver_options.hpipm_mode = "ROBUST"
        
    elif solver_type == 'IPOPT':
        ocp.solver_options.nlp_solver_type = 'IPOPT'
        ocp.solver_options.nlp_solver_max_iter = 100
        ocp.solver_options.mu_init = 0.1
        ocp.solver_options.mu_target = 1e-8
        ocp.solver_options.tol = 1e-6
        ocp.solver_options.max_wall_time = 0.1
        ocp.solver_options.print_level = 0
        
    elif solver_type == 'SQPMETHOD':
        ocp.solver_options.nlp_solver_type = 'SQPMETHOD'
        ocp.solver_options.max_iter = 100
        ocp.solver_options.merit_memory = 4
        ocp.solver_options.linesearch_memory = 10
        ocp.solver_options.tol_pr = 1e-6
        ocp.solver_options.tol_du = 1e-6
        ocp.solver_options.hpipm_mode = "BALANCE"
    else:
        raise ValueError(f"Unknown solver type: {solver_type}")
    
    # Create solver
    acados_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json', generate=build, build=build)

    return acados_solver

