from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import numpy as np
from casadi import *

# constants
lf = 1.49    # 전축에서 무게중심까지의 거리 [m]
lr = 1.51    # 후축에서 무게중심까지의 거리 [m]
L = lf + lr # 차량의 휠베이스 [m]

def ode_model() -> AcadosModel:
    model_name = 'integrate_kinematic_bicycle_nmpc'


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

    # s_dot    = (v * cos(mu))/(1.0-d*kappa)
    # d_dot    = v * sin(mu) 
    # mu_dot   = v * tan(delta) / (L) - kappa*s_dot
    # v_dot   = a
    # delta_dot = ddelta 
    # a_dot = jerk 
    
    
    xdot = vertcat(s_dot, d_dot, mu_dot, v_dot, delta_dot, a_dot)


    model = AcadosModel()
    model.f_expl_expr = xdot
    model.x = state
    model.u = input
    model.name = model_name
    model.p = c
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
    obj1 = SX.sym("obj1",6)
    obj2 = SX.sym("obj2",6)
    obj3 = SX.sym("obj3",6)

    c = model.p
    p = vertcat(s_ref,n_ref,mu_ref,speed_ref,
                max_speed,delta_prev,accel_prev,
                s_w,d_w,mu_w,speed_w,delta_w,accel_w,ddelta_w,jerk_w,obj_w,
                c,
                m1,c1, m2,c2, 
                obj0,obj1,obj2,obj3
                )
    
    ocp.model.p = p
    ocp.parameter_values = np.zeros(50)


    # path cost
    x = model.x # [x,y,yaw,speed,delta,accel]
    u = model.u # [ddelta,jerk]

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'


    # ## 1. reference dependent
    # cost_x_expr = vertcat(x[0]-s_ref, x[1]-n_ref, x[2]-mu_ref, x[3] - speed_ref, x[4], x[5])
    # Q_mat = 2*np.diag([s_w, d_w, mu_w, speed_w, delta_w, accel_w])
    # R_mat = 2*np.diag([ddelta_w, jerk_w])
    # ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr)+ 0.5 * u.T @ mtimes(R_mat, u) 
    # ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) 

    # ## 2. refernece n indenpendent
    # cost_x_expr = vertcat(x[1], x[2], speed_ref-x[3], x[4], x[5])
    # Q_mat = 2*np.diag([d_w, mu_w, speed_w, delta_w, accel_w])
    # R_mat = 2*np.diag([ddelta_w, jerk_w])
    # ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr)+ 0.5 * u.T @ mtimes(R_mat, u) 
    # ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) 
    
    ## 3. refernece sn indenpendent
    # target_n = if_else(n_ref > 0.5, n_ref, 0.0)
    cost_x_expr = vertcat(x[0]-s_ref, x[1]-n_ref, x[2]-mu_ref, x[3] - speed_ref, x[4] - delta_prev, x[5]- accel_prev)
    vel_cost = if_else( x[3] <  1, 1.0 , x[3])

    Q_mat = 2*np.diag([s_w, d_w, mu_w, speed_w, delta_w, accel_w])
    R_mat = 2*np.diag([ddelta_w, jerk_w])

    ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) \
                                    + 0.5 * u.T @ mtimes(R_mat, u)
                                    # + speed_w/(vel_cost *vel_cost) 
    ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) \
                                    + s_w * (Tf*max_speed+s_ref-x[0])**2 


    # set constraints
    delta_max = 0.79
    a_max = 2.0
    ocp.constraints.lbx = np.array([-1.0,-delta_max, -2*a_max])
    ocp.constraints.ubx = np.array([14.72,+delta_max,a_max])
    ocp.constraints.idxbx = np.array([3,4,5]) 
    nbx = ocp.constraints.idxbx.shape[0]
    ocp.constraints.Jsbx  = np.eye(nbx) 
    
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx 
    ocp.constraints.idxbx_e = ocp.constraints.idxbx 
    ocp.constraints.Jsbx_e  = np.eye(nbx) 



    ddelta_max = 0.1
    ocp.constraints.lbu = np.array([-ddelta_max])
    ocp.constraints.ubu = np.array([+ddelta_max])
    ocp.constraints.idxbu = np.array([0])

    ## lane boundary constraint
    yL = m1*x[0] + c1
    yR = m2*x[0] + c2
    bound = -(x[1] - yL)*(x[1] - yR)


    ## lateral accel constraint
    kappa = c[0]*x[0]**5 + c[1]*x[0]**4 + c[2]*x[0]**3 + c[3]*x[0]**2 + c[4]*x[0]+ c[5]
    a_lat = x[3]**2*sqrt(kappa**2)
    # a_lat = x[3]**2*sqrt(x[4]**2)/3.0
 
    ## object constraint
    # calculate ego center
    s_center = x[0] + lr*cos(x[2])
    n_center = x[1] + lr*sin(x[2])
    
    dist_ofront_0  = sqrt((s_center - obj0[0])**2 + (n_center - obj0[1])**2)
    dist_ocenter_0 = sqrt((s_center - obj0[2])**2 + (n_center - obj0[3])**2)
    dist_orear_0   = sqrt((s_center - obj0[4])**2 + (n_center - obj0[5])**2)

    dist_ofront_1  = sqrt((s_center - obj1[0])**2 + (n_center - obj1[1])**2)
    dist_ocenter_1 = sqrt((s_center - obj1[2])**2 + (n_center - obj1[3])**2)
    dist_orear_1   = sqrt((s_center - obj1[4])**2 + (n_center - obj1[5])**2)

    dist_ofront_2  = sqrt((s_center - obj2[0])**2 + (n_center - obj2[1])**2)
    dist_ocenter_2 = sqrt((s_center - obj2[2])**2 + (n_center - obj2[3])**2)
    dist_orear_2   = sqrt((s_center - obj2[4])**2 + (n_center - obj2[5])**2)

    dist_ofront_3  = sqrt((s_center - obj3[0])**2 + (n_center - obj3[1])**2)
    dist_ocenter_3 = sqrt((s_center - obj3[2])**2 + (n_center - obj3[3])**2)
    dist_orear_3   = sqrt((s_center - obj3[4])**2 + (n_center - obj3[5])**2)



    # safety 
    safe_dist = 3.0
       
    
    ocp.model.con_h_expr = vertcat(bound, a_lat, 
                                   dist_ofront_0, dist_ocenter_0, dist_orear_0, 
                                   dist_ofront_1, dist_ocenter_1, dist_orear_1, 
                                   dist_ofront_2, dist_ocenter_2, dist_orear_2 , 
                                   dist_ofront_3, dist_ocenter_3, dist_orear_3 
                                   )
    ocp.constraints.lh    = np.array([ 0.0 , 0.0, 
                                    safe_dist, safe_dist, safe_dist,
                                    safe_dist, safe_dist, safe_dist, 
                                    safe_dist, safe_dist, safe_dist, 
                                    safe_dist, safe_dist, safe_dist
                                    ]) 
    ocp.constraints.uh    = np.array([ 1000, 0.5, 
                                      1e9, 1e9, 1e9, 1e9,1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9, 1e9 
                                      ]) 
    nh = ocp.model.con_h_expr.shape[0]
    ocp.constraints.Jsh = np.eye(nh) 

    ocp.model.con_h_expr_e = ocp.model.con_h_expr
    ocp.constraints.lh_e = ocp.constraints.lh
    ocp.constraints.uh_e = ocp.constraints.uh
    ocp.constraints.Jsh_e = np.eye(nh) 

    z_1 = np.array([10, 100.0, 1.0, 
                    10, 1.0, 
                    1,1,1,1, 1,1,1,1, 1,1,1,1
                    ]) 
    z_2 = np.array([10, 100, 1.0,
                    10, 10, 
                    10,10,10,10, 10,10,10,10, 10,10,10,10
                    ])
    ocp.cost.Zl = z_2
    # Quadratic penalty to when the constraint is violated in upper bound in the stage cost
    ocp.cost.Zu = z_2
    # Linear penalty to when the constraint is violated in lower bound in the stage cost
    ocp.cost.zl = z_1
    # Linear penalty to when the constraint is violated in upper bound in the stage cost    
    ocp.cost.zu = z_1

    ocp.cost.Zl_e = z_2
    ocp.cost.Zu_e = z_2
    ocp.cost.zl_e = z_1
    ocp.cost.zu_e = z_1

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

    acados_solver = AcadosOcpSolver(ocp, generate=build, build=build)

    return acados_solver

