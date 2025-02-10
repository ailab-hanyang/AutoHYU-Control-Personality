#!/usr/bin/env python3

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan, atan2, fmod, pi, if_else, mtimes

def ode_model() -> AcadosModel:
    model_name = 'integrate_kinematic_bicycle_nmpc'

    # constants
    lf = 1.49    # 전축에서 무게중심까지의 거리 [m]
    lr = 1.51    # 후축에서 무게중심까지의 거리 [m]
    L = lf + lr # 차량의 휠베이스 [m]

    # set up states
    x       = SX.sym('x')
    y       = SX.sym('y')
    theta   = SX.sym('theta')
    v       = SX.sym('v')
    delta   = SX.sym('delta')
    a       = SX.sym('a')
    state = vertcat(x, y, theta, v, delta, a)

    # set up control inputs
    ddelta = SX.sym('ddelta')
    jerk = SX.sym('jerk')
    input = vertcat(ddelta, jerk)

    # xdot
    x_dot     = SX.sym('x_dot')
    y_dot     = SX.sym('y_dot')
    theta_dot = SX.sym('theta_dot')
    v_dot     = SX.sym('v_dot')
    delta_dot = SX.sym('delta_dot')
    a_dot     = SX.sym('a_dot')

    # states derivatives (continuous time dynamics)
    beta = atan2(lr*tan(delta),L)

    x_dot     = v * cos(theta)
    y_dot     = v * sin(theta)
    theta_dot = v * cos(beta) * tan(delta)/L
    v_dot     = a
    delta_dot = ddelta
    a_dot     = jerk
    
    xdot = vertcat(x_dot, y_dot, theta_dot, v_dot, delta_dot, a_dot)


    model = AcadosModel()
    model.f_expl_expr = xdot
    model.x = state
    model.u = input
    model.name = model_name

    return model

    
from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca

def main():
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
    # set model
    model = ode_model()
    ocp.model = model

    Tf = 3.0
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    N = 100 # number of prediction states (x0 - x39)

    # set prediction horizon
    ocp.solver_options.N_horizon = N-1 # Number of shooting intervals == number of inputs
    ocp.solver_options.tf = Tf



    # parameters
    x_ref = SX.sym('x_ref')
    y_ref = SX.sym('y_ref')
    yaw_ref = SX.sym('yaw_ref')
    speed_ref = SX.sym('speed_ref')
    x_w     = SX.sym('x_w')
    y_w         = SX.sym('y_w')
    yaw_w    = SX.sym('yaw_w')
    speed_w  = SX.sym('speed_w')
    delta_w  = SX.sym('delta_w')
    accle_w  = SX.sym('accle_w')
    ddelta_w = SX.sym('ddelta_w')
    jerk_w = SX.sym('jerk_w')
    p = vertcat(x_ref,y_ref,yaw_ref,speed_ref,
                x_w, y_w, yaw_w, speed_w, delta_w, accle_w, ddelta_w, jerk_w)
    ocp.model.p = p
    ocp.parameter_values = np.zeros(12)


    # cost matrices
    Q_mat = 2*np.diag([x_w, y_w, yaw_w, speed_w,delta_w, accle_w])
    R_mat = 2*np.diag([ddelta_w, jerk_w])

    # path cost
    x = model.x
    u = model.u

    ocp.cost.cost_type = 'EXTERNAL'
    cost_x_expr = vertcat(x[0]-x_ref, x[1]-y_ref, x[2] - yaw_ref, x[3] - speed_ref, x[4], x[5])
    ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr)+ 0.5 * u.T @ mtimes(R_mat, u) 


    # terminal cost
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) 


    # set constraints
    delta_max = 0.79
    ocp.constraints.lbx = np.array([-delta_max])
    ocp.constraints.ubx = np.array([+delta_max])
    ocp.constraints.idxbx = np.array([4]) 
    
    ddelta_max = 0.1
    jerk_max = 5
    ocp.constraints.lbu = np.array([-ddelta_max,-jerk_max])
    ocp.constraints.ubu = np.array([+ddelta_max,+jerk_max])
    ocp.constraints.idxbu = np.array([0, 1])


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

    ocp_solver = AcadosOcpSolver(ocp)
    simX = np.zeros((N, nx))
    simU = np.zeros((N-1, nu))
    status = ocp_solver.solve()

    # ocp_solver.print_statistics() # encapsulates: stat = ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')


    # get solution
    for i in range(N-1):
        simX[i,:] = ocp_solver.get(i, "x")
        simU[i,:] = ocp_solver.get(i, "u")
    simX[N-1,:] = ocp_solver.get(N-1, "x")


    # import matplotlib.pyplot as plt

    # plt.figure()
    # plt.plot(simX[:, 0], simX[:, 1], label='Trajectory')
    # plt.xlabel('x [m]')
    # plt.ylabel('y [m]')
    # plt.legend()
    # plt.title('Trajectory')
    # plt.grid(True)

    # plt.figure()
    # plt.step(range(N-1), simU[:, 0], where='post', label='ddelta')
    # plt.xlabel('Time Step')
    # plt.ylabel('Control Input [ddelta]')
    # plt.legend()
    # plt.title('Control Input')
    # plt.grid(True)

    # plt.show()



if __name__ == '__main__':
    main()
