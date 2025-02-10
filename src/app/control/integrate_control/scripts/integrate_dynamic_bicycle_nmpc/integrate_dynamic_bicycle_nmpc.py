#!/usr/bin/env python3

from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan, fmod, pi, if_else, mtimes

def ode_model() -> AcadosModel:
    model_name = 'integrate_dynamic_bicycle_nmpc'

    # constants

    lf = 1.49    # 전축에서 무게중심까지의 거리 [m]
    lr = 1.51    # 후축에서 무게중심까지의 거리 [m]
    L = lf + lr # 차량의 휠베이스 [m]

    m = 2300    # 차량 질량 [kg]
    Iz = 4600   # 차량 관성 모멘트 [kg·m²]

    ro = 1.225          # 공기 밀도 [kg/m³]
    Area = 2.8          # 전면 면적 [m²]
    Cd = 0.35           # 항력 계수

    Bf = 10.0  # 앞바퀴 스프링 계수
    Cf = 1.3   # 앞바퀴 형태 계수
    Df = 12552.0 # 앞바퀴 피크 값
    Ef = 0.98  # 앞바퀴 곡률 계수
    
    Br = 10.0   # 뒷바퀴 스프링 계수
    Cr = 1.6     # 뒷바퀴 형태 계수
    Dr = 20552.0 # 뒷바퀴 피크 값
    Er = 0.98   # 뒷바퀴 곡률 계수

    # parameters
    
    # set up states
    x = SX.sym('x')
    y = SX.sym('y')
    yaw = SX.sym('yaw')
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    yawrate = SX.sym('yawrate')
    delta = SX.sym('delta')
    ax = SX.sym('ax')
    state = vertcat(x, y, yaw, vx, vy, yawrate, delta, ax)

    # set up control inputs
    ddelta = SX.sym('ddelta')
    jerk = SX.sym('jerk')
    input = vertcat(ddelta, jerk)

    # xdot
    x_dot       = SX.sym('x_dot')
    y_dot       = SX.sym('y_dot')
    yaw_dot     = SX.sym('yaw_dot')
    vx_dot      = SX.sym('vx_dot')
    vy_dot      = SX.sym('vy_dot')
    yawrate_dot = SX.sym('yawrate_dot')
    delta_dot   = SX.sym('delta_dot')
    ax_dot      = SX.sym('ax_dot')

    # Help variables 
    Fx_r = m * ax
    Faero = 0.5 * ro * Area * Cd * vx**2  # 항력 효과

    # Lateral Behavior
    alpha_f = ca.if_else(vx > 0.001, delta - ca.arctan((vy + L * yawrate) / vx), 0.0)
    alpha_r = ca.if_else(vx > 0.001, ca.arctan(- vy / vx), 0.0)

    # Lateral Tire Forces (Pacejka 'magic formula')
    Fy_f = Df * ca.sin(Cf * ca.arctan(Bf * alpha_f - Ef * (Bf * alpha_f - ca.arctan(Bf * alpha_f))))
    Fy_r = Dr * ca.sin(Cr * ca.arctan(Br * alpha_r - Er * (Br * alpha_r - ca.arctan(Br * alpha_r))))

    # 동역학 방정식 정의 (다이나믹 자전거 모델)
    x_dot = vx * ca.cos(yaw) - vy * ca.sin(yaw)
    y_dot = vx * ca.sin(yaw) + vy * ca.cos(yaw)
    yaw_dot = yawrate
    vx_dot = (Fx_r - Faero - Fy_f * ca.sin(delta) + m * vy * yawrate) / m
    vy_dot = (Fy_r + Fy_f * ca.cos(delta) - m * vx * yawrate) / m
    yawrate_dot = (lf * Fy_f * ca.cos(delta) - lr * Fy_r) / Iz
    delta_dot = ddelta
    ax_dot     = jerk    


    xdot = ca.vertcat(x_dot, y_dot, yaw_dot, vx_dot, vy_dot, yawrate_dot, delta_dot, ax_dot)
    
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
    # x, y, yaw, vx, vy, yawrate, delta, ax
    cost_x_expr = vertcat(x[0]-x_ref, x[1]-y_ref, x[2] - yaw_ref, x[3] - speed_ref, x[6], x[7])
    ocp.model.cost_expr_ext_cost = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr)+ 0.5 * u.T @ mtimes(R_mat, u) 


    # terminal cost
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost_e = 0.5*cost_x_expr.T @ mtimes(Q_mat, cost_x_expr) 


    # set constraints
    delta_max = 0.79
    ocp.constraints.lbx = np.array([-delta_max])
    ocp.constraints.ubx = np.array([+delta_max])
    ocp.constraints.idxbx = np.array([6]) 
    
    ddelta_max = 0.1
    jerk_max = 5
    ocp.constraints.lbu = np.array([-ddelta_max,-jerk_max])
    ocp.constraints.ubu = np.array([+ddelta_max,+jerk_max])
    ocp.constraints.idxbu = np.array([0, 1])


    ocp.constraints.x0 = np.zeros(nx)

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
    # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
    # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
    ocp.solver_options.hpipm_mode = "BALANCE"
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
