# system headers
import os
import numpy as np

# algorithm header
import casadi as cs


class VehicleModel(object):
    def __init__(self, vehicle_state, params):
        self.model_name = "Cartesian Dynamic Single Track Model (CG Frame)"
    
        ## Variables
        self.x          = cs.DM(vehicle_state.x)
        self.y          = cs.DM(vehicle_state.y)
        self.yaw        = cs.DM(vehicle_state.yaw)
        self.vx         = cs.DM(vehicle_state.vx)
        self.vy         = cs.DM(vehicle_state.vy)
        self.yawrate    = cs.DM(vehicle_state.yaw_vel)
        
        ## Define Model
        # states
        x          = cs.MX.sym("x")
        y          = cs.MX.sym("y")
        yaw        = cs.MX.sym("yaw")
        vlon       = cs.MX.sym("vx")
        vlat       = cs.MX.sym("vy")
        yaw_rate   = cs.MX.sym("yaw_rate")
        state = cs.vertcat(x, y, yaw, vlon, vlat, yaw_rate)

        # control variable
        Fx         = cs.MX.sym("Fx")       # longitudinal rear tire force
        delta      = cs.MX.sym("delta")    # steer tire angle rad
        input = cs.vertcat(Fx, delta)

        ## help variables
        # tire slip angle
        alpha_f = cs.if_else(vlon > 0.0000001, delta - cs.arctan((vlat + params['lf']*yaw_rate) / vlon), 0.0)
        alpha_r = cs.if_else(vlon > 0.0000001, cs.arctan((params['lr']*yaw_rate - vlat) / vlon), 0.0)
        Bf = params['Bf']
        Cf = params['Cf']
        Df = params['Df']
        Ef = params['Ef']
        Br = params['Br']
        Cr = params['Cr']
        Dr = params['Dr']
        Er = params['Er']
        Fy_f = Df * cs.sin(Cf * cs.arctan(Bf * alpha_f - Ef * (Bf * alpha_f - cs.arctan(Bf * alpha_f))))
        Fy_r = Dr * cs.sin(Cr * cs.arctan(Br * alpha_r - Er * (Br * alpha_r - cs.arctan(Br * alpha_r))))

        # rolling resistance force
        Fz_f = params['mass']*params['lr']*params['g'] / (params['wheelbase'])
        Fz_r = params['mass']*params['lf']*params['g'] / (params['wheelbase'])
        Fr_f = params['rolling_resistance'] * Fz_f
        Fr_r = params['rolling_resistance'] * Fz_r

        Fx_f = -Fr_f
        Fx_r = Fx - Fr_r    # new control variable..?

        # aerodynamics
        Faero = 0.5 * params['rho'] * params['Af'] * params['Cd'] * vlon**2 

        # state derivative
        xdot           = vlon*cs.cos(yaw) - vlat*cs.sin(yaw)
        ydot           = vlon*cs.sin(yaw) + vlat*cs.cos(yaw)
        yawdot         = yaw_rate
        # with no rolling resistance & aerodrag
        vlondot        = (Fx   - Fy_f*cs.sin(delta)) / params['mass'] + yaw_rate*vlat
        vlatdot        = (Fy_r + Fy_f*cs.cos(delta)) / params['mass'] - yaw_rate*vlon
        yawratedot     = (Fy_f*params['lf']*cs.cos(delta) - Fy_r*params['lr']) / params['inertia']
        # with rolling resistance & aerodynamics
        # vlondot        = (Fx_r - Faero - Fy_f*cs.sin(delta) + Fx_f*cs.cos(delta)) / params['mass'] + yaw_rate*vlat
        # vlatdot        = (Fy_r + Fy_f*cs.cos(delta) + Fx_f*cs.sin(delta)) / params['mass'] - yaw_rate*vlon
        # yawratedot     = (Fy_f*params['lf']*cs.cos(delta) - Fy_r*params['lr'] + Fx_f*cs.sin(delta)) / params['inertia']
        
        # dead reckoning
        dt = params['dt']
        x_next = xdot*dt + x
        y_next = ydot*dt + y
        yaw_next = yawdot*dt + yaw
        vlon_next = vlondot*dt + vlon
        vlat_next = vlatdot*dt + vlat
        yaw_rate_next = yawratedot*dt + yaw_rate
        
        state_next = cs.vertcat(x_next, y_next, yaw_next, vlon_next, vlat_next, yaw_rate_next)

        self.system = cs.Function("cartesian_dynamic_stm", [state, input], [state_next])
        
        # for memery managing
        self.buf_yaw = 0.0



    # input torque [Nm], input steer [rad]
    def NextState(self, input_force, input_steer, params):
        predict_state = cs.vertcat(self.x, self.y, self.yaw, self.vx, self.vy, self.yawrate)
    
        force = cs.DM(input_force)
        steer = cs.DM(input_steer)
        input = cs.vertcat(force, steer)
        
        output = self.system(predict_state, input)
        self.x = cs.DM(output[0])
        self.y = cs.DM(output[1])
        # self.yaw = cs.DM(output[2])
        self.vx = cs.DM(output[3])
        self.vy = cs.DM(output[4])
        self.yawrate = cs.DM(output[5])
        
        # yaw normalization
        self.buf_yaw = float(output[2])
        self.buf_yaw = ((self.buf_yaw-cs.pi) % (2*cs.pi)) + cs.pi
        if self.buf_yaw == -cs.pi:
            self.buf_yaw = cs.pi
        self.yaw        = cs.DM(self.buf_yaw)
        
        
        # set return values (x, y, yawrate)
        return float(output[0]), float(output[1]), float(output[5])

