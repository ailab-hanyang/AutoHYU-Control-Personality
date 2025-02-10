# system headers
import os
import numpy as np

# algorithm header
import casadi as cs


class VehicleModel(object):
    def __init__(self, vehicle_state, params):
        self.model_name = "Cartesian Kinematic Single Track Model (Rear Frame)"
        
        ## Variables
        self.x          = cs.DM(vehicle_state.x)
        self.y          = cs.DM(vehicle_state.y)
        self.yaw        = cs.DM(vehicle_state.yaw)
        self.v         = cs.DM(vehicle_state.vx)
        
        ## Define Model
        # states
        x          = cs.MX.sym("x")
        y          = cs.MX.sym("y")
        yaw        = cs.MX.sym("yaw")
        v       = cs.MX.sym("v")
        state = cs.vertcat(x, y, yaw, v)

        # control variable
        a         = cs.MX.sym("a")        # longi acceleration
        delta      = cs.MX.sym("delta")    # steer tire angle rad
        input = cs.vertcat(a, delta)

        ## help variables
        # tire slip angle

        # aerodynamics
        Faero = 0.5 * params['rho'] * params['Af'] * params['Cd'] * v**2 

        # state derivative
        xdot           = v*cs.cos(yaw)
        ydot           = v*cs.sin(yaw)
        yawdot         = (v*cs.tan(delta)) / params['wheelbase']
        # with no rolling resistance & aerodrag
        vdot           = a
        
        
        # dead reckoning
        dt = params['dt']
        x_next = xdot*dt + x
        y_next = ydot*dt + y
        yaw_next = yawdot*dt + yaw
        v_next = vdot*dt + v
        
        state_next = cs.vertcat(x_next, y_next, yaw_next, v_next)

        self.system = cs.Function("cartesian_kinematic_stm_rear", [state, input], [state_next])
        
        # for memery managing
        self.buf_yaw = 0.0



    # input force[N], input steer [rad]
    def NextState(self, input_force, input_steer, params):
        predict_state = cs.vertcat(self.x, self.y, self.yaw, self.v)

        accel = cs.DM(input_force/params['mass'])
        steer = cs.DM(input_steer)
        input = cs.vertcat(accel, steer)
        
        output = self.system(predict_state, input)
       
        self.x = cs.DM(output[0])
        self.y = cs.DM(output[1])
        # self.yaw = cs.DM(output[2])
        self.v = cs.DM(output[3])
        
        # yaw normalization
        self.buf_yaw = float(output[2])
        self.buf_yaw = ((self.buf_yaw-cs.pi) % (2*cs.pi)) + cs.pi
        if self.buf_yaw == -cs.pi:
            self.buf_yaw = cs.pi
        self.yaw        = cs.DM(self.buf_yaw)
        
        
        # set return values (x, y, yawrate)
        return float(output[0]), float(output[1]), float(output[3]*cs.tan(steer)/params['wheelbase'])

