import rospy
import time

from morai_msgs.msg import EgoVehicleStatus
from morai_msgs.msg import CtrlCmd

import numpy as np


class SteerDelayCheck(object):
    def __init__(self):
        rospy.init_node('morai_steer_delay_check', anonymous=True)
        self.loop_period = 0.01
        self.rate = rospy.Rate(1/self.loop_period)
        self.params = {}
        
        
        # pub & sub       
        self.p_ctrl = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.before_time = time.time()
    
        # var
        self.front_tire_angle_deg = 0.0

        # output
        self.o_ctrl = CtrlCmd()

        self.b_angle_increase = True

    def run(self):

        while not rospy.is_shutdown():
            if (time.time() - self.before_time) > 1.0:
                self.before_time = time.time()
                
                if self.b_angle_increase == True:
                    self.front_tire_angle_deg = self.front_tire_angle_deg + 2.0
                elif self.b_angle_increase == False:
                    self.front_tire_angle_deg = self.front_tire_angle_deg - 2.0
                
                # switch mode of increase or decrease
                if self.front_tire_angle_deg > 40.0:
                    self.b_angle_increase = False
                elif self.front_tire_angle_deg < -40.0:
                    self.b_angle_increase = True
            

            
            self.o_ctrl.longlCmdType = 1
            self.o_ctrl.accel = 0.0
            self.o_ctrl.brake = 1.0
            self.o_ctrl.steering = np.deg2rad(self.front_tire_angle_deg)

            self.p_ctrl.publish(self.o_ctrl)
            self.rate.sleep()

        
if __name__ == '__main__':
    node = SteerDelayCheck()
    node.run()

    