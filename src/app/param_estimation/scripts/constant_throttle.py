import rospy

from autohyu_msgs.msg import VehicleState
from morai_msgs.msg import EgoVehicleStatus
from morai_msgs.msg import CtrlCmd


class ConstantThrottle(object):
    def __init__(self):
        rospy.init_node('constant_throttle', anonymous=True)
        self.loop_period = 0.01

        # ROS Param
        self.params = {}
        self.params['mode'] = rospy.get_param('const_throttle_mode')
        self.params['max_vel'] = rospy.get_param('const_throttle_max_vel')
        self.params['input_throttle'] = rospy.get_param('const_throttle_input')
        self.params['input_brake'] = rospy.get_param('const_brake_input')

        # ROS Subscribers
        # self.s_vehicle_state = rospy.Subscriber('/app/loc/vehicle_state', VehicleState, self.callback_vehicle_state)
        self.s_vehicle_state = rospy.Subscriber('/app/loc/vehicle_state_filtered', VehicleState, self.callback_vehicle_state)
        
        # input
        self.i_vehicle_state = VehicleState()
        
        # ROS Publishers & Output
        if self.params['mode'] == 'morai':
            self.p_cmd = \
                rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
            self.o_cmd = CtrlCmd()
      
        self.b_is_accel = True


    def callback_vehicle_state(self, data):
        self.i_vehicle_state = data

        
    def run(self):
        rate = rospy.Rate(1/self.loop_period)
        # 1 : throttle, 2 : velocity, 3 : accel
        self.o_cmd.longlCmdType = 1 
        self.o_cmd.brake = 0.0
        self.o_cmd.steering = 0.0
        # self.o_cmd.speed = 0.0          # actived when longlCmdType == 2
        # self.o_cmd.acceleration = 0.0   # actived when longlCmdType == 3
        # 좀 짜치네 accel & acceleration?
        
        while not rospy.is_shutdown():
            # keep accel during reaching high speed
            if self.b_is_accel == True:
                self.o_cmd.accel = 1.0
                
                if self.i_vehicle_state.vx < self.params['max_vel']/3.6 :
                    self.b_is_accel = True
                else :
                    self.b_is_accel = False
                    
                self.p_cmd.publish(self.o_cmd)
                    
            else:
                self.o_cmd.accel = self.params['input_throttle']
                self.o_cmd.brake = self.params['input_brake']
                self.p_cmd.publish(self.o_cmd)
                
            rate.sleep()
    
            
            
def main():
    node = ConstantThrottle()
    node.run()
        
if __name__ == '__main__':
    main()
    