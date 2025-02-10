#!/usr/bin/env python3

import rospy

from autohyu_msgs.msg import VehicleState
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from carla_msgs.msg import CarlaEgoVehicleControl

import tf
import random
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_UP
    from pygame.locals import K_w
    from pygame.locals import K_a
    from pygame.locals import K_s
    from pygame.locals import K_d
    from pygame.locals import K_q
    from pygame.locals import K_z
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_e
    from pygame.locals import K_c
    from pygame.locals import K_s
    from pygame.locals import K_LSHIFT
    
    
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed. Run "$pip install pygame"')

msg = """
Control Your Vehicle!
---------------------------
Moving around:
        i       |   ARROW
   j    k    l  |

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear accel by 10%
e/c : increase/decrease only angular speed by 10%
k: brake
space key: steering 0

CTRL-C to quit
"""


class Keyboard():
    def __init__(self):
        rospy.init_node('keyboard_carla_control', anonymous=True)
        self._role_name = 'hero'
        self.vehicle_control_publisher = rospy.Publisher('carla/'+self._role_name + '/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)
        # self.ego_bb_publisher = rospy.Publisher('/hmi/dae_ego_vehicle', Marker, queue_size=10)
        
        # rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.InitCallback)
        # self._tf_broadcaster = tf.TransformBroadcaster()

        self.ros_rate = 50
        
        self.delta_time = 1/self.ros_rate
        self.kVel = 0.75
        self.kAngle = 0.5
        self.PI = 3.14159265358979323846
        self.wheelbase = 2.7
        self.is_point_clicked = False
        self.prev_vx = 0
        self.prev_vy = 0
        self.vehicle_control = CarlaEgoVehicleControl()
        # self.vehicle_state = VehicleState()
        # self.filtered_vehicle_state = VehicleState()

    # def InitCallback(self, point):
    #     self.is_point_clicked = True
    #     self.vehicle_state.x = point.pose.pose.position.x
    #     self.vehicle_state.y = point.pose.pose.position.y
    #     self.vehicle_state.z = 0.0

    #     orientation_q = point.pose.pose.orientation
    #     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #     (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    #     self.vehicle_state.yaw = yaw

    #     print("init!")

    def run(self):

        rate = rospy.Rate(self.ros_rate)
        vel = 0
        input_accel = 0
        input_brake = 0
        frontAngle = 0


        print(msg)

        # rate_for_wait = rospy.Rate(1)
        # while not rospy.is_shutdown():
        #     if self.is_point_clicked is True:
        #         break
        #     print("wait for initpose")
        #     rate_for_wait.sleep()

        while not rospy.is_shutdown():
            
            # Transfer keyboard state in pygame -> msgs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True

            keys = pygame.key.get_pressed()

            if (keys[K_UP] and keys[K_RIGHT]) or (keys[K_w] and keys[K_d]):
                input_accel = 0.5
                # vel = sorted([-100, vel, 100])[1]
                frontAngle -= self.kAngle
                frontAngle = sorted([-30, frontAngle, 30])[1]
                
            elif (keys[K_UP] and keys[K_LEFT]) or (keys[K_w] and keys[K_a]):
                # vel += self.kVel
                # vel = sorted([-100, vel, 100])[1]
                input_accel = 0.5
                frontAngle += self.kAngle
                frontAngle = sorted([-30, frontAngle, 30])[1]
            elif (keys[K_DOWN] and keys[K_RIGHT]) or (keys[K_s] and keys[K_d]):
                # vel -= self.kVel
                # vel = sorted([-100, vel, 100])[1]
                input_brake = 1.0
                frontAngle -= self.kAngle
                frontAngle = sorted([-30, frontAngle, 30])[1]

            elif (keys[K_DOWN] and keys[K_LEFT]) or (keys[K_s] and keys[K_a]):
                # vel -= self.kVel
                # vel = sorted([-100, vel, 100])[1]
                input_brake = 1.0
                frontAngle += self.kAngle
                frontAngle = sorted([-30, frontAngle, 30])[1]

            elif keys[K_UP] or keys[K_w]:
                frontAngle /= 1.12
                input_accel = 0.5
                # vel += self.kVel
                # vel = sorted([-100, vel, 100])[1]

            elif keys[K_LEFT] or keys[K_a]:
                # vel = vel*0.99
                frontAngle += self.kAngle
                frontAngle = sorted([-30, frontAngle, 30])[1]

            elif keys[K_RIGHT] or keys[K_d]:
                # vel = vel*0.99
                frontAngle -= self.kAngle
                frontAngle = sorted([-30, frontAngle, 30])[1]

            elif keys[K_DOWN] or keys[K_s]:
                frontAngle /= 1.12
                input_brake = 1.0
                # vel -= self.kVel
                # vel = sorted([-100, vel, 100])[1]

            # elif keys[K_q]:
            #     self.vehicle_state.quality = 56

            # elif keys[K_z]:
            #     self.vehicle_state.quality = 53

            # elif keys[K_w]:
            #     self.kVel = self.kVel * 1.12

            # elif keys[K_x]:
            #     self.kVel = self.kVel * 0.99

            # elif keys[K_e]:
            #     self.kAngle = self.kAngle * 1.12

            # elif keys[K_c]:
            #     self.kAngle = self.kAngle * 0.99
            elif keys[K_LSHIFT]:
                frontAngle = 0.0
                # vel = 0.0
                input_accel = 0.0
                input_brake = 0.0
            else:
                # vel = vel*0.99
                # vel = sorted([-100, vel, 100])[1]
                input_accel = 0.0
                input_brake = 0.0
                frontAngle /= 1.12

            # print("vel : %s m/s\t frontAngle : %s degree\t" %
            #           (vel, frontAngle))
            # input_accel = Float32()
            
            # self.bicycle_model(vel, math.radians(frontAngle))
            # print("x : %s \t y : %s \t" %
            #           (self.vehicle_state.x,self.vehicle_state.y))
            self.vehicle_control.header.stamp = rospy.Time.now()
            self.vehicle_control.brake = input_brake
            self.vehicle_control.throttle = input_accel
            self.vehicle_control.hand_brake = False
            self.vehicle_control.manual_gear_shift = False
            self.vehicle_control.steer = -frontAngle / 70
            self.vehicle_control_publisher.publish( self.vehicle_control )


                        
            # marker = Marker()

            # marker.header.frame_id = "world"
            # marker.header.stamp = rospy.Time.now()

            # # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
            # marker.type = 1
            # marker.id = 0

            # # Set the scale of the marker
            # marker.scale.x = 4.635
            # marker.scale.y = 1.890
            # marker.scale.z = 1.647

            # # Set the color
            # marker.color.r = 0.0
            # marker.color.g = 1.0
            # marker.color.b = 0.0
            # marker.color.a = 1.0

            # # Set the pose of the marker
            # marker.pose.position.x = self.vehicle_state.x
            # marker.pose.position.y = self.vehicle_state.y
            # marker.pose.position.z = self.vehicle_state.z

            # q = quaternion_from_euler(0,0,self.vehicle_state.yaw)
            # marker.pose.orientation.x = q[0]
            # marker.pose.orientation.y = q[1]
            # marker.pose.orientation.z = q[2]
            # marker.pose.orientation.w = q[3]
            # self.ego_bb_publisher.publish(marker)


            # transform = Transform()
            # transform.translation.x = self.vehicle_state.x
            # transform.translation.y = self.vehicle_state.y
            # transform.translation.z = 0.0
            # transform.rotation.x = q[0]
            # transform.rotation.y = q[1]
            # transform.rotation.z = q[2]
            # transform.rotation.w = q[3]

            # update_time = rospy.Time.now()

            # self._tf_broadcaster.sendTransform(
            #     [transform.translation.x, 
            #     transform.translation.y, 
            #     transform.translation.z],
            #     [transform.rotation.x,
            #     transform.rotation.y,
            #     transform.rotation.z,
            #     transform.rotation.w],
            #     update_time,
            #     'ego_frame',
            #     'world')


            rate.sleep()

    # def bicycle_model(self, velocity, steering_angle):
    #     """
    #     -------
    #     Updates the vehicle's state using the kinematic bicycle model
    #     Parameters
    #     ----------
    #     """
    #     # Compute the angular velocity
    #     angular_velocity = velocity*math.tan(steering_angle) / self.wheelbase

    #     # Compute the final state using the discrete time model
    #     new_object_state_x   = self.vehicle_state.x + velocity*math.cos(self.vehicle_state.yaw)*self.delta_time
    #     new_object_state_y   = self.vehicle_state.y + velocity*math.sin(self.vehicle_state.yaw)*self.delta_time
    #     self.vehicle_state.yaw = self.vehicle_state.yaw + angular_velocity*self.delta_time
        
    #     vx = (new_object_state_x - self.vehicle_state.x)/self.delta_time
    #     vy = (new_object_state_y - self.vehicle_state.y)/self.delta_time

    #     ax = (vx - self.prev_vx)/self.delta_time
    #     ay = (vy - self.prev_vy)/self.delta_time
        
    #     self.prev_vx = vx
    #     self.prev_vy = vy

    #     self.vehicle_state.vx = math.sqrt(vx**2 + vy**2)
    #     self.vehicle_state.vy = 0.0
    #     self.vehicle_state.ax = math.sqrt(ax**2 + ay**2)
    #     self.vehicle_state.ay = 0.0
    #     self.vehicle_state.x = new_object_state_x
    #     self.vehicle_state.y = new_object_state_y


    #     self.filtered_vehicle_state.vx = math.sqrt(vx**2 + vy**2)
    #     self.filtered_vehicle_state.vy = 0.0
    #     self.filtered_vehicle_state.ax = math.sqrt(ax**2 + ay**2)
    #     self.filtered_vehicle_state.ay = 0.0
    #     self.filtered_vehicle_state.x = new_object_state_x
    #     self.filtered_vehicle_state.y = new_object_state_y
        
        # if(self.object_state.yaw > self.PI):
        #     self.object_state.yaw = self.object_state.yaw - 2.0*self.PI
        # if(self.object_state.yaw < -self.PI):
        #     self.object_state.yaw = self.object_state.yaw + 2.0*self.PI

        
        
    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE)


def main():
    key = Keyboard()
    pygame.init()
    pgscreen=pygame.display.set_mode((1, 1))
    pygame.display.set_caption('keyboard_obj')
    key.run()

if __name__ == '__main__':
    main()

