#!/usr/bin/env python3

import rospy

from autohyu_msgs.msg import Trajectory
from autohyu_msgs.msg import TrajectoryPoint
from autohyu_msgs.msg import VehicleState

import tf
import os
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
try:
    import pygame
    from pygame.locals import K_SPACE
    from pygame.locals import K_w
    from pygame.locals import K_a
    from pygame.locals import K_s
    from pygame.locals import K_d
    from pygame.locals import K_q

except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed. Run "$pip install pygame"')

class Keyboard():
    def __init__(self):
        rospy.init_node('custom_speed_profile', anonymous=True)
        self.trajectory_publisher = rospy.Publisher('app/pla/trajectory', Trajectory, queue_size=1)
        rospy.Subscriber("app/loc/vehicle_state", VehicleState, self.vehicle_state_callback)
        self.vehicle_state = VehicleState()

        current_directory = os.path.dirname(os.path.realpath(__file__))
        absolute_path = os.path.join(current_directory, 'speed_profile.csv')
        speed_profile = np.loadtxt(absolute_path, delimiter=',')

        self.profile_time = speed_profile[:, 0]
        self.profile_dt = (self.profile_time[-1] - self.profile_time[0])/len(self.profile_time)
        self.profile_speed = speed_profile[:, 1]

        self.time_horizon = 5.0
        self.start_speed_profile = False
        
        self.init_ind = 2
        self.ind = self.init_ind
        
        self.curr_sec = 0.0

    def vehicle_state_callback(self, vehicle_state):
        self.vehicle_state.x = vehicle_state.x
        self.vehicle_state.y = vehicle_state.y
        self.vehicle_state.z = 0.0
        self.vehicle_state.yaw = vehicle_state.yaw

    def transform_to_global_coordinates(self,dist):
        # 회전 행렬 생성
        R = np.array([[np.cos(self.vehicle_state.yaw), -np.sin(self.vehicle_state.yaw)],
                    [np.sin(self.vehicle_state.yaw),    np.cos(self.vehicle_state.yaw)]])

        # 차량 좌표계에서 앞쪽 1미터 지점 벡터
        front_point_vehicle = np.array([[dist], [0]])

        # 회전 변환
        global_front_point = np.dot(R, front_point_vehicle)

        # 글로벌 좌표로 변환된 앞쪽 dist미터 지점 좌표
        global_x = self.vehicle_state.x + global_front_point[0, 0]
        global_y = self.vehicle_state.y + global_front_point[1, 0]

        return global_x, global_y, self.vehicle_state.yaw

    def run(self):

        rate = rospy.Rate(1.0/self.profile_dt)
        while not rospy.is_shutdown():
            trajectory = Trajectory()

            # Transfer keyboard state in pygame -> msgs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True
            keys = pygame.key.get_pressed()

            if keys[K_SPACE]:
                self.start_speed_profile = True
            if self.start_speed_profile:
                if self.ind == self.init_ind:
                    self.ind += 1
                    self.curr_sec = rospy.get_time()
                curr_sec = rospy.get_time()-self.curr_sec

                if curr_sec > self.profile_time[self.ind]:
                    self.ind += 1

                print(curr_sec)

                for i in range(-self.init_ind, int(self.time_horizon/self.profile_dt) + 1):
                    [x,y,yaw] = self.transform_to_global_coordinates(int(i*self.profile_dt))
                    tp = TrajectoryPoint()
                    tp.x = x
                    tp.y = y
                    tp.yaw = yaw
                    tp.speed = self.profile_speed[self.ind+i]
                    trajectory.point.append(tp)

            self.trajectory_publisher.publish(trajectory)
            rate.sleep()


    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_q)

def main():
    key = Keyboard()
    pygame.init()
    pgscreen=pygame.display.set_mode((1, 1))
    pygame.display.set_caption('keyboard_obj')
    key.run()

if __name__ == '__main__':
    main()
