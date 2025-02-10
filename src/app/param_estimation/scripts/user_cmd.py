#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32

import tf
import random
import numpy as np
import math
try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_UP
    from pygame.locals import K_i
    from pygame.locals import K_j
    from pygame.locals import K_k
    from pygame.locals import K_l
    from pygame.locals import K_q
    from pygame.locals import K_z
    from pygame.locals import K_w
    from pygame.locals import K_x
    from pygame.locals import K_e
    from pygame.locals import K_c
    from pygame.locals import K_k
    from pygame.locals import K_SPACE
    
    
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed. Run "$pip install pygame"')

msg = """
Control Your Vehicle!
---------------------------
Moving around:
        i       |   ARROW
   j    k    l  |

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear trq by 10%
e/c : increase/decrease only angular speed by 10%
k: brake
space key: steering 0

CTRL-C to quit
"""


class Keyboard():
    def __init__(self):
        rospy.init_node('driving_input', anonymous=True)
        self.trq_publisher = rospy.Publisher('app/con/gripped_torque', Float32, queue_size=10)
        self.accel_publisher = rospy.Publisher('app/con/gripped_accel', Float32, queue_size=10)
        self.steer_publisher = rospy.Publisher('app/con/gripped_steer', Float32, queue_size=10)
        
        self.ros_rate = 60
        
        self.pre_defined_trq = 0
        self.pre_defined_steer = 0
        

        self.max_trq = 8000
        self.min_trq = -13000
        self.max_steer = 45.0

        self.kTrq = 2.5
        self.kAngle = 0.5
        self.PI = 3.14159265358979323846
        self.use_manual = True
    def vels(self, kTrq, kAngle):
        return "trq constant: %s\t angle constant: %s\t" % (self.kTrq, self.kAngle)



    def run(self):

        rate = rospy.Rate(self.ros_rate)

        trq = 0
        frontAngle = 0

        print(msg)
        print(self.vels(self.kTrq, self.kAngle))

        while not rospy.is_shutdown():
            
            # Transfer keyboard state in pygame -> msgs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True

            keys = pygame.key.get_pressed()

            if (keys[K_UP] and keys[K_RIGHT]) or (keys[K_i] and keys[K_l]):
                trq = trq + self.kTrq
                frontAngle -= self.kAngle
                trq = sorted([0, trq, self.max_trq])[1]
                frontAngle = sorted([-self.max_steer, frontAngle, self.max_steer])[1]
                print("trq : %s %%\t frontAngle : %s degree\t" %
                      (trq, frontAngle))
            elif (keys[K_UP] and keys[K_LEFT]) or (keys[K_i] and keys[K_j]):
                trq = trq + self.kTrq
                frontAngle += self.kAngle
                trq = sorted([0, trq, self.max_trq])[1]
                frontAngle = sorted([-self.max_steer, frontAngle, self.max_steer])[1]
                print("trq : %s %%\t frontAngle : %s degree\t" %
                      (trq, frontAngle))
            elif (keys[K_DOWN] and keys[K_RIGHT]) or (keys[K_k] and keys[K_l]):
                
                trq = trq - self.kTrq*100
                frontAngle -= self.kAngle
                trq = sorted([self.min_trq, trq, 0])[1]
                frontAngle = sorted([-self.max_steer, frontAngle, self.max_steer])[1]
                print("trq : %s %%!!!!!!!!!!\t " % trq)

            elif (keys[K_DOWN] and keys[K_LEFT]) or (keys[K_k] and keys[K_j]):
                
                frontAngle += self.kAngle
                trq = trq - self.kTrq*self.kTrq*100
                trq = sorted([self.min_trq, trq, 0])[1]
                frontAngle = sorted([-self.max_steer, frontAngle, self.max_steer])[1]
                print("trq : %s %%!!!!!!!!!!\t " % trq)

            elif keys[K_UP] or keys[K_i]:
                frontAngle /= 1.05
                trq = trq + self.kTrq
                trq = sorted([0, trq, self.max_trq])[1]
                print("trq : %s %%\t frontAngle : %s degree\t" %
                      (trq, frontAngle))
            elif keys[K_LEFT] or keys[K_j]:
                frontAngle += self.kAngle
                frontAngle = sorted([-self.max_steer, frontAngle, self.max_steer])[1]
                print("trq : %s %%\t frontAngle : %s degree\t" %
                      (trq, frontAngle))
            elif keys[K_RIGHT] or keys[K_l]:
                frontAngle -= self.kAngle
                frontAngle = sorted([-self.max_steer, frontAngle, self.max_steer])[1]
                print("trq : %s %%\t frontAngle : %s degree\t" %
                      (trq, frontAngle))
            elif keys[K_DOWN] or keys[K_k]:
                frontAngle /= 1.05
                
                trq = trq - self.kTrq*100 
                trq = sorted([self.min_trq, trq, 0])[1]

                print("trq : %s %%!!!!!!!!!!\t " % trq)

            elif keys[K_q]:
                print("full throttle : %s %%!!!!!!!!!!\t " % trq)
                trq = self.max_trq
            elif keys[K_z]:
                print("full brake : %s %%!!!!!!!!!!\t " % trq)
                trq = self.min_trq

            elif keys[K_w]:
                self.pre_defined_trq =  self.pre_defined_trq + 10
                print("pre_defined_trq : %s %%!!!!!!!!!!\t " % self.pre_defined_trq)

            elif keys[K_x]:
                self.pre_defined_trq =  self.pre_defined_trq - 10
                print("pre_defined_trq : %s %%!!!!!!!!!!\t " % self.pre_defined_trq)

            elif keys[K_e]:
                self.pre_defined_steer = self.pre_defined_steer + 1
                print("pre_defined_steer : %s %%!!!!!!!!!!\t " % self.pre_defined_steer)

            elif keys[K_c]:
                self.pre_defined_steer = self.pre_defined_steer - 1
                print("pre_defined_steer : %s %%!!!!!!!!!!\t " % self.pre_defined_steer)

            elif keys[K_SPACE]:
                frontAngle = self.pre_defined_steer
                trq = self.pre_defined_trq
            else:
                frontAngle /= 1.05
                pass


            input_steering = frontAngle
            # input.brake = brake/100000.0
            self.trq_publisher.publish(trq)
            self.accel_publisher.publish(trq/600)
            self.steer_publisher.publish(input_steering)
            rate.sleep()
    
    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE)


def main():
    key = Keyboard()
    if (key.use_manual == True):
        pygame.init()
        pgscreen=pygame.display.set_mode((1, 1))
        pygame.display.set_caption('keyboard_con')
        key.run()

if __name__ == '__main__':
    main()
