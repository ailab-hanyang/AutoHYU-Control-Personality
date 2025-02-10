#!/usr/bin/env python3

import rospy

from autohyu_msgs.msg import TrackObjects, TrackObject, ObjectState
from visualization_msgs.msg import MarkerArray, Marker


from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_w
    from pygame.locals import K_s
    from pygame.locals import K_RSHIFT
    
    
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed. Run "$pip install pygame"')

msg = """
Control Your Vehicle!
---------------------------
Moving around:
        i       |   ARROW
   j    k    l  |

a/d : increase/decrease yaw angle
w/s : increase/decrease linear speed

CTRL-C to quit
"""

class Keyboard():
    def __init__(self):
        rospy.init_node('keyboard_object_generator', anonymous=True)
        self.p_virtual_objects = rospy.Publisher(
            'bsw/virtual_objects', TrackObjects, queue_size=10)
        self.p_rviz_virtual_objects = rospy.Publisher(
            'hmi/perc/keyboard_objects', MarkerArray, queue_size=10)
        
        self.ros_rate = 100
        
        self.is_init = False
        
        self.x = -366.0
        self.y = 8.8
        self.yaw = 0.0
        self.speed = 0.0

        self.object_state = ObjectState()
        self.track_object = TrackObject()
        self.track_objects = TrackObjects()
        
        self.track_marker = MarkerArray()
        self.UpdateState()        

    def UpdateState(self):
        self.object_state.header.frame_id = "world"
        self.object_state.header.stamp = rospy.Time.now()
        
        self.object_state.x = self.x
        self.object_state.y = self.y
        self.object_state.z = 0.0
        
        self.object_state.roll = 0.0
        self.object_state.pitch = 0.0
        self.object_state.yaw = self.yaw
        
        self.object_state.v_x = self.speed
        self.object_state.v_y = 0.0
        self.object_state.v_z = 0.0
        
        self.object_state.a_x = 0.0
        self.object_state.a_y = 0.0
        self.object_state.a_z = 0.0
        
        self.object_state.roll_rate = 0.0
        self.object_state.pitch_rate = 0.0
        self.object_state.yaw_rate = 0.0

        self.is_init = True

    def run(self):
        rate = rospy.Rate(self.ros_rate)
        print(msg)

        rate_for_wait = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.is_init is True:
                break
            print("wait for initpose")
            rate_for_wait.sleep()

        while not rospy.is_shutdown():
            
            # Transfer keyboard state in pygame -> msgs
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return True
                elif event.type == pygame.KEYUP:
                    if self._is_quit_shortcut(event.key):
                        return True

            keys = pygame.key.get_pressed()
            
            if (keys[K_UP]) or (keys[K_i]):
                self.y -= 2.0
                
            elif (keys[K_DOWN]) or (keys[K_k]):
                self.y += 2.0
                
            elif (keys[K_LEFT]) or (keys[K_j]):
                self.x += 0.1
                
            elif (keys[K_RIGHT]) or (keys[K_l]):
                self.x -= 0.1
                
            elif (keys[K_a]):
                self.yaw += 1*math.pi/180
                
            elif (keys[K_d]):
                self.yaw -= 1*math.pi/180
                
            elif (keys[K_w]):
                self.speed += 0.1
                
            elif (keys[K_s]):
                self.speed -= 0.1
                
                if (self.speed <= 0.0) :
                    self.speed = 0.0
                
            elif keys[K_RSHIFT]:
                self.x = 0.0
                self.y = 0.0
                self.yaw = 0.0
                self.speed = 0.0
                
            self.UpdateState()

            self.track_object = TrackObject()

            self.track_object.id = 0
            self.track_object.classification = 1
            
            if (self.speed < 0.1):
                self.track_object.dynamic_state = 1
            else:
                self.track_object.dynamic_state = 2
                
            self.track_object.dimension.length = 4.635
            self.track_object.dimension.width = 1.890
            self.track_object.dimension.height = 1.647
            
            self.track_objects.header = self.object_state.header
            self.track_object.state = self.object_state            
            
            self.track_objects.object.clear()
            self.track_objects.object.append(self.track_object)

            self.p_virtual_objects.publish(self.track_objects)
                        
            position_marker = Marker()
            track_info_marker = Marker()

            position_marker.header = self.track_objects.header
            position_marker.ns = "position"
            position_marker.id = self.track_object.id
            position_marker.action = Marker.ADD
            position_marker.type = Marker.CUBE
            position_marker.lifetime = rospy.Duration(0.1)
            
            track_info_marker.header = self.track_objects.header
            track_info_marker.ns = "velocity"
            track_info_marker.id = self.track_object.id
            track_info_marker.action = Marker.ADD
            track_info_marker.type = Marker.TEXT_VIEW_FACING
            track_info_marker.lifetime = rospy.Duration(0.1)
            
            track_info_marker.text = "ID (" + str(self.track_object.id) + ")" + \
                                        " \nVel: " + str(round(self.track_object.state.v_x, 2)) + \
                                        " \nAccel: " + str(round(self.track_object.state.a_x, 2))
            
            position_marker.scale.x = self.track_object.dimension.length
            position_marker.scale.y = self.track_object.dimension.width
            position_marker.scale.z = self.track_object.dimension.height
            track_info_marker.scale.z = 1.0
            
            position_marker.color.r = 0.0
            position_marker.color.g = 1.0
            position_marker.color.b = 1.0
            position_marker.color.a = 0.5
            track_info_marker.color.r = 1.0
            track_info_marker.color.g = 1.0
            track_info_marker.color.b = 1.0
            track_info_marker.color.a = 1.0
            
            position_marker.pose.position.x = self.track_object.state.x
            position_marker.pose.position.y = self.track_object.state.y
            position_marker.pose.position.z = self.track_object.dimension.height/2.0
            
            q = quaternion_from_euler(0,0,self.track_object.state.yaw)
            position_marker.pose.orientation.x = q[0]
            position_marker.pose.orientation.y = q[1]
            position_marker.pose.orientation.z = q[2]
            position_marker.pose.orientation.w = q[3]
            
            track_info_marker.pose.position.x = self.track_object.state.x
            track_info_marker.pose.position.y = self.track_object.state.y
            track_info_marker.pose.position.z = 3.0
            track_info_marker.pose.orientation = position_marker.pose.orientation
            
            self.track_marker.markers.clear()
            self.track_marker.markers.append(position_marker)
            self.track_marker.markers.append(track_info_marker)
            
            self.p_rviz_virtual_objects.publish(self.track_marker)
            rate.sleep()     
        
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

