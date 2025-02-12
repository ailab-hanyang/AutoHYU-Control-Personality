#!/usr/bin/env python3
#
# Copyright (c) 2023 AI Lab.
# 24.06.24 Junhee Lee : Fix warning "TF_REPEATED_DATA ignoring data with redundant timestamp"
"""
tf_dynamic node

"""

import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from visualization_msgs.msg import Marker

from autohyu_msgs.msg import VehicleState
import math

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

class TransformFrameDynamic(object):
    """
    Handles tf frames in autohyu

    """

    def __init__(self):
        # ROS node
        rospy.init_node('tf_dynamic', anonymous=True)
        
        # Parameters
        self._period = rospy.get_param('/task_period/period_tf_dynamic')
        self._tf_stamp_from_vehicle_state = rospy.get_param('/tf_dynamic/tf_stamp_from_vehicle_state')
        self._tf_broadcaster = tf.TransformBroadcaster()
        self.frame_list = []
        self.vehicle_state = VehicleState()
        self.dae = self.visualize_dae('package://tf_dynamic/dae/Ioniq5.stl', 'ego_mesh','IONIQ 5', (0.0,0.0,0.0), (0.0,0.0,180.0))
        
        # Publisher
        self.dae_publisher = rospy.Publisher(
            "hmi/dae_ego_vehicle", Marker, queue_size=1)
        
        # Subscriber
        self.vehicle_state_subscriber = rospy.Subscriber(
            "app/loc/vehicle_state", VehicleState, self.on_vehicle_state)        

    def destroy(self):
        """
        Destroy all objects
        """
        self.vehicle_state_subscriber.unregister()

    def on_vehicle_state(self, data):
        """
        Callback on vehicle state event from
        vehicle_state_driver
        """
        self.vehicle_state = data

    def visualize_dae(self, object_name, frame_id, name_space, xyz, rpy):
        """
        Visualize dae object
        """
        marker = Marker()
        
        marker.header.frame_id = frame_id
        marker.header.stamp = self.vehicle_state.header.stamp
        marker.ns = name_space
        marker.id = 0
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = object_name
        marker.mesh_use_embedded_materials = True
        marker.action = marker.ADD
        marker.pose.position.x = xyz[0]
        marker.pose.position.y = xyz[1]
        marker.pose.position.z = xyz[2]
        orientation = quaternion_from_euler(rpy[0]/RAD2DEG, rpy[1]/RAD2DEG, rpy[2]/RAD2DEG)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 0.5
        marker.color.r = 0.32156862745
        marker.color.g = 0.43137254902
        marker.color.b = 0.21568627451
        
        return marker
    
    def calculate_transform_from_ego_to_world_3dof(self):
        """
        Calculate position difference between 
        /ego_frame and /world in Cartesian coordinate
        """
        transform = Transform()
        transform.translation.x = self.vehicle_state.x
        transform.translation.y = self.vehicle_state.y
        transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, (self.vehicle_state.yaw))
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        
        return transform
    
    def calculate_transform_from_ego_to_world_6dof(self):
        """
        Calculate position difference between 
        /ego_frame and /world in Cartesian coordinate
        """
        transform = Transform()
        transform.translation.x = self.vehicle_state.x
        transform.translation.y = self.vehicle_state.y
        transform.translation.z = self.vehicle_state.z
        q = quaternion_from_euler(self.vehicle_state.roll, self.vehicle_state.pitch, (self.vehicle_state.yaw))
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        
        return transform

    def add_frame(self, type, child, parent, xyz, q, time):
        """
        Add frame into TransformFrameDynamic.tf_list
        @child  : child frame
        @parent : parent frame
        @xyz    : translations in x, y, and z direction [m]
        @rpy    : rotations in x, y, and z direction [rad]
        @time   : transform time
        """
        transform = Transform()
        transform.translation.x = xyz[0]
        transform.translation.y = xyz[1]
        transform.translation.z = xyz[2]
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        frame = {"type": type,
                 "child": child, 
                 "parent": parent, 
                 "transform": transform,
                 "time": time}
        self.frame_list.append(frame)

    def update_frame(self):
        """
        Update frame function
        """
        update_time = 0
        if(self._tf_stamp_from_vehicle_state == True):
            update_time = self.vehicle_state.header.stamp
        else:
            update_time = rospy.Time.now()
        if self.frame_list is not None:
            for frame in self.frame_list:
                if frame["type"] != 'static':
                    if frame["child"] == 'ego_frame':
                        frame["transform"] = self.calculate_transform_from_ego_to_world_3dof()
                    frame["time"] = update_time
                    if frame["child"] == 'ego_frame_6dof':
                        frame["transform"] = self.calculate_transform_from_ego_to_world_6dof()
                    frame["time"] = update_time

    def run(self):
        """
        main loop
        """
        rate = rospy.Rate(1 / self._period)

        update_time = self.vehicle_state.header.stamp

        transform = self.calculate_transform_from_ego_to_world_3dof()
        self.add_frame('dynamic', 'ego_frame', 'world',
            [transform.translation.x, transform.translation.y, transform.translation.z], 
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w], update_time)
        
        transform_6dof = self.calculate_transform_from_ego_to_world_3dof()
        self.add_frame('dynamic', 'ego_frame_6dof', 'world',
            [transform_6dof.translation.x, transform_6dof.translation.y, transform_6dof.translation.z], 
            [transform_6dof.rotation.x, transform_6dof.rotation.y, transform_6dof.rotation.z, transform_6dof.rotation.w], update_time)
                     
        while not rospy.is_shutdown():
            self.update_frame()
            for frame in self.frame_list:
                if frame["type"] != "static":
                    self._tf_broadcaster.sendTransform(
                        [frame["transform"].translation.x, 
                        frame["transform"].translation.y, 
                        frame["transform"].translation.z],
                        [frame["transform"].rotation.x,
                        frame["transform"].rotation.y,
                        frame["transform"].rotation.z,
                        frame["transform"].rotation.w],
                        frame["time"],
                        frame["child"],
                        frame["parent"])
            self.dae.header.stamp = self.vehicle_state.header.stamp
            self.dae_publisher.publish(self.dae)
            rate.sleep()
        
# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    tf_dynamic = TransformFrameDynamic()
    try:
        tf_dynamic.run()
    finally:
        if tf_dynamic is not None:
            tf_dynamic.destroy()


if __name__ == '__main__':
    main()