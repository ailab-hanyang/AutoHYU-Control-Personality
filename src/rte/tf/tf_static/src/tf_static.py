#!/usr/bin/env python3
#
# Copyright (c) 
#
"""
tf_static node
"""

import rospy
import tf2_ros
import os
from tf.transformations import quaternion_from_euler
from configparser import ConfigParser

from geometry_msgs.msg import TransformStamped, Transform

import math

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

class TransformFrameStatic(object):
    """
    Handles tf frames in autohyu
    """

    def __init__(self):
        # ROS node
        rospy.init_node('tf_static', anonymous=True)

        self.ini_file = os.path.join(os.environ.get('PWD', '.'), "config", "calibration.ini")
        # Parameters
        self._period = rospy.get_param('/task_period/period_tf_static', 1.0)  # Default to 1.0 if not set
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.frame_list = []

        # Variables to manage vehicle_origin
        self.current_vehicle_origin = None

        # Mapping of vehicle_origin to frame sections
        self.vehicle_origin_frames = {
            0: ['Rear To Main LiDAR', 'Rear To Imu', 'Rear To Gps', 'Rear To Ego Mesh'],
            1: ['CG To Main LiDAR', 'CG To Imu', 'CG To Gps', 'CG To Ego Mesh']
        }

        # Initial setup
        self.load_initial_frames()

    def destroy(self):
        """
        Destroy all objects
        """
        pass  # No dynamic resources to clean up

    def add_frame(self, type, child, parent, xyz, q, time):
        """
        Add frame into TransformFrameStatic.frame_list
        @child  : child frame
        @parent : parent frame
        @xyz    : translations in x, y, and z direction [m]
        @q      : rotation as quaternion [x, y, z, w]
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
        frame = {
            "type": type,
            "child": child, 
            "parent": parent, 
            "transform": transform,
            "time": time
        }
        self.frame_list.append(frame)

    def remove_frames(self, child_frame_ids):
        """
        Remove frames from frame_list based on a list of child_frame_ids
        """
        self.frame_list = [frame for frame in self.frame_list if frame["child"] not in child_frame_ids]

    def update_frame_time(self):
        """
        Update the timestamp of all frames
        """
        update_time = rospy.Time.now()
        for frame in self.frame_list:
            frame["time"] = update_time

    def add_tf_from_config(self, ini_file, section, update_time):
        """
        Add a transform frame based on the configuration file
        """
        config = ConfigParser()
        config.read(ini_file)

        if not config.has_section(section):
            rospy.logwarn(f"Section '{section}' not found in {ini_file}. Skipping.")
            return

        try:
            parent_frame_id = config.get(section, 'parent_frame_id')
            child_frame_id = config.get(section, 'child_frame_id')
            transform_xyz_m = list(map(float, config.get(section, 'transform_xyz_m').split()))
            rotation_rpy_deg = list(map(float, config.get(section, 'rotation_rpy_deg').split()))
        except (KeyError, ValueError) as e:
            rospy.logerr(f"Error reading section '{section}': {e}")
            return

        # Convert degrees to radians
        rotation_rpy_rad = [math.radians(angle) for angle in rotation_rpy_deg]
        rotation_quat = quaternion_from_euler(*rotation_rpy_rad)

        self.add_frame(
            'static',
            child_frame_id,
            parent_frame_id,
            transform_xyz_m, 
            rotation_quat,
            update_time
        )

    def load_initial_frames(self):
        """
        Load initial frames based on the current vehicle_origin
        """
        vehicle_origin = self.read_vehicle_origin()

        self.current_vehicle_origin = vehicle_origin

        update_time = rospy.Time.now()

        # Add frames based on vehicle_origin
        self.add_frames_for_vehicle_origin(vehicle_origin, update_time)

        # Add other static frames
        # self.add_tf_from_config(self.ini_file, 'Main LiDAR To Second LiDAR', update_time)
        self.add_tf_from_config(self.ini_file, 'Main LiDAR To Radar', update_time)
        # self.add_tf_from_config(self.ini_file, 'Main LiDAR To Third LiDAR', update_time)

    def read_vehicle_origin(self):
        """
        Read vehicle_origin from ini_file
        """
        config = ConfigParser()
        config.read(self.ini_file)

        # Read vehicle_origin
        try:
            vehicle_origin = config.getint('Vehicle Origin', 'vehicle_origin')
            if vehicle_origin not in [0, 1]:
                rospy.logwarn(f"Invalid vehicle_origin value: {vehicle_origin}. Defaulting to 0.")
                vehicle_origin = 0
        except (KeyError, ValueError) as e:
            rospy.logerr(f"Error reading vehicle_origin: {e}. Defaulting to 0.")
            vehicle_origin = 0

        return vehicle_origin

    def add_frames_for_vehicle_origin(self, vehicle_origin, update_time):
        """
        Add frames associated with the given vehicle_origin
        """
        sections = self.vehicle_origin_frames.get(vehicle_origin, [])
        for section in sections:
            self.add_tf_from_config(self.ini_file, section, update_time)

    def remove_frames_for_vehicle_origin(self, vehicle_origin):
        """
        Remove frames associated with the given vehicle_origin
        """
        # Get child_frame_ids for the frames to remove
        config = ConfigParser()
        config.read(self.ini_file)
        child_frame_ids = []
        sections = self.vehicle_origin_frames.get(vehicle_origin, [])
        for section in sections:
            if config.has_section(section):
                try:
                    child_frame_id = config.get(section, 'child_frame_id')
                    child_frame_ids.append(child_frame_id)
                except (KeyError, ValueError) as e:
                    rospy.logerr(f"Error reading child_frame_id in section '{section}': {e}")
        # Remove frames
        self.remove_frames(child_frame_ids)

    def run(self):
        """
        Main loop
        """
        rate = rospy.Rate(1.0 / self._period)

        while not rospy.is_shutdown():
            # Check if vehicle_origin has changed
            vehicle_origin = self.read_vehicle_origin()
            if vehicle_origin != self.current_vehicle_origin:
                rospy.loginfo(f"vehicle_origin changed from {self.current_vehicle_origin} to {vehicle_origin}")
                # Remove existing frames for the old vehicle_origin
                self.remove_frames_for_vehicle_origin(self.current_vehicle_origin)

                # Update current_vehicle_origin
                self.current_vehicle_origin = vehicle_origin

                # Add frames for the new vehicle_origin
                update_time = rospy.Time.now()
                self.add_frames_for_vehicle_origin(vehicle_origin, update_time)

            # Update frame timestamps
            self.update_frame_time()

            # Prepare and send static transforms
            static_transforms = []
            for frame in self.frame_list:
                if frame["type"] == "static":
                    static_transform = TransformStamped()
                    static_transform.header.stamp = frame["time"]
                    static_transform.header.frame_id = frame["parent"]
                    static_transform.child_frame_id = frame["child"]
                    static_transform.transform = frame["transform"]
                    static_transforms.append(static_transform)

            if static_transforms:
                self._static_broadcaster.sendTransform(static_transforms)

            rate.sleep()

# ==============================================================================

def main():
    """
    Main function
    """
    tf_static = TransformFrameStatic()
    try:
        tf_static.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        tf_static.destroy()

if __name__ == '__main__':
    main()
