# -----------------------------------------------------------------------------
# @file           scenario_manager.py 
# @brief          control evaluation tool by using waypoint planner and lanelet maps
#
# @authors        Seounghoon Park (sunghoon8585@gmail.com)
#
# @date           2024-12-30      Created by Seounghoon Park
# -----------------------------------------------------------------------------

# SYSTEM
import sys
import os
import numpy as np
from pathlib import Path
sys.path.append(os.path.join(os.path.dirname(__file__), "../scripts"))

# ROS
import rospy
from autohyu_msgs.msg import VehicleState

# get scenes from lanelet2 map
from trajectory_process import TrajectoryProcess

# Simulator Pose Init
from init_morai_pose import InitMoraiPose



class ScenarioManager(object):
    def __init__(self):
        # ROS Node
        rospy.init_node('scenario_manager', anonymous=True)

        # ROS params
        self.mode                   = rospy.get_param("~mode", "")
        self.ref_lat                = rospy.get_param("~ref_lat", 0.0)
        self.ref_lon                = rospy.get_param("~ref_lon", 0.0)
        self.init_pose_json_path    = rospy.get_param("~init_pose_path", "")
        self.udp_ip                 = rospy.get_param("~udp_ip", "")
        self.udp_port               = rospy.get_param("~udp_port", 0)
        self.speed_kph              = rospy.get_param("~init_speed", 0)
        self.map_directory          = rospy.get_param("~map_file_path", "")  # Lanelet2 맵이 저장된 디렉터리
        self.check_scene_change_x   = rospy.get_param("~check_scene_change_x", 0.0)
        self.check_scene_change_y   = rospy.get_param("~check_scene_change_y", 0.0)

        rospy.loginfo("map_directory: %s", self.map_directory)

        # ROS Settings
        self.rate = rospy.Rate(100)  # 1Hz

        # ROS Subscriber
        self.s_vehicle_state = rospy.Subscriber("/app/loc/vehicle_state", VehicleState, self.vehicle_state_callback)

        # ROS Inputs
        self.i_vehicle_state = VehicleState()




        ########################################################
        # Simulator Pose Init
        if (self.mode == "morai"):
            self.init_ego_pose = InitMoraiPose(self.init_pose_json_path, self.udp_ip, self.udp_port, self.speed_kph)
        elif (self.mode == "carla"):
            pass
        elif (self.mode == "carmaker"):
            pass
        else:
            rospy.logerr("Invalid mode: %s", self.mode)
            sys.exit(1)


        ########################################################
        # Variables
        self.current_scene_idx = 0
        self.current_trajectory = {}
        
        self.b_change_sig_send = True
        self.scene_end = False
        self.scene_list = {}
        self.scene_list['trajectory'] = []
        self.scene_list['file_path'] = []
        
        
        ########################################################
        # import xy trajectory from lanelet2 map
        self.trajectory_process = TrajectoryProcess(self.map_directory, self.ref_lat, self.ref_lon)
        self.scene_list['trajectory'], self.scene_list['file_path'] = \
            self.trajectory_process.get_scenes()
        self.current_trajectory = self.scene_list['trajectory'][0]

        
        
    def vehicle_state_callback(self, msg):
        self.i_vehicle_state = msg


    def find_closest_point_idx(self, trajectory, x, y):
        distances = np.linalg.norm(np.array(trajectory) - np.array([x, y]), axis=1)
        closest_idx = np.argmin(distances)
        return closest_idx, distances[closest_idx]


    # TODO : algorithm in different python file
    def check_scene_end(self, trajectory, vehicle_state):
        x, y = vehicle_state.x, vehicle_state.y
        closest_idx, distance = self.find_closest_point_idx(trajectory, x, y)
        # print(f"to end: {len(trajectory) - closest_idx}")

        if len(trajectory) - closest_idx < 30:
            return True
        else :
            return False
        
    def check_scenario_changed(self):
        # get distance of start position and current position
        dist = np.sqrt((self.i_vehicle_state.x - self.check_scene_change_x)**2 + (self.i_vehicle_state.y - self.check_scene_change_y)**2)
        print("dist: ", dist)
        if dist < 1:
            return True
        else:
            return False
        
        
    def change_scenario(self, scene_list):
        if self.current_scene_idx == len(scene_list['trajectory']):
            self.current_scene_idx = 0
        
        map_file_path = scene_list['file_path'][self.current_scene_idx]
        self.current_trajectory = scene_list['trajectory'][self.current_scene_idx]
        rospy.set_param('/waypoint_planning/map_file', map_file_path)
        print(f"Change scenario to {map_file_path}")
        
        self.init_ego_pose.run()
        
        self.current_scene_idx += 1
        self.b_change_sig_send = True


    def run(self):
        self.change_scenario(self.scene_list)
        
        while not rospy.is_shutdown():
            scene_end = self.check_scene_end(self.current_trajectory, self.i_vehicle_state)
            
            if scene_end == True:
                if self.b_change_sig_send == False:
                    self.change_scenario(self.scene_list)

            elif scene_end == False:
                self.b_change_sig_send = False
            

            self.rate.sleep()


def main():
    node = ScenarioManager()
    node.run()


if __name__ == '__main__':
    main()
