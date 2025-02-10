
#!/usr/bin/env python3

"""
opti_sn_planning node
Junhee Lee 2024 Dec
"""
# ROS Library
import rospy
from std_msgs.msg import Float32, Float32MultiArray
from autohyu_msgs.msg import VehicleState, BehaviorTrajectory, Trajectory, TrajectoryPoint, PredictObjects, ControlInfo
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

# Python Library
import numpy as np
import yaml
import time
from casadi import *
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from utils.trajectory_process import *

# MPC Models
from MPC.NMPC_class_kinematic import Nonlinear_Model_Predictive_Controller as Model_Predictive_Controller

class OptiSNPlanning(object):
    """
    Control With Acados
    """

    def __init__(self):
        # init ROS node
        rospy.init_node('opti_sn_planning', anonymous=True)
        self.period = rospy.get_param('/task_period/period_trajectory_planning')

        # Subscriber
        self.s_vehicle_state = rospy.Subscriber(
            "app/loc/vehicle_state", VehicleState, self.callback_vehicle_state)
        self.is_vehicle_state = False

        self.s_trajectory = rospy.Subscriber(
            "app/pla/behavior_trajectory", BehaviorTrajectory, self.callback_behavior_trajectory)
        self.is_trajectory = False

        self.s_predicted_objects = rospy.Subscriber(
            "app/perc/predict_objects", PredictObjects, self.callback_objects)
        self.is_obstacles = False

        # Publisher
        self.p_command_steer = rospy.Publisher(
            "app/con/command_steer", Float32, queue_size=10)
        
        self.p_predicted_trajectory = rospy.Publisher(
            "app/con/predicted_trajectory", Trajectory, queue_size=10)
        
        self.p_control_path = rospy.Publisher(
            "hmi/con/control_path", MarkerArray, queue_size=10)
        self.p_resample_path = rospy.Publisher(
            "hmi/con/resample_path", MarkerArray, queue_size=10)
        self.p_control_info = rospy.Publisher(
            "hmi/con/control_info", ControlInfo, queue_size=10)


        # Add new publishers for sampled_trajectory and target_pred
        self.p_sampled_trajectory = rospy.Publisher(
            "hmi/con/sampled_trajectory", Float32MultiArray, queue_size=10)
        self.p_target_pred = rospy.Publisher(
            "hmi/con/target_pred", Float32MultiArray, queue_size=10)

        # Input Variables
        self.i_vehicle_state = VehicleState()
        self.i_bhv_trajectory = BehaviorTrajectory()

        # Output Variables
        self.o_command_steer_deg = Float32()
        self.o_predicted_trajectory = Trajectory()
        self.o_control_path = MarkerArray()
        self.o_resample_path = MarkerArray()
        self.o_control_info = ControlInfo()
                
                
        # Predictions : Every controller's output must satisfy ... [x,y,yaw,v,delta,a]
        self.N_MPC = 40
        self.Ts_MPC = 0.1

        self.target_pred = np.zeros((self.N_MPC, 6)) 
        self.sampled_trajectory = np.zeros((self.N_MPC+1, 7)) 
        
        # Initialize markers
        self.initialize_markers()

        self.MPC = Model_Predictive_Controller()


    def loop(self):
        """
        main loop
        """   
        rate = rospy.Rate(float(1. / self.period))

        while not rospy.is_shutdown():
            total_time_start = time.time()

            if not self.is_vehicle_state:
                print(f"{time.time():.1f}", " waiting for vehicle state")
                continue
            if not self.is_obstacles:
                print(f"{time.time():.1f}", " waiting for objects")
                continue
            if not self.is_trajectory or len(self.i_bhv_trajectory.point) < 1 :
                print(f"{time.time():.1f}", " waiting for trajectory")
                continue
            
            start = time.perf_counter()
            vehicle_state, measured_state = npize_vehicle_state(self.i_vehicle_state)
            
            point, reference_map, right_boundary, left_boundary = npize_trajectory(self.i_bhv_trajectory)
            objects         = npize_objects(self.i_objects,max_objects_num=4)
            
            road_map = generate_road_map(reference_map)
            
            vehicle_state  = frenet_transform(road_map,vehicle_state)
            vehicle_state  = frenet_mu_transform(road_map,vehicle_state)

            point = frenet_transform(road_map,point)
            point = frenet_mu_transform(road_map, point, start_mu=vehicle_state[0,6])

            right_boundary  = frenet_transform(road_map,right_boundary)
            right_boundary  = frenet_mu_transform(road_map, right_boundary, start_mu=vehicle_state[0,6])

            left_boundary  = frenet_transform(road_map,left_boundary)
            left_boundary  = frenet_mu_transform(road_map, left_boundary, start_mu=vehicle_state[0,6])

            sn_objects = np.zeros((np.shape(objects)[0], np.shape(objects)[1], 11))  
            for i in range(np.shape(objects)[0]):
                if objects[i,:,4].all() == True:  
                    object = objects[i, :, :]
                    object = frenet_transform(road_map,object)
                    object = frenet_mu_transform(road_map,object)
                    if np.isnan(object).any():
                        object[np.isnan(object)] = 0  # NaN인 위치에 0 할당
                    sn_objects[i, :, :] = object
                else:
                    sn_objects[i, :, :] = np.zeros((np.shape(objects)[1], 11))
            objects = filtering_objects_in_lane(sn_objects,left_boundary,right_boundary)

            end = time.perf_counter()

            target_cart, self.target_pred, target_input, states = self.MPC.calculate_optimal(road_map,
                                       vehicle_state,
                                       objects,
                                       point,
                                       right_boundary,
                                       left_boundary,
                                       measured_state)

            roadmap_time = end - start

            print(f"RoadMap Time: {roadmap_time*1000:.6f} ms")
            self.o_command_steer_deg.data = np.float32(target_input[0])

            self.update_resample_path(point)
            self.update_control_path(target_cart)
            total_time_end = time.time()           
            total_time = 1000.0 * (total_time_end - total_time_start)
            self.update_control_info(vehicle_state, point, total_time, states)
            # # Publish 
            self.publish()

            rate.sleep()


    def destroy(self):
        """
        Destroy all objects
        """

        # 모든 객체 해제
        self.p_command_steer.unregister()
        self.p_control_path.unregister()
        self.p_predicted_trajectory.unregister()
        self.p_resample_path.unregister()
        self.p_control_info.unregister()
        self.p_sampled_trajectory.unregister()
        self.p_target_pred.unregister()
        self.s_vehicle_state.unregister()
        self.s_trajectory.unregister()
        
    
    def callback_vehicle_state(self, data):
        """
        Callback function for /vehicle_state
        """
        self.i_vehicle_state = data
        self.is_vehicle_state = True

    def callback_behavior_trajectory(self, data):
        """
        Callback function for /trajectory
        """
        self.i_bhv_trajectory = data
        self.is_trajectory = True
    
    def callback_objects(self, data):
        self.i_objects = data
        self.is_obstacles = True


    def publish(self):
        """
        Publish Control Command and Errors 
        """
        # self.p_command_steer.publish(self.o_command_steer_deg)
        self.p_control_path.publish(self.o_control_path)
        self.p_predicted_trajectory.publish(self.o_predicted_trajectory)
        self.p_resample_path.publish(self.o_resample_path)
        self.p_control_info.publish(self.o_control_info)

        # Publish sampled_trajectory and target_pred
        self.publish_float32_multi_array(self.p_sampled_trajectory, self.sampled_trajectory)
        self.publish_float32_multi_array(self.p_target_pred, self.target_pred)

    def publish_float32_multi_array(self, publisher, array):
        """
        Publish numpy array as Float32MultiArray
        """
        array = np.float32(array)
        msg = Float32MultiArray()
        msg.data = array.flatten().tolist()
        publisher.publish(msg)
    
    def initialize_markers(self):
        ## Messages
        # optimized control path (w/ speed)
        self.speed_marker = Marker()
        self.speed_marker.header.frame_id  = "world"
        self.speed_marker.ns = "temporal"
        self.speed_marker.id = 0
        self.speed_marker.action = Marker().ADD
        self.speed_marker.type = Marker().LINE_LIST
        self.speed_marker.lifetime = rospy.Duration(0.2)
        self.speed_marker.scale.x = 0.15
        self.speed_marker.color.r = 1.0
        self.speed_marker.color.g = 0.0
        self.speed_marker.color.b = 0.58
        self.speed_marker.color.a = 1.0

        for i in range(self.N_MPC): 
            pt1 = Point()
            pt2 = Point()
            self.speed_marker.points.append(pt1)
            self.speed_marker.points.append(pt2)

        self.path_marker = Marker()
        self.path_marker.header.frame_id = "world"
        self.path_marker.ns = "spartial"
        self.path_marker.id = 1
        self.path_marker.action = Marker().ADD
        self.path_marker.type = Marker().LINE_STRIP
        self.path_marker.lifetime = rospy.Duration(0.2)
        self.path_marker.scale.x = 0.25
        self.path_marker.color.r = 1.0
        self.path_marker.color.g = 0.0
        self.path_marker.color.b = 0.58
        self.path_marker.color.a = 1.0

        for i in range(self.N_MPC): 
            pt = Point()
            self.path_marker.points.append(pt)

        # reference trajectory control uses (w/o speed)
        self.resample_marker = MarkerArray()
        for i in range((self.N_MPC+1)*2):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.action   = Marker().ADD
            marker.type     = Marker().CYLINDER
            marker.lifetime = rospy.Duration(0.2)
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.color.a = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            self.resample_marker.markers.append(marker)



    def update_resample_path(self, sampled_trajectory):
        sampled_x       = sampled_trajectory[:, 0]
        sampled_y       = sampled_trajectory[:, 1]
        sampled_yaw     = sampled_trajectory[:, 2]
        sampled_speed   = sampled_trajectory[:, 3]

        for j in range(len(sampled_x)):
            visual_yaw = sampled_yaw[j] * 7

            # self.resample_marker.markers[j].header.stamp = rospy.Time.now()
            self.resample_marker.markers[j].id = j
            self.resample_marker.markers[j].ns = 'center/yaw'
            
            self.resample_marker.markers[j].pose.position.x = sampled_x[j]
            self.resample_marker.markers[j].pose.position.y = sampled_y[j]
            self.resample_marker.markers[j].pose.position.z = visual_yaw * 0.5
            
            self.resample_marker.markers[j].scale.z = visual_yaw

            self.resample_marker.markers[j].color.r = 0.0
            self.resample_marker.markers[j].color.g = 0.0
            self.resample_marker.markers[j].color.b = 1.0
            self.resample_marker.markers[j].color.a = 0.5
 

        for i in range(len(sampled_x)):
            j = j + 1

            # self.resample_marker.markers[j].header.stamp = rospy.Time.now()
            self.resample_marker.markers[j].id = j
            self.resample_marker.markers[j].ns = 'center/speed'
            
            self.resample_marker.markers[j].pose.position.x = sampled_x[i]
            self.resample_marker.markers[j].pose.position.y = sampled_y[i]
            self.resample_marker.markers[j].pose.position.z = sampled_speed[i] * 0.27778 * 0.5
            
            self.resample_marker.markers[j].scale.z = sampled_speed[i] * 0.27778

            self.resample_marker.markers[j].color.r = 0.0
            self.resample_marker.markers[j].color.g = 1.0
            self.resample_marker.markers[j].color.b = 1.0
            self.resample_marker.markers[j].color.a = 0.5


        self.o_resample_path = self.resample_marker

    def update_control_path(self, pred_X):
        predicted_trajectory = Trajectory()
        
        N = pred_X.shape[0]  # pred_X의 첫 번째 차원 크기
             
        for i in range(N):
            point = TrajectoryPoint()
            point.x = pred_X[i, 0]
            point.y = pred_X[i, 1]
            point.yaw = pred_X[i, 2]
            point.speed = pred_X[i, 3] 
            predicted_trajectory.point.append(point)

        marker_array = MarkerArray()
        
        self.speed_marker.header.frame_id = "world"
        self.speed_marker.header.stamp = predicted_trajectory.header.stamp

        idx = 0
        for i in range(0, len(predicted_trajectory.point) * 2, 2):
            self.speed_marker.points[i].x = predicted_trajectory.point[idx].x
            self.speed_marker.points[i].y = predicted_trajectory.point[idx].y
            self.speed_marker.points[i].z = 0

            self.speed_marker.points[i + 1].x = predicted_trajectory.point[idx].x
            self.speed_marker.points[i + 1].y = predicted_trajectory.point[idx].y
            self.speed_marker.points[i + 1].z = predicted_trajectory.point[idx].speed * 0.27778 # kph2mps
            idx += 1
        marker_array.markers.append(self.speed_marker)

        self.path_marker.header.frame_id = "world"
        self.path_marker.header.stamp = predicted_trajectory.header.stamp

        for i in range(len(predicted_trajectory.point)):
            self.path_marker.points[i].x = predicted_trajectory.point[i].x
            self.path_marker.points[i].y = predicted_trajectory.point[i].y
            self.path_marker.points[i].z = 0.1
        marker_array.markers.append(self.path_marker)

        self.o_control_path = marker_array
        self.o_predicted_trajectory = predicted_trajectory



    def update_control_info(self, vehicle_state, sampled_trajectory, total_time, MPC_stats):
        self.o_control_info.current_speed = vehicle_state[0,3] * 3.6
        self.o_control_info.target_speed = sampled_trajectory[0, 4] * 3.6
        self.o_control_info.speed_error = (sampled_trajectory[0, 4] - vehicle_state[0,3] ) * 3.6
        self.o_control_info.yaw_error = sampled_trajectory[0, 3]
        self.o_control_info.cross_track_error = vehicle_state[0,5]
        self.o_control_info.total_time = total_time
        self.o_control_info.opt_cost = MPC_stats[0]
        self.o_control_info.opt_time = MPC_stats[1]
        self.o_control_info.sqp_iter = MPC_stats[2]
        self.o_control_info.qp_iter = MPC_stats[3]
        self.o_control_info.solve_time = MPC_stats[5]

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main():
    """
    main function
    """
    opti_sn_planning = OptiSNPlanning()
    try:
        opti_sn_planning.loop()
    finally:
        if opti_sn_planning is not None:
            opti_sn_planning.destroy()

if __name__ == '__main__':
    main()
