/**
 * @file        ros_bridge_trajectories.hpp
 * @brief       ROS bridge for trajectories
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __ROS_BRIDGE_TRAJECTORIES__
#define __ROS_BRIDGE_TRAJECTORIES__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PolygonStamped.h>
#include <autohyu_msgs/Trajectories.h>
#include <autohyu_msgs/BehaviorTrajectories.h>
#include <autohyu_msgs/GoalPoints.h>

// Interface Header
#include "interface_trajectories.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"
#include "ros_bridge_geometry_msgs.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::GoalPoints GetGoalPoints(const autohyu_msgs::GoalPoints& msg){
        interface::GoalPoints goal_points;
        for (int i = 0 ; i < msg.goal_points.size(); i++){
            interface::Point point;
            point.x = msg.goal_points[i].x;
            point.y = msg.goal_points[i].y;
            point.z = msg.goal_points[i].z;
            goal_points.point.push_back(point);
        }
        return goal_points;
    }
    
    interface::BehaviorTrajectory GetBehaviorTrajectory(const autohyu_msgs::BehaviorTrajectory& msg) {
        interface::BehaviorTrajectory behavior_trajectory;
        // Header & Id
        behavior_trajectory.header = GetHeader(msg.header);
        behavior_trajectory.id = msg.id;
        // Trajectory point
        for (const auto& point : msg.point) {
            interface::BehaviorTrajectoryPoint btp;
            btp.id = point.id;
            btp.on_ego = point.on_ego;
            btp.on_shortest_path = point.on_shortest_path;
            btp.x = point.x;
            btp.y = point.y;
            btp.z = point.z;
            btp.yaw = point.yaw;
            btp.curvature = point.curvature;
            btp.ref_speed = point.ref_speed;
            btp.speed = point.speed;
            btp.lb_point.x = point.lb_point.x;
            btp.lb_point.y = point.lb_point.y;
            btp.lb_point.z = point.lb_point.z;
            btp.rb_point.x = point.rb_point.x;
            btp.rb_point.y = point.rb_point.y;
            btp.rb_point.z = point.rb_point.z;

            behavior_trajectory.point.push_back(btp);
        }
        // Reference map
        for (const auto& rmp : msg.reference_map){
            interface::BehaviorBoundaryLinePoint bblp;
            bblp.x = rmp.x;
            bblp.y = rmp.y;
            bblp.z = rmp.z;
            bblp.s = rmp.s;
            bblp.n = rmp.n;

            behavior_trajectory.reference_map.push_back(bblp);
        }
        // Right boundary
        for (const auto& rbp : msg.right_boundary){
            interface::BehaviorBoundaryLinePoint bblp;
            bblp.x = rbp.x;
            bblp.y = rbp.y;
            bblp.z = rbp.z;
            bblp.s = rbp.s;
            bblp.n = rbp.n;
            behavior_trajectory.right_boundary.push_back(bblp);
        }
        // Left boundary
        for (const auto& lbp : msg.left_boundary){
            interface::BehaviorBoundaryLinePoint bblp;
            bblp.x = lbp.x;
            bblp.y = lbp.y;
            bblp.z = lbp.z;
            bblp.s = lbp.s;
            bblp.n = lbp.n;
            behavior_trajectory.left_boundary.push_back(bblp);
        }
        // Trajectory cost
        behavior_trajectory.cost.lateral_distance       = msg.cost.lateral_distance;
        behavior_trajectory.cost.lateral_accel          = msg.cost.lateral_accel;
        behavior_trajectory.cost.longitudinal_distance  = msg.cost.longitudinal_distance;
        behavior_trajectory.cost.collision              = msg.cost.collision;
        behavior_trajectory.cost.stability              = msg.cost.stability;
        behavior_trajectory.cost.drivable_area          = msg.cost.drivable_area;
        behavior_trajectory.cost.is_collision           = msg.cost.is_collision;
        behavior_trajectory.cost.total                  = msg.cost.total;
        return behavior_trajectory;
    }

    interface::Trajectory GetTrajectory(const autohyu_msgs::Trajectory& msg, 
                                        const bool& use_boundary = true) {
        interface::Trajectory trajectory;
        trajectory.header = GetHeader(msg.header);
        trajectory.id = msg.id;

        for (const auto& point : msg.point) {            
            interface::TrajectoryPoint tp;
            tp.time = point.time;
            tp.x = point.x;
            tp.y = point.y;
            tp.z = point.z;
            tp.yaw = point.yaw;
            tp.curvature = point.curvature;
            tp.distance = point.distance;
            tp.speed = point.speed;
            tp.acceleration = point.acceleration;

            trajectory.point.push_back(tp);
        }

        if(use_boundary == false){
            return trajectory;
        }

        for (const auto& point : msg.left_boundary.point) {
            interface::TrajectoryBoundaryPoint tbp;
            tbp.x = point.x;
            tbp.y = point.y;
            tbp.z = point.z;
            tbp.s = point.s;
            tbp.n = point.n;

            trajectory.left_boundary.point.push_back(tbp);
        }
        for (const auto& point : msg.right_boundary.point) {
            interface::TrajectoryBoundaryPoint tbp;
            tbp.x = point.x;
            tbp.y = point.y;
            tbp.z = point.z;
            tbp.s = point.s;
            tbp.n = point.n;

            trajectory.right_boundary.point.push_back(tbp);
        }

        return trajectory;
    }   

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    autohyu_msgs::TrajectoryCost UpdateTrajectoryCost(const interface::TrajectoryCost& trajectory_cost) {
        autohyu_msgs::TrajectoryCost msg;
        msg.header = UpdateHeader(trajectory_cost.header);
        
        msg.lateral_difference = trajectory_cost.lateral_difference;
        msg.lateral_distance = trajectory_cost.lateral_distance;
        msg.lateral_accel = trajectory_cost.lateral_accel;
        msg.longitudinal_distance = trajectory_cost.longitudinal_distance;
        msg.collision = trajectory_cost.collision;
        msg.stability = trajectory_cost.stability;
        msg.drivable_area = trajectory_cost.drivable_area;
        msg.is_collision = trajectory_cost.is_collision;

        msg.total = trajectory_cost.total;

        return msg;
    }
    autohyu_msgs::BehaviorTrajectory UpdateBehaviorTrajectory(const interface::BehaviorTrajectory& behavior_trajectory) {
        autohyu_msgs::BehaviorTrajectory msg;
        // Header
        msg.header = UpdateHeader(behavior_trajectory.header);
        msg.header.frame_id = "world";
        msg.id = behavior_trajectory.id;

        // Trajectory point
        for (auto point : behavior_trajectory.point) {
            autohyu_msgs::BehaviorTrajectoryPoint bpoint;
            bpoint.id = point.id;
            bpoint.on_ego = point.on_ego;
            bpoint.on_shortest_path = point.on_shortest_path;
            bpoint.x = point.x;
            bpoint.y = point.y;
            bpoint.z = point.z;
            bpoint.yaw = point.yaw;
            bpoint.curvature = point.curvature;
            bpoint.ref_speed = point.ref_speed;
            bpoint.speed = point.speed;
            bpoint.lb_point.x = point.lb_point.x;
            bpoint.lb_point.y = point.lb_point.y;
            bpoint.lb_point.z = point.lb_point.z;
            bpoint.rb_point.x = point.rb_point.x;
            bpoint.rb_point.y = point.rb_point.y;
            bpoint.rb_point.z = point.rb_point.z;

            msg.point.push_back(bpoint);
        }

        // Reference map
        for(auto rmp : behavior_trajectory.reference_map){
            autohyu_msgs::BehaviorBoundaryPoint blp;
            blp.x = rmp.x;
            blp.y = rmp.y;
            blp.z = rmp.z;
            blp.s = rmp.s;
            blp.n = rmp.n;

            msg.reference_map.push_back(blp);
        }

        // Right boundary
        for(auto rbp : behavior_trajectory.right_boundary){
            autohyu_msgs::BehaviorBoundaryPoint blp;
            blp.x = rbp.x;
            blp.y = rbp.y;
            blp.z = rbp.z;
            blp.s = rbp.s;
            blp.n = rbp.n;

            msg.right_boundary.push_back(blp);
        }

        // Left boundary
        for(auto lbp : behavior_trajectory.left_boundary){
            autohyu_msgs::BehaviorBoundaryPoint blp;
            blp.x = lbp.x;
            blp.y = lbp.y;
            blp.z = lbp.z;
            blp.s = lbp.s;
            blp.n = lbp.n;

            msg.left_boundary.push_back(blp);
        }

        // Cost
        msg.cost = UpdateTrajectoryCost(behavior_trajectory.cost);
        

        return msg;
    }

    std::pair<visualization_msgs::MarkerArray, visualization_msgs::MarkerArray> UpdateRvizForwardBackwardSpeedProfile(const interface::BehaviorTrajectory& trajectory,
                                                                                                       const std::vector<double>& forward,
                                                                                                       const std::vector<double>& backward){
        // Initialize output
        visualization_msgs::MarkerArray forward_msg, backward_msg;

        visualization_msgs::Marker path_marker_forward;
        visualization_msgs::Marker speed_marker_forward;
        visualization_msgs::Marker path_marker_backward;
        visualization_msgs::Marker speed_marker_backward;

        path_marker_forward.header.stamp    = ros::Time::now();
        path_marker_forward.header.frame_id = "world";
        path_marker_forward.ns = "spatial";
        path_marker_forward.id = 0;
        path_marker_forward.action = visualization_msgs::Marker::ADD;
        path_marker_forward.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker_forward.lifetime = ros::Duration(0.1);

        speed_marker_forward.header.stamp    = ros::Time::now();
        speed_marker_forward.header.frame_id = "world";
        speed_marker_forward.ns = "temporal";
        speed_marker_forward.id = 1;
        speed_marker_forward.action = visualization_msgs::Marker::ADD;
        speed_marker_forward.type = visualization_msgs::Marker::LINE_LIST;
        speed_marker_forward.lifetime = ros::Duration(0.1);

        path_marker_backward.header.stamp    = ros::Time::now();
        path_marker_backward.header.frame_id = "world";
        path_marker_backward.ns = "spatial";
        path_marker_backward.id = 0;
        path_marker_backward.action = visualization_msgs::Marker::ADD;
        path_marker_backward.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker_backward.lifetime = ros::Duration(0.1);

        speed_marker_backward.header.stamp    = ros::Time::now();
        speed_marker_backward.header.frame_id = "world";;
        speed_marker_backward.ns = "temporal";
        speed_marker_backward.id = 1;
        speed_marker_backward.action = visualization_msgs::Marker::ADD;
        speed_marker_backward.type = visualization_msgs::Marker::LINE_LIST;
        speed_marker_backward.lifetime = ros::Duration(0.1);

        // Line width
        path_marker_forward.scale.x = 0.25f;
        speed_marker_forward.scale = path_marker_forward.scale;

        path_marker_backward.scale.x = 0.25f;
        speed_marker_backward.scale = path_marker_backward.scale;

        // Color space
        path_marker_forward.color.r = 0.0f;
        path_marker_forward.color.g = 1.0f;
        path_marker_forward.color.b = 0.0f;
        path_marker_forward.color.a = 0.5f;
        speed_marker_forward.color = path_marker_forward.color;

        path_marker_backward.color.r = 1.0f;
        path_marker_backward.color.g = 0.0f;
        path_marker_backward.color.b = 0.0f;
        path_marker_backward.color.a = 1.0f;
        speed_marker_backward.color = path_marker_backward.color;
        for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
            geometry_msgs::Point point;
            point.x = trajectory.point[i].x;
            point.y = trajectory.point[i].y;
            point.z = trajectory.point[i].z;
            path_marker_forward.points.push_back(point);
            path_marker_backward.points.push_back(point);
            speed_marker_forward.points.push_back(point);
            // speed_marker_backward.points.push_back(point);
            point.z = forward[i];
            speed_marker_forward.points.push_back(point);
            point.z = backward[i];
            speed_marker_backward.points.push_back(point);
        }
        forward_msg.markers.push_back(path_marker_forward);
        forward_msg.markers.push_back(speed_marker_forward);
        backward_msg.markers.push_back(path_marker_backward);
        backward_msg.markers.push_back(speed_marker_backward);
        // Return
        return std::make_pair(forward_msg, backward_msg);
    }

    visualization_msgs::MarkerArray UpdateRvizSamplingBehaviorTrajectory(const interface::BehaviorTrajectory& behavior_trajectory) {
        visualization_msgs::MarkerArray msg;

        visualization_msgs::Marker path_marker;
        visualization_msgs::Marker speed_marker;
        visualization_msgs::Marker left_boundary_marker;
        visualization_msgs::Marker right_boundary_marker;

        path_marker.header.stamp    = ros::Time::now();
        path_marker.header.frame_id = "world";
        path_marker.ns = "spatial";
        path_marker.id = 0;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.lifetime = ros::Duration(0.1);

        speed_marker.header = path_marker.header;
        speed_marker.ns = "temporal";
        speed_marker.id = 1;
        speed_marker.action = visualization_msgs::Marker::ADD;
        speed_marker.type = visualization_msgs::Marker::LINE_LIST;
        speed_marker.lifetime = ros::Duration(0.1);

        left_boundary_marker.header = path_marker.header;
        left_boundary_marker.ns = "left_boundary";
        left_boundary_marker.id = 2;
        left_boundary_marker.action = visualization_msgs::Marker::ADD;
        left_boundary_marker.type = visualization_msgs::Marker::LINE_LIST;
        left_boundary_marker.lifetime = ros::Duration(0.1);

        right_boundary_marker.header = path_marker.header;
        right_boundary_marker.ns = "right_boundary_marker";
        right_boundary_marker.id = 3;
        right_boundary_marker.action = visualization_msgs::Marker::ADD;
        right_boundary_marker.type = visualization_msgs::Marker::LINE_LIST;
        right_boundary_marker.lifetime = ros::Duration(0.1);

        // Line width
        path_marker.scale.x         = 0.25f;
        speed_marker.scale          = path_marker.scale;
        left_boundary_marker.scale.x  = 0.75f;
        right_boundary_marker.scale.x = 0.75f;

        // Color space
        path_marker.color.r = 0.0f;
        path_marker.color.g = 0.33f;
        path_marker.color.b = 0.5f;
        path_marker.color.a = 0.9f;
        speed_marker.color = path_marker.color;

        left_boundary_marker.color.r  = 0.0f;
        left_boundary_marker.color.g  = 0.0f;
        left_boundary_marker.color.b  = 1.0f;
        left_boundary_marker.color.a  = 0.9f;
        right_boundary_marker.color   = left_boundary_marker.color;
        // Trajectory point
        for (uint16_t i = 0; i < (uint16_t)behavior_trajectory.point.size(); i++) {
            geometry_msgs::Point point;
            point.x = behavior_trajectory.point[i].x;
            point.y = behavior_trajectory.point[i].y;
            point.z = behavior_trajectory.point[i].z;
            path_marker.points.push_back(point);
            speed_marker.points.push_back(point);
            point.z = behavior_trajectory.point[i].speed * interface::KPH2MPS;
            speed_marker.points.push_back(point);
        }
        msg.markers.push_back(path_marker);
        msg.markers.push_back(speed_marker);

        // Left boundary
        for(auto& lbp :behavior_trajectory.left_boundary){
            geometry_msgs::Point point;
            point.x = lbp.x;
            point.y = lbp.y;
            point.z = lbp.z;
            left_boundary_marker.points.push_back(point);
        }
        msg.markers.push_back(left_boundary_marker);

        for(auto& rbp :behavior_trajectory.right_boundary){
            geometry_msgs::Point point;
            point.x = rbp.x;
            point.y = rbp.y;
            point.z = rbp.z;
            right_boundary_marker.points.push_back(point);
        }
        msg.markers.push_back(right_boundary_marker);

        return msg;
    }

    visualization_msgs::MarkerArray UpdateRvizBehaviorTrajectory(const interface::BehaviorTrajectory& behavior_trajectory) {
        visualization_msgs::MarkerArray msg;
        visualization_msgs::Marker speed_marker;

        speed_marker.header = UpdateHeader(behavior_trajectory.header);
        speed_marker.header.frame_id = "world";
        speed_marker.ns = "speed";
        speed_marker.id = 0;
        speed_marker.action = visualization_msgs::Marker::ADD;
        speed_marker.type = visualization_msgs::Marker::LINE_LIST;
        speed_marker.lifetime = ros::Duration(0.1);
        speed_marker.scale.x = 0.1;
        speed_marker.color.r = 1.0;
        speed_marker.color.g = 1.0;
        speed_marker.color.b = 1.0;
        speed_marker.color.a = 0.4;
        speed_marker.pose.orientation.x = 0.0;
        speed_marker.pose.orientation.y = 0.0;
        speed_marker.pose.orientation.z = 0.0;
        speed_marker.pose.orientation.w = 0.0; 

        for (uint64_t i = 0; i < behavior_trajectory.point.size(); i++) {
            visualization_msgs::Marker point_marker;
            visualization_msgs::Marker edge_marker;
            visualization_msgs::Marker lb_marker, rb_marker;
            visualization_msgs::Marker lb_edge_marker, rb_edge_marker;
            interface::BehaviorTrajectoryPoint point = behavior_trajectory.point[i];
            
            point_marker.header = UpdateHeader(behavior_trajectory.header);
            point_marker.header.frame_id = "world";
            point_marker.ns = "point";
            point_marker.id = point.id;
            point_marker.action = visualization_msgs::Marker::ADD;
            point_marker.type = visualization_msgs::Marker::SPHERE;
            point_marker.lifetime = ros::Duration(0.1);

            edge_marker = point_marker;
            edge_marker.ns = "edge";
            edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
            edge_marker.scale.x = 0.4;
            edge_marker.color.r = 1.0;
            edge_marker.color.g = 0.7;
            edge_marker.color.b = 0.0;
            edge_marker.color.a = 1.0;
            
            point_marker.pose.position.x = point.x;
            point_marker.pose.position.y = point.y;
            point_marker.pose.position.z = 0.0;
            point_marker.pose.orientation.x = 0.0;
            point_marker.pose.orientation.y = 0.0;
            point_marker.pose.orientation.z = 0.0;
            point_marker.pose.orientation.w = 0.0;       
            point_marker.scale.x = 0.5;
            point_marker.scale.y = 0.5;
            point_marker.scale.z = 0.5;     
            point_marker.color.a = 1.0;

            edge_marker.pose.orientation.x = 0.0;
            edge_marker.pose.orientation.y = 0.0;
            edge_marker.pose.orientation.z = 0.0;
            edge_marker.pose.orientation.w = 0.0;

            geometry_msgs::Point p;
            p.x = point_marker.pose.position.x;
            p.y = point_marker.pose.position.y;
            p.z = point_marker.pose.position.z;
            speed_marker.points.push_back(p);
            edge_marker.points.push_back(p);
            p.z = point.speed*interface::KPH2MPS;
            speed_marker.points.push_back(p);

            if (i < behavior_trajectory.point.size()-1) {
                p.x = behavior_trajectory.point[i+1].x;
                p.y = behavior_trajectory.point[i+1].y;
                p.z = behavior_trajectory.point[i+1].z;
                edge_marker.points.push_back(p);
            }

            if (point.on_ego == 0) {
                point_marker.color.r = 1.0;
                point_marker.color.g = 0.0;
                point_marker.color.b = 0.0;
            } 
            else if (point.on_ego == 1) {
                point_marker.color.r = 0.0;
                point_marker.color.g = 1.0;
                point_marker.color.b = 0.0;
            }
            else if (point.on_ego == 2) {
                point_marker.color.r = 0.0;
                point_marker.color.g = 0.0;
                point_marker.color.b = 1.0;
            }

            lb_marker = point_marker;
            lb_marker.ns = "left_bound";
            lb_marker.type = visualization_msgs::Marker::CUBE;
            lb_marker.pose.position.x = point.lb_point.x;
            lb_marker.pose.position.y = point.lb_point.y;
            lb_marker.scale.x = 1.0;
            lb_marker.scale.y = 0.2;
            lb_marker.scale.z = 0.2;     
            lb_marker.color.a = 1.0;

            rb_marker = lb_marker;
            rb_marker.ns = "right_bound";
            rb_marker.pose.position.x = point.rb_point.x;
            rb_marker.pose.position.y = point.rb_point.y;

            lb_edge_marker = edge_marker;
            lb_edge_marker.ns = "left_bound_edge";   
            lb_edge_marker.color = lb_marker.color;  
            lb_edge_marker.points.clear();
            p.x = point.lb_point.x;
            p.y = point.lb_point.y;
            p.z = 0.0;
            lb_edge_marker.points.push_back(p);

            rb_edge_marker = edge_marker;
            rb_edge_marker.ns = "right_bound_edge";
            rb_edge_marker.color = rb_marker.color;
            rb_edge_marker.points.clear();
            p.x = point.rb_point.x;
            p.y = point.rb_point.y;
            p.z = 0.0;
            rb_edge_marker.points.push_back(p);

            if (i < behavior_trajectory.point.size()-1) {
                p.x = behavior_trajectory.point[i+1].lb_point.x;
                p.y = behavior_trajectory.point[i+1].lb_point.y;
                p.z = behavior_trajectory.point[i+1].lb_point.z;
                lb_edge_marker.points.push_back(p);

                p.x = behavior_trajectory.point[i+1].rb_point.x;
                p.y = behavior_trajectory.point[i+1].rb_point.y;
                p.z = behavior_trajectory.point[i+1].rb_point.z;
                rb_edge_marker.points.push_back(p);
            }

            msg.markers.push_back(point_marker);
            msg.markers.push_back(edge_marker);
            msg.markers.push_back(lb_marker);
            msg.markers.push_back(rb_marker);
            msg.markers.push_back(lb_edge_marker);
            msg.markers.push_back(rb_edge_marker);
        }
        msg.markers.push_back(speed_marker);

        return msg;
    }

    autohyu_msgs::Trajectory UpdateTrajectory(const interface::BehaviorTrajectory& trajectory, 
                                              const bool& use_boundary = false) {
        autohyu_msgs::Trajectory msg;
        msg.header              = UpdateHeader(trajectory.header);
        msg.header.frame_id     = "world";
        msg.id                  = trajectory.id;

        for (auto point : trajectory.point) {
            autohyu_msgs::TrajectoryPoint tpoint;
            tpoint.time         = point.time;
            tpoint.x            = point.x;
            tpoint.y            = point.y;
            tpoint.z            = point.z;
            tpoint.yaw          = point.yaw;
            tpoint.curvature    = point.curvature;
            tpoint.distance     = point.distance;
            tpoint.speed        = point.speed;
            tpoint.acceleration = point.acceleration;

            msg.point.push_back(tpoint);
        }

        if (use_boundary == false) {
            return msg;
        }
        
        return msg;
    }

    autohyu_msgs::Trajectory UpdateTrajectory(const interface::Trajectory& trajectory, 
                                              const bool& use_boundary = true) {
        autohyu_msgs::Trajectory msg;
        msg.header              = UpdateHeader(trajectory.header);
        msg.header.frame_id     = "world";
        msg.id                  = trajectory.id;

        for (auto point : trajectory.point) {
            autohyu_msgs::TrajectoryPoint tpoint;
            tpoint.time         = point.time;
            tpoint.x            = point.x;
            tpoint.y            = point.y;
            tpoint.z            = point.z;
            tpoint.yaw          = point.yaw;
            tpoint.curvature    = point.curvature;
            tpoint.distance     = point.distance;
            tpoint.speed        = point.speed;
            tpoint.acceleration = point.acceleration;

            msg.point.push_back(tpoint);
        }

        if (use_boundary == false) {
            return msg;
        }

        for (auto point : trajectory.left_boundary.point) {
            autohyu_msgs::TrajectoryBoundaryPoint tbpoint;
            tbpoint.x = point.x;
            tbpoint.y = point.y;
            tbpoint.z = point.z;
            tbpoint.s = point.s;
            tbpoint.n = point.n;

            msg.left_boundary.point.push_back(tbpoint);
        }

        for (auto point : trajectory.right_boundary.point) {
            autohyu_msgs::TrajectoryBoundaryPoint tbpoint;
            tbpoint.x = point.x;
            tbpoint.y = point.y;
            tbpoint.z = point.z;
            tbpoint.s = point.s;
            tbpoint.n = point.n;

            msg.right_boundary.point.push_back(tbpoint);
        }

        return msg;
    }

    autohyu_msgs::Trajectories UpdateTrajectories(const interface::Trajectories& trajectories) {
        autohyu_msgs::Trajectories msg;
        msg.header.frame_id = "world";
        msg.header = UpdateHeader(trajectories.header);        

        for (auto trajectory : trajectories.trajectory) {
            autohyu_msgs::Trajectory o_trajectory;
            for (auto point : trajectory.point) {
                autohyu_msgs::TrajectoryPoint tpoint;
                tpoint.time = point.time;
                tpoint.x = point.x;
                tpoint.y = point.y;
                tpoint.z = point.z;
                tpoint.yaw = point.yaw;
                tpoint.curvature = point.curvature;
                tpoint.distance = point.distance - trajectory.point.at(0).distance;
                tpoint.speed = point.speed;
                tpoint.acceleration = point.acceleration;

                o_trajectory.point.push_back(tpoint);
            }
            msg.trajectory.push_back(o_trajectory);
        }
        
        return msg;
    }

    visualization_msgs::MarkerArray UpdateRvizTrajectory(const interface::Trajectory& trajectory) {
        visualization_msgs::MarkerArray msg;

        visualization_msgs::Marker path_marker;
        visualization_msgs::Marker speed_marker;

        path_marker.header = UpdateHeader(trajectory.header);
        path_marker.ns = "spatial";
        path_marker.id = 0;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.lifetime = ros::Duration(0.1);

        speed_marker.header = UpdateHeader(trajectory.header);
        speed_marker.ns = "temporal";
        speed_marker.id = 1;
        speed_marker.action = visualization_msgs::Marker::ADD;
        speed_marker.type = visualization_msgs::Marker::LINE_LIST;
        speed_marker.lifetime = ros::Duration(0.1);

        // Line width
        path_marker.scale.x = 0.25f;
        speed_marker.scale = path_marker.scale;

        // Color space
        path_marker.color.r = 0.0f;
        path_marker.color.g = 0.33f;
        path_marker.color.b = 0.5f;
        path_marker.color.a = 0.9f;
        speed_marker.color = path_marker.color;

        for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
            geometry_msgs::Point point;
            point.x = trajectory.point[i].x;
            point.y = trajectory.point[i].y;
            point.z = trajectory.point[i].z;
            path_marker.points.push_back(point);
            speed_marker.points.push_back(point);
            point.z = trajectory.point[i].speed * interface::KPH2MPS;
            speed_marker.points.push_back(point);
        }
        msg.markers.push_back(path_marker);
        msg.markers.push_back(speed_marker);

        return msg;
    }

    visualization_msgs::MarkerArray UpdateRvizTrajectories(const interface::Trajectories& trajectories) {
        
        visualization_msgs::MarkerArray msg;
        uint16_t id = 0;
        
        for (auto trajectory : trajectories.trajectory) {
            visualization_msgs::Marker path_marker;
            visualization_msgs::Marker speed_marker;

            path_marker.header = UpdateHeader(trajectory.header);
            path_marker.ns = "spatial";
            path_marker.id = id++;
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.lifetime = ros::Duration(0.1);

            speed_marker.header = UpdateHeader(trajectory.header);
            speed_marker.ns = "temporal";
            speed_marker.id = id;
            speed_marker.action = visualization_msgs::Marker::ADD;
            speed_marker.type = visualization_msgs::Marker::LINE_LIST;
            speed_marker.lifetime = ros::Duration(0.1);

            // Line width
            path_marker.scale.x = 0.1f;
            speed_marker.scale = path_marker.scale;

            path_marker.color.r = 0.0f;
            path_marker.color.g = 0.38f;
            path_marker.color.b = 0.68f;
            path_marker.color.a = 0.3f;
            speed_marker.color = path_marker.color;

            for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
                geometry_msgs::Point point;
                point.x = trajectory.point[i].x;
                point.y = trajectory.point[i].y;
                point.z = trajectory.point[i].z;
                path_marker.points.push_back(point);
                speed_marker.points.push_back(point);
                point.z = trajectory.point[i].speed * interface::KPH2MPS;
                speed_marker.points.push_back(point);
            }
            msg.markers.push_back(path_marker);
            msg.markers.push_back(speed_marker);
        }

        return msg;
    }

    visualization_msgs::Marker UpdateRvizRouteCandidate(const interface::RouteCandidate& route_candidate, 
                                                        const std::string& ns) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.color.a = 1.0;

        if (route_candidate.segment.size() == 0) {
            return marker;
        }

        for (const auto& segment : route_candidate.segment) {
            for (const auto& point : segment.point) {
                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;

                marker.points.push_back(p);
            }
        }

        if (route_candidate.on_ego == 0) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } 
        else if (route_candidate.on_ego == 1) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if (route_candidate.on_ego == 2) {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        return marker;
    }

    visualization_msgs::MarkerArray UpdateRvizRouteCandidates(const interface::RouteCandidates& route_candidates) {
        visualization_msgs::MarkerArray msg;
        visualization_msgs::Marker marker;

        // Ego Route
        marker = UpdateRvizRouteCandidate(route_candidates.ego_route, "ego_route");
        msg.markers.push_back(marker);

        // Left Route
        marker = UpdateRvizRouteCandidate(route_candidates.left_route, "left_route");
        msg.markers.push_back(marker);

        // Right Route
        marker = UpdateRvizRouteCandidate(route_candidates.right_route, "right_route");
        msg.markers.push_back(marker);

        return msg;
    }

    visualization_msgs::Marker UpdateRvizSearchSpaceEdge(const interface::SearchSpace& search_space,
                                                         const interface::SearchSpacePoint& current_point,
                                                         const uint64_t& next_id,
                                                         const uint64_t& edge_id) {
        visualization_msgs::Marker edge_marker;

        edge_marker.header = UpdateHeader(search_space.header);
        edge_marker.header.frame_id = "world";
        edge_marker.ns = "search_space_edge";
        edge_marker.id = edge_id;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
        edge_marker.lifetime = ros::Duration(0.1);
        edge_marker.scale.x = 0.1;
        edge_marker.color.r = 1.0;
        edge_marker.color.g = 0.7;
        edge_marker.color.b = 0.0;
        edge_marker.color.a = 0.3;

        geometry_msgs::Point p;
        p.x = current_point.x;
        p.y = current_point.y;
        p.z = current_point.z;

        edge_marker.points.push_back(p);

        for (const auto& point : search_space.point) {
            if (point.id == next_id) {
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;

                edge_marker.points.push_back(p);
                break;
            }
        }

        return edge_marker;
    }

    visualization_msgs::MarkerArray UpdateRvizSearchSpace(const interface::SearchSpace& search_space) {
        visualization_msgs::MarkerArray msg;
        visualization_msgs::Marker speed_marker;

        speed_marker.header = UpdateHeader(search_space.header);
        speed_marker.header.frame_id = "world";
        speed_marker.ns = "search_space_speed";
        speed_marker.id = 0;
        speed_marker.action = visualization_msgs::Marker::ADD;
        speed_marker.type = visualization_msgs::Marker::LINE_LIST;
        speed_marker.lifetime = ros::Duration(0.1);
        speed_marker.scale.x = 0.1;
        speed_marker.color.r = 1.0;
        speed_marker.color.g = 1.0;
        speed_marker.color.b = 1.0;
        speed_marker.color.a = 0.3;

        uint64_t edge_id = 0;
        for (const auto& point : search_space.point) {
            visualization_msgs::Marker node_marker;
            visualization_msgs::Marker lb_marker, rb_marker;
            visualization_msgs::Marker id_marker;
            for (const auto& next_id : point.next_id) {
                visualization_msgs::Marker edge_marker = UpdateRvizSearchSpaceEdge(search_space, point, next_id, edge_id);
                msg.markers.push_back(edge_marker);
                edge_id++;
            }

            node_marker.header = UpdateHeader(search_space.header);
            node_marker.header.frame_id = "world";
            node_marker.ns = "search_space_node";
            node_marker.id = point.id;
            node_marker.action = visualization_msgs::Marker::ADD;
            node_marker.type = visualization_msgs::Marker::SPHERE;
            node_marker.lifetime = ros::Duration(0.1);
            
            node_marker.pose.position.x = point.x;
            node_marker.pose.position.y = point.y;
            node_marker.pose.position.z = 0.0;
            node_marker.pose.orientation.x = 0.0;
            node_marker.pose.orientation.y = 0.0;
            node_marker.pose.orientation.z = 0.0;
            node_marker.pose.orientation.w = 0.0;       
            node_marker.scale.x = 0.3;
            node_marker.scale.y = 0.3;
            node_marker.scale.z = 0.3;    

            if (point.on_shortest_path == false) {
                node_marker.color.a = 0.3;
            }
            else {
                node_marker.color.a = 1.0;
            }            

            id_marker = node_marker;
            id_marker.ns = "search_space_id";
            id_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            id_marker.pose.position.z = 3.0;
            id_marker.text = "id: " + std::to_string(point.id);
            id_marker.color.r = 1.0;
            id_marker.color.g = 1.0;
            id_marker.color.b = 1.0;

            geometry_msgs::Point p;
            p.x = node_marker.pose.position.x;
            p.y = node_marker.pose.position.y;
            p.z = node_marker.pose.position.z;
            speed_marker.points.push_back(p);
            p.z = point.speed*interface::KPH2MPS;
            speed_marker.points.push_back(p);

            if (point.on_ego == 0) {
                node_marker.color.r = 1.0;
                node_marker.color.g = 0.0;
                node_marker.color.b = 0.0;
            } 
            else if (point.on_ego == 1) {
                node_marker.color.r = 0.0;
                node_marker.color.g = 1.0;
                node_marker.color.b = 0.0;
            }
            else if (point.on_ego == 2) {
                node_marker.color.r = 0.0;
                node_marker.color.g = 0.0;
                node_marker.color.b = 1.0;
            }

            lb_marker = node_marker;
            lb_marker.ns = "search_space_lb";
            lb_marker.type = visualization_msgs::Marker::CUBE;
            lb_marker.pose.position.x = point.lb_point.x;
            lb_marker.pose.position.y = point.lb_point.y;
            lb_marker.scale.x = 0.7;
            lb_marker.scale.y = 0.1;
            lb_marker.scale.z = 0.2;     
            lb_marker.color.a = 0.3;

            rb_marker = lb_marker;
            rb_marker.ns = "search_space_rb";
            rb_marker.pose.position.x = point.rb_point.x;
            rb_marker.pose.position.y = point.rb_point.y;

            msg.markers.push_back(node_marker);
            msg.markers.push_back(id_marker);
            msg.markers.push_back(lb_marker);
            msg.markers.push_back(rb_marker);
        }
        msg.markers.push_back(speed_marker);

        return msg;
    }

    std::vector<geometry_msgs::Point> ConvertSamplingBoundaryLaneToPoints(const interface::SamplingBoundaryLane& segment, const std_msgs::Header& header) {
        std::vector<geometry_msgs::Point> points;
        // 1. Left line forward
        for(uint16_t idx = 0 ; idx < (uint16_t)segment.left.point.size() ; idx++){
            geometry_msgs::Point point;
            
            point.x = segment.left.point[idx].x;
            point.y = segment.left.point[idx].y;
            point.z = 0.0;
            points.push_back(point);
        }
        // 2. Right line backward
        uint16_t size_of_right = (uint16_t)segment.right.point.size();
        for(uint16_t idx = 0 ; idx < size_of_right ; idx++){
            geometry_msgs::Point point;
            
            point.x = segment.right.point[(size_of_right - 1) - idx].x;
            point.y = segment.right.point[(size_of_right - 1) - idx].y;
            point.z = 0.0;
            points.push_back(point);
        }
        // 3. First point
        geometry_msgs::Point point;
            
        point.x = segment.left.point[0].x;
        point.y = segment.left.point[0].y;
        point.z = 0.0;
        points.push_back(point);

        return points;
    }

    visualization_msgs::MarkerArray UpdateRvizSamplingBoundary(const interface::SamplingBoundary& sampling_boundary){
        visualization_msgs::MarkerArray marker_array;

        // marker_array.header = UpdateHeader(sampling_boundary.header);
        int i = 0;
        for (const auto& segment : sampling_boundary.segment) {
            // Center lane
            for(const auto& center_lane : segment.center){
                if (center_lane.is_exist) {
                    visualization_msgs::Marker marker;
                    marker.header   = UpdateHeader(sampling_boundary.header);
                    marker.ns       = "Center Segment";
                    marker.id       = i;
                    marker.action   = visualization_msgs::Marker::ADD;
                    marker.type     = visualization_msgs::Marker::LINE_STRIP;
                    marker.lifetime = ros::Duration(1.2);           
                    marker.scale.x  = 0.6;
                    marker.scale.y  = 0.6;
                    marker.scale.z  = 0.6;
                    marker.color.r  = 1.0;
                    marker.color.g  = 0.0;
                    marker.color.b  = 0.0;
                    marker.color.a  = 1.0;
                    std::vector<geometry_msgs::Point> points = ConvertSamplingBoundaryLaneToPoints(center_lane, marker.header);
                    for(auto& point : points){
                        marker.points.push_back(point);
                    }
                    marker_array.markers.push_back(marker);
                    i++;
                }
            }
            // Right lane
            if (segment.right.is_exist) {
                visualization_msgs::Marker marker;
                marker.header   = UpdateHeader(sampling_boundary.header);
                marker.ns       = "Right Segment";
                marker.id       = i;
                marker.action   = visualization_msgs::Marker::ADD;
                marker.type     = visualization_msgs::Marker::LINE_STRIP;
                marker.lifetime = ros::Duration(1.2);           
                marker.scale.x  = 0.6;
                marker.scale.y  = 0.6;
                marker.scale.z  = 0.6;
                marker.color.r  = 0.0;
                marker.color.g  = 1.0;
                marker.color.b  = 0.0;
                marker.color.a  = 1.0;
                std::vector<geometry_msgs::Point> points = ConvertSamplingBoundaryLaneToPoints(segment.right, marker.header);
                for(auto& point : points){
                    marker.points.push_back(point);
                }
                marker_array.markers.push_back(marker);
                i++;
            }
            // Left lane
            if (segment.left.is_exist) {
                visualization_msgs::Marker marker;
                marker.header   = UpdateHeader(sampling_boundary.header);
                marker.ns       = "Left Segment";
                marker.id       = i;
                marker.action   = visualization_msgs::Marker::ADD;
                marker.type     = visualization_msgs::Marker::LINE_STRIP;
                marker.lifetime = ros::Duration(1.2);           
                marker.scale.x  = 0.6;
                marker.scale.y  = 0.6;
                marker.scale.z  = 0.6;
                marker.color.r  = 0.0;
                marker.color.g  = 0.0;
                marker.color.b  = 1.0;
                marker.color.a  = 1.0;
                std::vector<geometry_msgs::Point> points = ConvertSamplingBoundaryLaneToPoints(segment.left, marker.header);
                for(auto& point : points){
                    marker.points.push_back(point);
                }
                marker_array.markers.push_back(marker);
                i++;
            }
        }

        return marker_array;
    }

    visualization_msgs::MarkerArray UpdateRvizBehaviorTrajectories(const interface::BehaviorTrajectories& trajectories) {
        visualization_msgs::MarkerArray msg;
        uint16_t id = 0;

        for (const auto& trajectory : trajectories.trajectory) {
            visualization_msgs::Marker path_marker;

            path_marker.header = UpdateHeader(trajectories.header);
            path_marker.header.frame_id = "world";
            path_marker.ns = "path";
            path_marker.id = id++;
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.lifetime = ros::Duration(0.1);

            path_marker.scale.x = 0.1f;
            path_marker.color.a = 0.2f;

            path_marker.color.r = 0.0f;
            path_marker.color.g = 1.0f;
            path_marker.color.b = 0.0f;

            for (const auto& point : trajectory.point) {
                geometry_msgs::Point p;
                p.x = point.x;
                p.y = point.y;
                // p.z = point.time*5.0;
                p.z = 0.0;
                path_marker.points.push_back(p);
            }
            msg.markers.push_back(path_marker);
        }

        return msg;
    }

    visualization_msgs::MarkerArray UpdateRvizSamplingReferenceLine(const interface::SamplingBoundaryLine& reference_line) {
        visualization_msgs::MarkerArray msg;

        visualization_msgs::Marker path_marker;
        visualization_msgs::Marker speed_marker;

        path_marker.header.stamp    = ros::Time::now();
        path_marker.header.frame_id = "world";
        path_marker.ns = "spatial";
        path_marker.id = 0;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.lifetime = ros::Duration(0.1);

        speed_marker.header = path_marker.header;
        speed_marker.ns = "temporal";
        speed_marker.id = 1;
        speed_marker.action = visualization_msgs::Marker::ADD;
        speed_marker.type = visualization_msgs::Marker::LINE_LIST;
        speed_marker.lifetime = ros::Duration(0.1);

        // Line width
        path_marker.scale.x = 0.25f;
        speed_marker.scale = path_marker.scale;

        // Color space
        path_marker.color.r = 1.0f;
        path_marker.color.g = 0.33f;
        path_marker.color.b = 0.f;
        path_marker.color.a = 0.6f;
        speed_marker.color = path_marker.color;

        for (uint16_t i = 0; i < (uint16_t)reference_line.point.size(); i++) {
            geometry_msgs::Point point;
            point.x = reference_line.point[i].x;
            point.y = reference_line.point[i].y;
            point.z = 0.0;
            path_marker.points.push_back(point);
            speed_marker.points.push_back(point);
            point.z = reference_line.point[i].z * interface::KPH2MPS;
            speed_marker.points.push_back(point);
        }
        msg.markers.push_back(path_marker);
        msg.markers.push_back(speed_marker);

        return msg;
    }

    visualization_msgs::MarkerArray UpdateRvizGoalPoints(const interface::GoalPoints& points,
                                                        const std::string& ns,
                                                        const double& color_r, 
                                                        const double& color_g, 
                                                        const double& color_b, 
                                                        const double& color_a) {
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < points.point.size(); ++i) {
            visualization_msgs::Marker msg;

            // msg.header = UpdateHeader(points[i].header);
            msg.header.seq = i;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "world";
            msg.ns = ns + "_" + std::to_string(i + 1); 
            msg.id = i;  // 각 마커에 고유 ID를 부여
            msg.type = visualization_msgs::Marker::SPHERE;
            msg.action = visualization_msgs::Marker::ADD;
            msg.pose.position = UpdatePoint(points.point[i]);
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;
            msg.pose.orientation.w = 1.0;
            msg.scale.x = 2.5;
            msg.scale.y = 2.5;
            msg.scale.z = 2.5;
            msg.color.r = color_r;
            msg.color.g = color_g;
            msg.color.b = color_b;
            msg.color.a = color_a;
            msg.lifetime = ros::Duration(1);

            marker_array.markers.push_back(msg);
        }

        return marker_array;
    }
} // namespace ros_bridge

#endif  // __ROS_BRIDGE_TRAJECTORIES__