/**
 * @file        ros_bridge_geometry_msgs.hpp
 * @brief       ROS bridge for geometry_msgs
 * 
 * @authors     Seheon Ha (seheonha@hanyang.ac.kr)          
 * 
 * @date        2024-4-25 Created   
 */

#ifndef __ROS_BRIDGE_POSE_WITH_COVARIANCE_STAMPPED__
#define __ROS_BRIDGE_POSE_WITH_COVARIANCE_STAMPPED__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"

// Interface Header
#include "interface_geometry_msgs.hpp"
#include "interface_trajectories.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::Point GetPoint(const geometry_msgs::Point& msg){
        interface::Point point;
        point.x = msg.x;
        point.y = msg.y;
        point.z = msg.z;

        return point;
    }

    interface::Quaternion GetQuaternion(const geometry_msgs::Quaternion& msg){
        interface::Quaternion quaternion;
        quaternion.x = msg.x;
        quaternion.y = msg.y;
        quaternion.z = msg.z;
        quaternion.w = msg.w;

        return quaternion;
    }

    interface::Pose GetPose(const geometry_msgs::Pose& msg){
        interface::Pose pose;
        pose.position = GetPoint(msg.position);
        pose.orientation = GetQuaternion(msg.orientation);
        return pose;
    }

    interface::PointStamped GetPointStamped(const geometry_msgs::PointStamped& msg) {
        interface::PointStamped point_stamped;
        point_stamped.header = GetHeader(msg.header);
        point_stamped.point = GetPoint(msg.point);

        return point_stamped;
    }

    interface::PointStamped GetPointStamped(const geometry_msgs::PoseStamped& msg) {
        interface::PointStamped point_stamped;
        point_stamped.header = GetHeader(msg.header);
        point_stamped.point = GetPoint(msg.pose.position);

        return point_stamped;
    }

    interface::PoseStamped GetPoseStamped(const geometry_msgs::PoseStamped& msg){
        interface::PoseStamped pose_stamped;
        pose_stamped.header = GetHeader(msg.header);
        pose_stamped.pose   = GetPose(msg.pose);

        return pose_stamped;
    }



    interface::GoalPoints GetPoints(const std::vector<geometry_msgs::Point>& msg){
        interface::GoalPoints points;
        interface::Point point;
        for (int i = 0; i < msg.size(); i++){
            point = GetPoint(msg[i]);
            points.point.push_back(point);
        }

        return points;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    geometry_msgs::Point UpdatePoint(const interface::Point& point) {
        geometry_msgs::Point msg;

        msg.x = point.x;
        msg.y = point.y;
        msg.z = point.z;

        return msg;
    }

    geometry_msgs::PointStamped UpdatePointStamped(const interface::PointStamped& point_stamped) {
        geometry_msgs::PointStamped msg;

        msg.header = UpdateHeader(point_stamped.header);
        msg.point  = UpdatePoint(point_stamped.point);

        return msg;
    }

    visualization_msgs::Marker UpdateRvizPointStamped(const interface::PointStamped& point_stamped,
                                                      const std::string& ns,
                                                      const double& color_r, 
                                                      const double& color_g, 
                                                      const double& color_b, 
                                                      const double& color_a) {
        visualization_msgs::Marker msg;

        msg.header = UpdateHeader(point_stamped.header);
        msg.ns = ns;
        msg.id = 0;
        msg.type = visualization_msgs::Marker::SPHERE;
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.position = UpdatePoint(point_stamped.point);
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = 0.1;
        msg.scale.y = 0.1;
        msg.scale.z = 0.1;
        msg.color.r = color_r;
        msg.color.g = color_g;
        msg.color.b = color_b;
        msg.color.a = color_a;
        msg.lifetime = ros::Duration(0.1);

        return msg;
    }


} // namespace ros_bridge

#endif  // __ROS_BRIDGE_GEOMETRY_MSGS__
