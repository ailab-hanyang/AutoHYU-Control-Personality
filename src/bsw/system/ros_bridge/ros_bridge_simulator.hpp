/**
 * @file        ros_bridge_simulator.hpp
 * @brief       ROS bridge for simulator
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-07-17 created by Yuseung Na
 */

#ifndef __ROS_BRIDGE_SIMULATOR_HPP__
#define __ROS_BRIDGE_SIMULATOR_HPP__
#pragma once

// ROS Header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <morai_msgs/ObjectStatus.h>
#include <morai_msgs/ObjectStatusList.h>
#include <autohyu_msgs/DetectObjects3D.h>
#include <autohyu_msgs/TrackObjects.h>
#include <visualization_msgs/MarkerArray.h>

// Interface Header
#include "interface_objects.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"

// Utility header
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/UTM.h>

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::TrackObjects GetMoraiObjects(const morai_msgs::ObjectStatusList& msg,
                                            const double& ref_lat,
                                            const double& ref_lon) {
        interface::TrackObjects objects;
        objects.header = GetHeader(msg.header);
        objects.header.frame_id = "world";

        for (const auto& i_object : msg.npc_list) {
            interface::TrackObject object;
            object.id = i_object.unique_id;
            // object.classification = i_object.type;
            // object.dynamic_state;

            object.dimension.length = i_object.size.x;
            object.dimension.width  = i_object.size.y;
            object.dimension.height = i_object.size.z;

            object.state.header = GetHeader(msg.header);

            // Convert ego_yaw to radians
            double theta = i_object.heading * interface::DEG2RAD;

            // Rotation matrix elements
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);

            // Perform rotation
            double global_vx = i_object.velocity.x * interface::KPH2MPS * cos_theta 
                                - i_object.velocity.y * interface::KPH2MPS * sin_theta;
            double global_vy = i_object.velocity.x * interface::KPH2MPS * sin_theta 
                                + i_object.velocity.y * interface::KPH2MPS * cos_theta;

            double global_ax = i_object.acceleration.x * cos_theta 
                                - i_object.acceleration.y * sin_theta;
            double global_ay = i_object.acceleration.x * sin_theta 
                                + i_object.acceleration.y * cos_theta;

            object.state.header = objects.header;
            object.state.yaw    = i_object.heading * interface::DEG2RAD;
            object.state.v_x    = global_vx;
            object.state.v_y    = global_vy;
            object.state.a_x    = global_ax;
            object.state.a_y    = global_ay;

            // 1. Morai Postion to GPS (reverse projection)
            lanelet::BasicPoint3d object_morai_position(i_object.position.x,i_object.position.y,i_object.position.z);
            // lanelet::projection::LocalCartesianProjector morai_projector(lanelet::Origin({cfg_ref_lat_morai_, cfg_ref_lon_morai_}));
            lanelet::projection::UtmProjector morai_projector(lanelet::Origin({ref_lat, ref_lon}));
            lanelet::GPSPoint object_gps_point = morai_projector.reverse(object_morai_position);
            
            // 2. GPS to World Position (forward projection)
            lanelet::projection::LocalCartesianProjector autohyu_projector(lanelet::Origin({ref_lat, ref_lon}));
            lanelet::BasicPoint3d object_autohyu_projpos = autohyu_projector.forward(object_gps_point);
            
            object.state.x = object_autohyu_projpos.x() + object.dimension.length/2.0 * cos(object.state.yaw);
            object.state.y = object_autohyu_projpos.y() + object.dimension.length/2.0 * sin(object.state.yaw);

            objects.object.push_back(object);
        }

        for (const auto& i_object : msg.obstacle_list) {
            interface::TrackObject object;
            object.id = i_object.unique_id;
            // object.classification = i_object.type;
            // object.dynamic_state;

            object.dimension.length = i_object.size.x;
            object.dimension.width  = i_object.size.y;
            object.dimension.height = i_object.size.z;

            object.state.header = GetHeader(msg.header);

            // Convert ego_yaw to radians
            double theta = i_object.heading * interface::DEG2RAD;

            // Rotation matrix elements
            double cos_theta = cos(theta);
            double sin_theta = sin(theta);

            // Perform rotation
            double global_vx = i_object.velocity.x * interface::KPH2MPS * cos_theta 
                                - i_object.velocity.y * interface::KPH2MPS * sin_theta;
            double global_vy = i_object.velocity.x * interface::KPH2MPS * sin_theta 
                                + i_object.velocity.y * interface::KPH2MPS * cos_theta;

            double global_ax = i_object.acceleration.x * cos_theta 
                                - i_object.acceleration.y * sin_theta;
            double global_ay = i_object.acceleration.x * sin_theta 
                                + i_object.acceleration.y * cos_theta;

            object.state.header = objects.header;
            object.state.yaw    = i_object.heading * interface::DEG2RAD;
            object.state.v_x    = global_vx;
            object.state.v_y    = global_vy;
            object.state.a_x    = global_ax;
            object.state.a_y    = global_ay;

            // 1. Morai Postion to GPS (reverse projection)
            lanelet::BasicPoint3d object_morai_position(i_object.position.x,i_object.position.y,i_object.position.z);
            // lanelet::projection::LocalCartesianProjector morai_projector(lanelet::Origin({cfg_ref_lat_morai_, cfg_ref_lon_morai_}));
            lanelet::projection::UtmProjector morai_projector(lanelet::Origin({ref_lat, ref_lon}));
            lanelet::GPSPoint object_gps_point = morai_projector.reverse(object_morai_position);
            
            // 2. GPS to World Position (forward projection)
            lanelet::projection::LocalCartesianProjector autohyu_projector(lanelet::Origin({ref_lat, ref_lon}));
            lanelet::BasicPoint3d object_autohyu_projpos = autohyu_projector.forward(object_gps_point);
            
            object.state.x = object_autohyu_projpos.x() + object.dimension.length/2.0 * cos(object.state.yaw);
            object.state.y = object_autohyu_projpos.y() + object.dimension.length/2.0 * sin(object.state.yaw);

            objects.object.push_back(object);
        }

        return objects;
    }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //   

    
} // namespace ros_bridge

#endif // __ROS_BRIDGE_SIMULATOR_HPP__