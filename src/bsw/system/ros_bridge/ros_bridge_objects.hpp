/**
 * @file        ros_bridge_objects.hpp
 * @brief       ROS bridge for objects
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-01 updated by Yuseung Na: Refactoring 
 */

#ifndef __ROS_BRIDGE_OBJECTS__
#define __ROS_BRIDGE_OBJECTS__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <autohyu_msgs/DetectObjects2D.h>
#include <autohyu_msgs/DetectObjects3D.h>
#include <autohyu_msgs/TrackObjects.h>
#include <autohyu_msgs/PredictObjects.h>
#include <visualization_msgs/MarkerArray.h>

// Interface Header
#include "interface_objects.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::DetectObjects3D GetDetectObjects3D(const autohyu_msgs::DetectObjects3D& msg) {
        interface::DetectObjects3D objects;
        objects.header = ros_bridge::GetHeader(msg.header);

        for (auto i_object : msg.object) {
            interface::DetectObject3D object;
            object.id = i_object.id;
            object.confidence_score = i_object.confidence_score;
            object.classification = static_cast<interface::ObjectClass>(i_object.classification);
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            object.state.header = ros_bridge::GetHeader(i_object.state.header);
            object.state.x = i_object.state.x;
            object.state.y = i_object.state.y;
            object.state.z = i_object.state.z;
            object.state.roll = i_object.state.roll;
            object.state.pitch = i_object.state.pitch;
            object.state.yaw = i_object.state.yaw;
            object.state.v_x = i_object.state.v_x;
            object.state.v_y = i_object.state.v_y;
            object.state.v_z = i_object.state.v_z;
            object.state.a_x = i_object.state.a_x;
            object.state.a_y = i_object.state.a_y;
            object.state.a_z = i_object.state.a_z;
            object.state.roll_rate = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate = i_object.state.yaw_rate;

            objects.object.push_back(object);
        }

        return objects;
    }  

    interface::TrackObjects GetTrackObjects(const autohyu_msgs::TrackObjects& msg) {
        interface::TrackObjects objects;
        objects.header = ros_bridge::GetHeader(msg.header);

        for (auto i_object : msg.object) {
            interface::TrackObject object;
            object.id = i_object.id;
            object.classification = static_cast<interface::ObjectClass>(i_object.classification);
            object.dynamic_state = static_cast<interface::ObjectDynamicState>(i_object.dynamic_state);
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;
            
            object.state.header = GetHeader(i_object.state.header);
            object.state.x = i_object.state.x;
            object.state.y = i_object.state.y;
            object.state.z = i_object.state.z;
            object.state.roll = i_object.state.roll;
            object.state.pitch = i_object.state.pitch;
            object.state.yaw = i_object.state.yaw;
            object.state.v_x = i_object.state.v_x;
            object.state.v_y = i_object.state.v_y;
            object.state.v_z = i_object.state.v_z;
            object.state.a_x = i_object.state.a_x;
            object.state.a_y = i_object.state.a_y;
            object.state.a_z = i_object.state.a_z;
            object.state.roll_rate = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate = i_object.state.yaw_rate;
            
            object.state_covariance.x = i_object.state_covariance.x;
            object.state_covariance.y = i_object.state_covariance.y;
            object.state_covariance.z = i_object.state_covariance.z;
            object.state_covariance.roll = i_object.state_covariance.roll;
            object.state_covariance.pitch = i_object.state_covariance.pitch;
            object.state_covariance.yaw = i_object.state_covariance.yaw;
            object.state_covariance.v_x = i_object.state_covariance.v_x;
            object.state_covariance.v_y = i_object.state_covariance.v_y;
            object.state_covariance.v_z = i_object.state_covariance.v_z;
            object.state_covariance.a_x = i_object.state_covariance.a_x;
            object.state_covariance.a_y = i_object.state_covariance.a_y;
            object.state_covariance.a_z = i_object.state_covariance.a_z;
            object.state_covariance.roll_rate = i_object.state_covariance.roll_rate;
            object.state_covariance.pitch_rate = i_object.state_covariance.pitch_rate;
            object.state_covariance.yaw_rate = i_object.state_covariance.yaw_rate;

            objects.object.push_back(object);
        }

        return objects;
    }

    interface::PredictObjects GetPredictObjects(const autohyu_msgs::PredictObjects& msg) {
        interface::PredictObjects objects;
        objects.header = ros_bridge::GetHeader(msg.header);

        for (auto i_object : msg.object) {
            interface::PredictObject object;
            object.id = i_object.id;
            object.classification = static_cast<interface::ObjectClass>(i_object.classification);
            object.dynamic_state = static_cast<interface::ObjectDynamicState>(i_object.dynamic_state);
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            for (const auto& i_state : i_object.state) {       
                interface::Object3DState state;
                state.header = ros_bridge::GetHeader(i_state.header);
                state.x = i_state.x;
                state.y = i_state.y;
                state.z = i_state.z;
                state.roll = i_state.roll;
                state.pitch = i_state.pitch;
                state.yaw = i_state.yaw;
                state.v_x = i_state.v_x;
                state.v_y = i_state.v_y;
                state.v_z = i_state.v_z;
                state.a_x = i_state.a_x;
                state.a_y = i_state.a_y;
                state.a_z = i_state.a_z;
                state.roll_rate = i_state.roll_rate;
                state.pitch_rate = i_state.pitch_rate;
                state.yaw_rate = i_state.yaw_rate;

                object.state.push_back(state);
            }

            objects.object.push_back(object);
        }

        return objects;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //    
    autohyu_msgs::DetectObjects2D UpdateDetectObjects2D(const interface::DetectObjects2D& detect_objects){
        autohyu_msgs::DetectObjects2D msg;        
        msg.header = ros_bridge::UpdateHeader(detect_objects.header);

        for (auto i_object : detect_objects.object){
            autohyu_msgs::DetectObject2D object;    
            object.id               = i_object.id;
            object.confidence_score = i_object.confidence_score;
            object.classification   = i_object.classification;
            object.dimension.length = i_object.dimension.length;
            object.dimension.width  = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;
            object.state.x          = i_object.state.x;
            object.state.y          = i_object.state.y;    

            msg.object.push_back(object);        
        }

        return msg;
    }

    autohyu_msgs::DetectObjects3D UpdateDetectObjects3D(const interface::DetectObjects3D& detect_objects){
        autohyu_msgs::DetectObjects3D msg;        
        msg.header = ros_bridge::UpdateHeader(detect_objects.header);

        for (auto i_object : detect_objects.object){
            autohyu_msgs::DetectObject3D object;    
            object.id               = i_object.id;
            object.confidence_score = i_object.confidence_score;
            object.classification   = i_object.classification;
            object.dimension.length = i_object.dimension.length;
            object.dimension.width  = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;
            object.state.x          = i_object.state.x;
            object.state.y          = i_object.state.y;
            object.state.z          = i_object.state.z;
            object.state.roll       = i_object.state.roll;
            object.state.pitch      = i_object.state.pitch;
            object.state.yaw        = i_object.state.yaw;
            object.state.v_x        = i_object.state.v_x;
            object.state.v_y        = i_object.state.v_y;
            object.state.v_z        = i_object.state.v_z;
            object.state.a_x        = i_object.state.a_x;
            object.state.a_y        = i_object.state.a_y;
            object.state.a_z        = i_object.state.a_z;
            object.state.roll_rate  = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate   = i_object.state.yaw_rate;

            msg.object.push_back(object);
        }

        return msg;
    }

    autohyu_msgs::TrackObjects UpdateTrackObjects(const interface::TrackObjects& objects) {
        autohyu_msgs::TrackObjects msg;
        msg.header = ros_bridge::UpdateHeader(objects.header);

        for (const auto& i_object : objects.object) {
            autohyu_msgs::TrackObject object;
            object.id = i_object.id;
            object.classification = static_cast<uint8_t>(i_object.classification);
            object.dynamic_state = static_cast<uint8_t>(i_object.dynamic_state);
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            object.state.header = ros_bridge::UpdateHeader(i_object.state.header);
            object.state.x = i_object.state.x;
            object.state.y = i_object.state.y;
            object.state.z = i_object.state.z;
            object.state.roll = i_object.state.roll;
            object.state.pitch = i_object.state.pitch;
            object.state.yaw = i_object.state.yaw;
            object.state.v_x = i_object.state.v_x;
            object.state.v_y = i_object.state.v_y;
            object.state.v_z = i_object.state.v_z;
            object.state.a_x = i_object.state.a_x;
            object.state.a_y = i_object.state.a_y;
            object.state.a_z = i_object.state.a_z;
            object.state.roll_rate = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate = i_object.state.yaw_rate;

            object.state_covariance.x = i_object.state_covariance.x;
            object.state_covariance.y = i_object.state_covariance.y;
            object.state_covariance.z = i_object.state_covariance.z;
            object.state_covariance.roll = i_object.state_covariance.roll;
            object.state_covariance.pitch = i_object.state_covariance.pitch;
            object.state_covariance.yaw = i_object.state_covariance.yaw;
            object.state_covariance.v_x = i_object.state_covariance.v_x;
            object.state_covariance.v_y = i_object.state_covariance.v_y;
            object.state_covariance.v_z = i_object.state_covariance.v_z;
            object.state_covariance.a_x = i_object.state_covariance.a_x;
            object.state_covariance.a_y = i_object.state_covariance.a_y;
            object.state_covariance.a_z = i_object.state_covariance.a_z;
            object.state_covariance.roll_rate = i_object.state_covariance.roll;
            object.state_covariance.pitch_rate = i_object.state_covariance.pitch_rate;
            object.state_covariance.yaw_rate = i_object.state_covariance.yaw_rate;

            msg.object.push_back(object);
        }

        return msg;
    }

    autohyu_msgs::PredictObjects UpdatePredictObjects(interface::PredictObjects& objects) {
        autohyu_msgs::PredictObjects msg;
        msg.header = ros_bridge::UpdateHeader(objects.header);

        for (auto& i_object : objects.object) {
            autohyu_msgs::PredictObject object;
            object.id = i_object.id;
            object.classification = static_cast<uint8_t>(i_object.classification);
            object.dynamic_state = static_cast<uint8_t>(i_object.dynamic_state);
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            double highest_probability = 0.0;
            for (const auto& prediction : i_object.state_multi) {
                autohyu_msgs::PredictObjectMultimodal state_multi;
                for (const auto& i_state : prediction.state) {
                    autohyu_msgs::Object3DState state;
                    state.header = ros_bridge::UpdateHeader(i_state.header);
                    state.x = i_state.x;
                    state.y = i_state.y;
                    state.z = i_state.z;
                    state.roll = i_state.roll;
                    state.pitch = i_state.pitch;
                    state.yaw = i_state.yaw;
                    state.v_x = i_state.v_x;
                    state.v_y = i_state.v_y;
                    state.v_z = i_state.v_z;
                    state.a_x = i_state.a_x;
                    state.a_y = i_state.a_y;
                    state.a_z = i_state.a_z;
                    state.roll_rate = i_state.roll_rate;
                    state.pitch_rate = i_state.pitch_rate;
                    state.yaw_rate = i_state.yaw_rate;

                    state_multi.state.push_back(state);
                }
                object.state_multi.push_back(state_multi);

                // Update highest probability
                if (prediction.probability > highest_probability) {
                    highest_probability = prediction.probability;
                    object.state = state_multi.state;
                    i_object.state = prediction.state;
                }
            }            

            msg.object.push_back(object);
        }

        return msg;
    }

    visualization_msgs::MarkerArray UpdateRvizTrackObjects(const interface::TrackObjects& objects) {
        visualization_msgs::MarkerArray msg;

        for (const auto& object : objects.object) {
            visualization_msgs::Marker position_marker;
            visualization_msgs::Marker track_info_marker;

            position_marker.header = ros_bridge::UpdateHeader(objects.header);
            position_marker.ns = "position";
            position_marker.id = object.id;
            position_marker.action = visualization_msgs::Marker::ADD;
            position_marker.type = visualization_msgs::Marker::CUBE;
            position_marker.lifetime = ros::Duration(0.1);

            track_info_marker.header = ros_bridge::UpdateHeader(objects.header);
            track_info_marker.ns = "velocity";
            track_info_marker.id = object.id;
            track_info_marker.action = visualization_msgs::Marker::ADD;
            track_info_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            track_info_marker.lifetime = ros::Duration(0.1);

            double track_velocity = sqrt(object.state.v_x*object.state.v_x + object.state.v_y*object.state.v_y);
            double longitudinal_accel = object.state.a_x * cos(object.state.yaw) + object.state.a_y * sin(object.state.yaw);

            std::ostringstream display_text;
            display_text << "ID (" << object.id << ")";
            display_text << "CLASS (" << object.classification << ")";
            display_text << "\nVel:   " << std::fixed << std::setprecision(3) << std::round(track_velocity * 1000.0) / 1000.0 << "m/s"
                         << "\nAccel: " << std::fixed << std::setprecision(3) << std::round(longitudinal_accel * 1000.0) / 1000.0 << "m/s^2";

            track_info_marker.text = display_text.str();

            // Line width
            position_marker.scale.x = object.dimension.length;
            position_marker.scale.y = object.dimension.width;
            position_marker.scale.z = object.dimension.height;
            
            track_info_marker.scale.z = 1.0;

            // Color space
            position_marker.color.r = 0.5f;
            position_marker.color.g = 0.5f;
            position_marker.color.b = 1.0f;
            position_marker.color.a = 0.7f;

            track_info_marker.color.r = 0.5f;
            track_info_marker.color.g = 0.5f;
            track_info_marker.color.b = 1.0f;
            track_info_marker.color.a = 1.0f;

            position_marker.pose.position.x = object.state.x;
            position_marker.pose.position.y = object.state.y;
            position_marker.pose.position.z = object.dimension.height/2;
            position_marker.pose.orientation = tf::createQuaternionMsgFromYaw(object.state.yaw);

            track_info_marker.pose.position.x = object.state.x;
            track_info_marker.pose.position.y = object.state.y;
            track_info_marker.pose.position.z = 3.0;
            track_info_marker.pose.orientation = tf::createQuaternionMsgFromYaw(object.state.yaw);

            msg.markers.push_back(position_marker);
            msg.markers.push_back(track_info_marker);
        }

        return msg;
    };

    visualization_msgs::MarkerArray UpdateRvizPredictObjects(const interface::PredictObjects& objects,
                                                             const float& max_speed) {
        visualization_msgs::MarkerArray msg;
        
        int id_position_marker = 0;
        int id_reference_marker = 0;
        for (const auto& object : objects.object) {
            if (object.state.size() < 1) {
                continue;
            }

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            // Track Info Marker
            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            visualization_msgs::Marker track_info_marker;
            track_info_marker.header = ros_bridge::UpdateHeader(objects.header);
            track_info_marker.ns = "track_info";
            track_info_marker.id = object.id;
            track_info_marker.action = visualization_msgs::Marker::ADD;
            track_info_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            track_info_marker.lifetime = ros::Duration(0.1);

            // Line width
            track_info_marker.scale.z = 1.0;

            // Color space
            track_info_marker.color.r = 1.0f;
            track_info_marker.color.g = 1.0f;
            track_info_marker.color.b = 1.0f;
            track_info_marker.color.a = 1.0f;

            double track_velocity = sqrt(object.state.front().v_x*object.state.front().v_x + object.state.front().v_y*object.state.front().v_y);
            double local_accel = object.state.front().a_x * cos(object.state.front().yaw) + object.state.front().a_y * sin(object.state.front().yaw);

            std::ostringstream display_text;
            display_text << "ID (" << object.id << ")";
            if (object.dynamic_state == interface::ObjectDynamicState::STATIC) {
                display_text << " STATIC";
            }
            else if (object.dynamic_state == interface::ObjectDynamicState::DYNAMIC) {
                display_text << " DYNAMIC";
            }
            else {
                display_text << " UNKNOWN";
            }
            display_text << "\nVel:   " << std::fixed << std::setprecision(3) << std::round(track_velocity * 1000.0) / 1000.0 << "m/s"
                         << "\nAccel: " << std::fixed << std::setprecision(3) << std::round(local_accel * 1000.0) / 1000.0 << "m/s^2";

            track_info_marker.text = display_text.str();
            track_info_marker.pose.position.x = object.state.front().x;
            track_info_marker.pose.position.y = object.state.front().y;
            track_info_marker.pose.position.z = 3.0;
            track_info_marker.pose.orientation = tf::createQuaternionMsgFromYaw(object.state.front().yaw);
            msg.markers.push_back(track_info_marker);

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            // Position Marker
            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            // visualization_msgs::Marker prediction_marker;
            // prediction_marker.header = ros_bridge::UpdateHeader(objects.header);
            // prediction_marker.ns = "multi_modal";
            // prediction_marker.id = id_position_marker;
            // prediction_marker.action = visualization_msgs::Marker::ADD;
            // prediction_marker.type = visualization_msgs::Marker::CUBE;
            // prediction_marker.lifetime = ros::Duration(0.1);

            // // Line width
            // prediction_marker.scale.x = object.dimension.length;
            // prediction_marker.scale.y = object.dimension.width;
            // prediction_marker.scale.z = object.dimension.height;

            // for (const auto& prediction : object.state_multi) {
            //     int idx_state = 0;
            //     for (const auto& state : prediction.state) {
            //         geometry_msgs::Point point;
            //         point.x = state.x;
            //         point.y = state.y;
            //         point.z = 0.0;
            //         float iter_track_speed = sqrt(state.v_x*state.v_x + state.v_y*state.v_y);
            //         float normalized_color = (std::min(max_speed, iter_track_speed))/max_speed;

            //         prediction_marker.color.r = 0.0;
            //         prediction_marker.color.g = 1.0 - normalized_color;
            //         prediction_marker.color.b = 1.0;
            //         prediction_marker.color.a = 0.1*(float)(object.state.size() - idx_state)/((float)object.state.size()) + 0.1;
            //         if(idx_state == 0){
            //             prediction_marker.color.a = 0.4;
            //         }

            //         prediction_marker.pose.position.x = state.x;
            //         prediction_marker.pose.position.y = state.y;
            //         prediction_marker.pose.position.z = object.dimension.height/2;
            //         prediction_marker.pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw);

            //         prediction_marker.id = id_position_marker++;
            //         msg.markers.push_back(prediction_marker);

            //         idx_state++;
            //     }
            // }

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            // Position Marker
            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            visualization_msgs::Marker position_marker;
            position_marker.header = ros_bridge::UpdateHeader(objects.header);
            position_marker.ns = "position";
            position_marker.id = id_position_marker;
            position_marker.action = visualization_msgs::Marker::ADD;
            position_marker.type = visualization_msgs::Marker::CUBE;
            position_marker.lifetime = ros::Duration(0.1);
            
            // Box size
            position_marker.scale.x = object.dimension.length;
            position_marker.scale.y = object.dimension.width;
            position_marker.scale.z = object.dimension.height;

            int idx_state = 0;
            for (const auto& state : object.state) {
                geometry_msgs::Point point;
                point.x = state.x;
                point.y = state.y;
                point.z = 0.0;
                float iter_track_speed = sqrt(state.v_x*state.v_x + state.v_y*state.v_y);
                float normalized_color = (std::min(max_speed, iter_track_speed))/max_speed;

                position_marker.color.r = 1.0;
                position_marker.color.g = 1.0 - normalized_color;
                position_marker.color.b = 0.0;
                position_marker.color.a = 0.1*(float)(object.state.size() - idx_state)/((float)object.state.size()) + 0.1;
                if(idx_state == 0){
                    position_marker.color.a = 0.4;
                }

                position_marker.pose.position.x = state.x;
                position_marker.pose.position.y = state.y;
                position_marker.pose.position.z = object.dimension.height/2;
                position_marker.pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw);

                position_marker.id = id_position_marker++;
                msg.markers.push_back(position_marker);

                idx_state++;
            }

            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            // Reference Marker
            // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
            for (const auto& reference : object.reference) {
                visualization_msgs::Marker reference_marker;
                reference_marker.header = ros_bridge::UpdateHeader(objects.header);
                reference_marker.ns = "reference";
                reference_marker.id = id_reference_marker;
                reference_marker.action = visualization_msgs::Marker::ADD;
                reference_marker.type = visualization_msgs::Marker::LINE_STRIP;
                reference_marker.lifetime = ros::Duration(0.1);
                reference_marker.scale.x = 1.0;
                reference_marker.color.r = 0.8f;
                reference_marker.color.g = 0.8f;
                reference_marker.color.b = 0.8f;
                reference_marker.color.a = 0.7f;  

                for (const auto& reference_point : reference.point) {
                    geometry_msgs::Point point;
                    point.x = reference_point.x;
                    point.y = reference_point.y;
                    point.z = 0.0;
                    reference_marker.points.push_back(point);
                }

                reference_marker.pose.orientation.x = 0.0;
                reference_marker.pose.orientation.y = 0.0;
                reference_marker.pose.orientation.z = 0.0;
                reference_marker.pose.orientation.w = 0.0;  

                msg.markers.push_back(reference_marker);
                id_reference_marker++;
            }
        }

        return msg;
    }
} // namespace ros_bridge

#endif  // __ROS_BRIDGE_OBJECTS__