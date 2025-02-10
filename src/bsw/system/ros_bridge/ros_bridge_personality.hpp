/**
 * @file        ros_bridge_personality.hpp
 * @brief       ros bridge for personality bayesopt
 * 
 * @authors     Seounghoon Park(sunghoon8585@gmail.com)          
 * 
 * @date        2024-11-19 created by Seounghoon Park
 * 
 */

#ifndef __ROS_BRIDGE_PERSONALITY_HPP__
#define __ROS_BRIDGE_PERSONALITY_HPP__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <personality_msgs/PersonalityTimeWindow.h>

// Interface Header
#include "interface_personality.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    interface::Stats GetStats(const personality_msgs::Stats& msg) {
        interface::Stats stats;
        
        stats.avg = msg.avg;
        stats.std_dev = msg.std_dev;
        stats.min = msg.min;
        stats.max = msg.max;

        return stats;
    }

    interface::PersonalityScene GetScene(const personality_msgs::PersonalityScene& msg) {
        interface::PersonalityScene scene;
        
        scene.scene_type = msg.scene_type;

        for (int i = 0; i < msg.scene_points.size(); i++) {
            interface::PersonalityScenePoint point;
            point.x = msg.scene_points[i].x;
            point.y = msg.scene_points[i].y;
            point.yaw = msg.scene_points[i].yaw;
            point.vel_mps = msg.scene_points[i].vel_mps;
            point.delta = msg.scene_points[i].delta;
            point.ax = msg.scene_points[i].ax;
            point.ay = msg.scene_points[i].ay;
            point.jx = msg.scene_points[i].jx;
            point.jy = msg.scene_points[i].jy;
            scene.scene_points.push_back(point);
        }

        return scene;
    }

    interface::PersonalityTimeWindow GetPersonalityTimeWindow(
        const personality_msgs::PersonalityTimeWindow& msg) {
        interface::PersonalityTimeWindow window;
        
        window.ax = GetStats(msg.ax);
        window.ax_plus = GetStats(msg.ax_plus);
        window.ax_minus = GetStats(msg.ax_minus);
        window.ay = GetStats(msg.ay);

        window.jx = GetStats(msg.jx);
        window.jx_plus = GetStats(msg.jx_plus);
        window.jx_minus = GetStats(msg.jx_minus);
        window.jy = GetStats(msg.jy);

        interface::PersonalityScene scene;

        for (int i = 0; i < msg.accel_scenes.size(); i++) {
            scene = GetScene(msg.accel_scenes[i]);
            window.accel_scenes.push_back(scene);
        }

        for (int i = 0; i < msg.decel_scenes.size(); i++) {
            scene = GetScene(msg.decel_scenes[i]);
            window.decel_scenes.push_back(scene);
        }

        for (int i = 0; i < msg.corner_scenes.size(); i++) {
            scene = GetScene(msg.corner_scenes[i]);
            window.corner_scenes.push_back(scene);
        }

        return window;
    }
 
    personality_msgs::Stats UpdateStats(const interface::Stats& stats) {
        personality_msgs::Stats msg;
        
        msg.avg = stats.avg;
        msg.std_dev = stats.std_dev;
        msg.min = stats.min;
        msg.max = stats.max;

        return msg;
    }

    personality_msgs::PersonalityScene UpdateScene(
        const interface::PersonalityScene& scene) {
        
        personality_msgs::PersonalityScene msg;
        
        msg.scene_type = scene.scene_type;

        for (int i = 0; i < scene.scene_points.size(); i++) {
            personality_msgs::PersonalityScenePoint point;
            point.x = scene.scene_points[i].x;
            point.y = scene.scene_points[i].y;
            point.yaw = scene.scene_points[i].yaw;
            point.vel_mps = scene.scene_points[i].vel_mps;
            point.delta = scene.scene_points[i].delta;
            point.ax = scene.scene_points[i].ax;
            point.ay = scene.scene_points[i].ay;
            point.jx = scene.scene_points[i].jx;
            point.jy = scene.scene_points[i].jy;
            msg.scene_points.push_back(point);
        }

        return msg;
    }


    personality_msgs::PersonalityTimeWindow UpdateTimeWindow(
        const interface::PersonalityTimeWindow& window) {
        
        personality_msgs::PersonalityTimeWindow msg;
        
        msg.ax = UpdateStats(window.ax);
        msg.ax_plus = UpdateStats(window.ax_plus);
        msg.ax_minus = UpdateStats(window.ax_minus);
        msg.ay = UpdateStats(window.ay);

        msg.jx = UpdateStats(window.jx);
        msg.jx_plus = UpdateStats(window.jx_plus);
        msg.jx_minus = UpdateStats(window.jx_minus);
        msg.jy = UpdateStats(window.jy);

        personality_msgs::PersonalityScene scene;   

        for (int i = 0; i < window.accel_scenes.size(); i++) {
            scene = UpdateScene(window.accel_scenes[i]);
            msg.accel_scenes.push_back(scene);
        }
        for (int i = 0; i < window.decel_scenes.size(); i++) {
            scene = UpdateScene(window.decel_scenes[i]);
            msg.decel_scenes.push_back(scene);
        }
        for (int i = 0; i < window.corner_scenes.size(); i++) {
            scene = UpdateScene(window.corner_scenes[i]);
            msg.corner_scenes.push_back(scene);
        }

        return msg;
    }

} // namespace ros_bridge

#endif  // __ROS_BRIDGE_CONTROL__