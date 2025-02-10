/**
 * @file        ros_bridge_lanelet.hpp
 * @brief       ROS bridge for lanelet
 * 
 * @authors     JunheeLee (998jun@gmail.com)          
 * 
 * @date        2024-05-01 created by Junhee Lee
 *              2024-06-20 updated by Yuseung Na: add useful functions
 * 
 */

#ifndef __ROS_BRIDGE_LANELET__
#define __ROS_BRIDGE_LANELET__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Message Header
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <autohyu_msgs/MapBin.h>
#include <autohyu_msgs/LaneletPath.h>
#include <autohyu_msgs/LaneletRoute.h>

// Interface Header
#include "interface_lanelet.hpp"

// Util Function Header
#include "function_lanelet.hpp"

// Bridge Header
#include "ros_bridge_header.hpp"
#include "ros_bridge_geometry_msgs.hpp"

namespace ros_bridge {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    interface::MapBin GetLaneletMapBin(const autohyu_msgs::MapBin& msg) {
        interface::MapBin map_data;
        map_data.header = ros_bridge::GetHeader(msg.header);
        map_data.data = msg.data;

        return map_data;
    }

    lanelet::LaneletMapPtr GetLaneletMap(const autohyu_msgs::MapBin& msg) {
        std::vector<int8_t> data = msg.data;
        return util_function::DecodeLaneletMapBin(data);
    }

    interface::LaneletPath GetLaneletPath(const autohyu_msgs::LaneletPath& msg) {
        interface::LaneletPath lanelet_path;
        lanelet_path.id_ego = msg.ll_id_ego;
        lanelet_path.id_path = msg.ll_id_path;
        lanelet_path.goal_point = ros_bridge::GetPoint(msg.goal_point);

        return lanelet_path;
    }

    interface::LaneletRelation GetLaneletRelation(const autohyu_msgs::LaneletRelation& msg) {
        interface::LaneletRelation lanelet_relation;
        lanelet_relation.id = msg.id;
        lanelet_relation.type = static_cast<interface::LaneletRelationType>(msg.type);

        return lanelet_relation;
    }

    interface::Lanelet GetLanelet(const autohyu_msgs::Lanelet& msg) {
        interface::Lanelet lanelet;
        lanelet.id = msg.id;
        lanelet.is_ego = msg.is_ego;
        lanelet.is_goal = msg.is_goal;
        lanelet.is_shortest_path = msg.is_shortest_path;
        lanelet.left_boundary_line_id = msg.left_boundary_line_id;
        lanelet.right_boundary_line_id = msg.right_boundary_line_id;


        for (const auto& relation : msg.relation) {
            lanelet.relation.push_back(ros_bridge::GetLaneletRelation(relation));
        }

        return lanelet;
    }

    interface::LaneletLinePoint GetLaneletLinePoint(const autohyu_msgs::LaneletLinePoint& msg) {
        interface::LaneletLinePoint lanelet_line_point;
        lanelet_line_point.x = msg.x;
        lanelet_line_point.y = msg.y;
        lanelet_line_point.z = msg.z;

        return lanelet_line_point;
    }

    interface::LaneletLine GetLaneletLine(const autohyu_msgs::LaneletLine& msg) {
        interface::LaneletLine lanelet_line;
        lanelet_line.id = msg.id;

        for (const auto& point : msg.point) {
            lanelet_line.point.push_back(ros_bridge::GetLaneletLinePoint(point));
        }

        return lanelet_line;
    }

    interface::LaneletRoute GetLaneletRoute(const autohyu_msgs::LaneletRoute& msg){
        interface::LaneletRoute lanelet_route;
        lanelet_route.header = ros_bridge::GetHeader(msg.header);

        for (const auto& lanelet : msg.lanelet) {
            lanelet_route.lanelet.push_back(ros_bridge::GetLanelet(lanelet));
        }

        for (const auto& left_boundary : msg.left_boundary) {
            lanelet_route.left_boundary.push_back(ros_bridge::GetLaneletLine(left_boundary));
        }

        for (const auto& right_boundary : msg.right_boundary) {
            lanelet_route.right_boundary.push_back(ros_bridge::GetLaneletLine(right_boundary));
        }

        for (const auto& center_line : msg.center_line) {
            lanelet_route.center_line.push_back(ros_bridge::GetLaneletLine(center_line));
        }

        return lanelet_route;
    }

    // interface::LaneletRoute GetLaneletRoute(const autohyu_msgs::LaneletRoute& msg){
    //     interface::LaneletRoute lanelet_route;
    //     lanelet_route.id_ego = msg.ll_id_ego;
    //     lanelet_route.id_route = msg.ll_id_route;
    //     lanelet_route.goal_point = ros_bridge::GetPoint(msg.goal_point);

    //     return lanelet_route;
    // }
    
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    autohyu_msgs::MapBin UpdateLaneletMapBin(const interface::MapBin& map_data) {
        autohyu_msgs::MapBin msg;
        msg.header = ros_bridge::UpdateHeader(map_data.header);
        msg.data = map_data.data;
        
        return msg;
    }

    autohyu_msgs::LaneletPath UpdateLaneletPath(const interface::LaneletPath& lanelet_path) {
        autohyu_msgs::LaneletPath msg;
        msg.ll_id_ego = lanelet_path.id_ego;
        msg.ll_id_path = lanelet_path.id_path;
        msg.goal_point = ros_bridge::UpdatePoint(lanelet_path.goal_point);

        return msg;
    }

    autohyu_msgs::LaneletRelation UpdateLaneletRelation(const interface::LaneletRelation& lanelet_relation) {
        autohyu_msgs::LaneletRelation msg;
        msg.id = lanelet_relation.id;
        msg.type = static_cast<uint8_t>(lanelet_relation.type);

        return msg;
    }

    autohyu_msgs::Lanelet UpdateLanelet(const interface::Lanelet& lanelet) {
        autohyu_msgs::Lanelet msg;
        msg.id = lanelet.id;
        msg.is_ego = lanelet.is_ego;
        msg.is_goal = lanelet.is_goal;
        msg.is_shortest_path = lanelet.is_shortest_path;
        msg.left_boundary_line_id = lanelet.left_boundary_line_id;
        msg.right_boundary_line_id = lanelet.right_boundary_line_id;

        for (const auto& relation : lanelet.relation) {
            msg.relation.push_back(ros_bridge::UpdateLaneletRelation(relation));
        }

        return msg;
    }

    autohyu_msgs::LaneletLinePoint UpdateLaneletLinePoint(const interface::LaneletLinePoint& lanelet_line_point) {
        autohyu_msgs::LaneletLinePoint msg;
        msg.x = lanelet_line_point.x;
        msg.y = lanelet_line_point.y;
        msg.z = lanelet_line_point.z;

        return msg;
    }

    autohyu_msgs::LaneletLine UpdateLaneletLine(const interface::LaneletLine& lanelet_line) {
        autohyu_msgs::LaneletLine msg;
        msg.id = lanelet_line.id;

        for (const auto& point : lanelet_line.point) {
            msg.point.push_back(ros_bridge::UpdateLaneletLinePoint(point));
        }

        return msg;
    }

    autohyu_msgs::LaneletRoute UpdateLaneletRoute(const interface::LaneletRoute& lanelet_route) {
        autohyu_msgs::LaneletRoute msg;
        msg.header = ros_bridge::UpdateHeader(lanelet_route.header);

        for (const auto& lanelet : lanelet_route.lanelet) {
            msg.lanelet.push_back(ros_bridge::UpdateLanelet(lanelet));
        }

        for (const auto& left_boundary : lanelet_route.left_boundary) {
            msg.left_boundary.push_back(ros_bridge::UpdateLaneletLine(left_boundary));
        }

        for (const auto& right_boundary : lanelet_route.right_boundary) {
            msg.right_boundary.push_back(ros_bridge::UpdateLaneletLine(right_boundary));
        }

        for (const auto& center_line : lanelet_route.center_line) {
            msg.center_line.push_back(ros_bridge::UpdateLaneletLine(center_line));
        }

        return msg;
    }


    // autohyu_msgs::LaneletRoute UpdateLaneletRoute(const interface::LaneletRoute& lanelet_route) {
    //     autohyu_msgs::LaneletRoute msg;
    //     msg.ll_id_ego = lanelet_route.id_ego;
    //     msg.ll_id_route = lanelet_route.id_route;
    //     msg.goal_point = ros_bridge::UpdatePoint(lanelet_route.goal_point);

    //     return msg;
    // }
    ////////////////////////////////
    // rviz update function
    ////////////////////////////////

    // update rviz path, input : lanelet::routing::LaneletPath 
    visualization_msgs::MarkerArray UpdateRvizPath(const lanelet::routing::LaneletPath &path){
        using namespace lanelet;
        visualization_msgs::MarkerArray marker_array;
        for (auto lanelet : path) {
            auto centerline = lanelet.centerline();
            visualization_msgs::Marker marker_center;
            marker_center.header.stamp    = ros::Time::now();
            marker_center.header.frame_id = "world";
            marker_center.ns              = "shortest_path";
            marker_center.id              = (int32_t)lanelet.id();
            marker_center.type            = visualization_msgs::Marker::LINE_STRIP;
            marker_center.action          = visualization_msgs::Marker::ADD;
            marker_center.scale.x         = 1.0;
            marker_center.color.r         = 0.6;
            marker_center.color.g         = 0.98;
            marker_center.color.b         = 0.98;
            marker_center.color.a         = 0.3;
            marker_center.lifetime        = ros::Duration(1.2);
            for (auto i_cl = 0; i_cl < centerline.size(); i_cl++) {
                geometry_msgs::Point point;
                point.x = centerline[i_cl].x();
                point.y = centerline[i_cl].y();
                point.z = 0.2;
                marker_center.points.push_back(point);
            }
            marker_array.markers.push_back(marker_center);
        }
        return marker_array;
    }

    // update rviz route, input : lanelet::LaneletSubmapConstPtr 
    visualization_msgs::MarkerArray UpdateRvizRoute(const lanelet::LaneletSubmapConstPtr& sub_map){
        using namespace lanelet;
        visualization_msgs::MarkerArray marker_array;
        //std::cout << "here 0" << std::endl;
        for (auto lanelet : sub_map->laneletLayer) {
            //std::cout << "here -1" << std::endl;
            auto centerline = lanelet.centerline();

            //std::cout << "here 1" << std::endl;
            visualization_msgs::Marker marker_center;
            marker_center.header.stamp    = ros::Time::now();
            marker_center.header.frame_id = "world";
            marker_center.ns              = "route";
            marker_center.id              = (int32_t)lanelet.id();
            marker_center.type            = visualization_msgs::Marker::LINE_STRIP;
            marker_center.action          = visualization_msgs::Marker::ADD;
            marker_center.scale.x         = 2.0;
            marker_center.color.r         = 1.0;
            marker_center.color.g         = 1.0;
            marker_center.color.b         = 1.0;
            marker_center.color.a         = 0.15;
            marker_center.lifetime        = ros::Duration(1.2);
            for (auto i_cl = 0; i_cl < centerline.size(); i_cl++) {
                geometry_msgs::Point point;
                point.x = centerline[i_cl].x();
                point.y = centerline[i_cl].y();
                point.z = 0.0;
                marker_center.points.push_back(point);
            }      
            marker_array.markers.push_back(marker_center);
        }
        return marker_array;   
    }

    visualization_msgs::MarkerArray UpdateRvizRouteSub(const interface::LaneletRoute &sub_route){
        visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < sub_route.center_line.size(); i++) {
            auto centerline = sub_route.center_line;
            visualization_msgs::Marker marker_center;
            marker_center.header.stamp    = ros::Time::now();
            marker_center.header.frame_id = "world";
            marker_center.ns              = "route_sub";
            marker_center.id              = i;
            marker_center.type            = visualization_msgs::Marker::LINE_STRIP;
            marker_center.action          = visualization_msgs::Marker::ADD;
            marker_center.scale.x         = 1.0;
            marker_center.color.r         = 1.0;
            marker_center.color.g         = 1.0;
            marker_center.color.b         = 0.0;
            marker_center.color.a         = 1.0;
            marker_center.lifetime        = ros::Duration(1.2);
            for (auto i_cl = 0; i_cl < centerline[i].point.size(); i_cl++) {
                geometry_msgs::Point point;
                point.x = centerline[i].point[i_cl].x;
                point.y = centerline[i].point[i_cl].y;
                point.z = 0.2;
                marker_center.points.push_back(point);
            }
            marker_array.markers.push_back(marker_center);
        }
        return marker_array;
    }

    // update rviz route, input : center x,y point of lanelet and lanelet_route_id
    visualization_msgs::MarkerArray UpdateRvizRouteNum(const std::vector<std::pair<float, float>> route_center_point,
                                                    std::vector<int64_t> ll_id_route){
        visualization_msgs::MarkerArray marker_array;
        for(int i=0; i< route_center_point.size(); i++){
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route id";
            marker.id = i;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.9;
            marker.lifetime = ros::Duration(1.2);
            // geometry_msgs::Point point;
            // point.x = waypoint.first[i];
            // point.y = waypoint.second[i];
            // point.z = 1;
            //marker.points.push_back(point);
            marker.pose.position.x = route_center_point[i].first;
            marker.pose.position.y = route_center_point[i].second;
            marker.pose.position.z = 2.0;
            marker.pose.orientation.w = 1.0;\
            std::stringstream ss;
            ss << i <<" th route: " << ll_id_route[i]; 
            marker.text = ss.str();

            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }
    visualization_msgs::MarkerArray UpdateRvizRoutePoints(const interface::LaneletRoute route_info){
        visualization_msgs::MarkerArray marker_array;

        for(int i=0; i< route_info.left_boundary.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route boundary";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.lifetime = ros::Duration(1.2);        
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            for (int j = 0; j < route_info.left_boundary[i].point.size(); j++){
                geometry_msgs::Point point;
                
                point.x = route_info.left_boundary[i].point[j].x;
                point.y = route_info.left_boundary[i].point[j].y;
                point.z = 0.0;
                marker.points.push_back(point);         
            }        
            marker_array.markers.push_back(marker);    
        }

        for(int i=0; i< route_info.right_boundary.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route boundary";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.lifetime = ros::Duration(1.2);     
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            for (int j = 0; j < route_info.right_boundary[i].point.size(); j++){
                geometry_msgs::Point point;
                
                point.x = route_info.right_boundary[i].point[j].x;
                point.y = route_info.right_boundary[i].point[j].y;
                point.z = 0.0;
                marker.points.push_back(point);         
            }        
            marker_array.markers.push_back(marker);    
        }

        for(int i=0; i< route_info.center_line.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route center";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.lifetime = ros::Duration(1.2);   
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            for (int j = 0; j < route_info.center_line[i].point.size(); j++){
                geometry_msgs::Point point;
                
                point.x = route_info.center_line[i].point[j].x;
                point.y = route_info.center_line[i].point[j].y;
                point.z = 0.0;
                marker.points.push_back(point);         
            }        
            marker_array.markers.push_back(marker);    
        }
        
        return marker_array;
    }

    visualization_msgs::MarkerArray UpdateRvizLocalRoutePoints(const interface::LaneletRoute route_info){
        visualization_msgs::MarkerArray marker_array;

        for(int i=0; i< route_info.left_boundary.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route left";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.lifetime = ros::Duration(1.2);        
            marker.scale.x = 0.6;
            marker.scale.y = 0.6;
            marker.scale.z = 0.6;
            marker.color.r = 0.5;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            for (int j = 0; j < route_info.left_boundary[i].point.size(); j++){
                geometry_msgs::Point point;
                
                point.x = route_info.left_boundary[i].point[j].x;
                point.y = route_info.left_boundary[i].point[j].y;
                point.z = 0.0;
                marker.points.push_back(point);         
            }        
            marker_array.markers.push_back(marker);    
        }

        for(int i=0; i< route_info.right_boundary.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route right";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.lifetime = ros::Duration(1.2);           
            marker.scale.x = 0.6;
            marker.scale.y = 0.6;
            marker.scale.z = 0.6;
            marker.color.r = 0.5;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            for (int j = 0; j < route_info.right_boundary[i].point.size(); j++){
                geometry_msgs::Point point;
                
                point.x = route_info.right_boundary[i].point[j].x;
                point.y = route_info.right_boundary[i].point[j].y;
                point.z = 0.0;
                marker.points.push_back(point);         
            }        
            marker_array.markers.push_back(marker);    
        }

        for(int i=0; i< route_info.center_line.size(); i++){
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route center";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.lifetime = ros::Duration(1.2);          
            marker.scale.x = 0.6;
            marker.scale.y = 0.6;
            marker.scale.z = 0.6;
            marker.color.r = 1.0;
            marker.color.g = 0.647;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            for (int j = 0; j < route_info.center_line[i].point.size(); j++){
                geometry_msgs::Point point;
                
                point.x = route_info.center_line[i].point[j].x;
                point.y = route_info.center_line[i].point[j].y;
                point.z = 0.0;
                marker.points.push_back(point);         
            }        
            marker_array.markers.push_back(marker);    
        }
        
        return marker_array;
    }

    // not complete
    visualization_msgs::MarkerArray UpdateRvizWayPoint(const std::pair<std::vector<double>,std::vector<double>> waypoint){
        visualization_msgs::MarkerArray marker_array;
        for(int i=0; i< waypoint.first.size(); i++){
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "path way point";
            marker.id = i;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.r = 0.9;
            marker.color.g = 0.1;
            marker.color.b = 0.1;
            marker.color.a = 0.9;
            marker.lifetime = ros::Duration(1.2);
            // geometry_msgs::Point point;
            // point.x = waypoint.first[i];
            // point.y = waypoint.second[i];
            // point.z = 1;
            //marker.points.push_back(point);
            marker.pose.position.x = waypoint.first[i];
            marker.pose.position.y = waypoint.second[i];
            marker.pose.position.z = 2.0;
            marker.pose.orientation.w = 1.0;        
        }
        return marker_array;
    }




visualization_msgs::MarkerArray UpdateRvizRawRoutePoints(const interface::LaneletRoute route_info) {
    visualization_msgs::MarkerArray marker_array;

    // Boundary line을 위한 SPHERE 시각화
    for (int i = 0; i < route_info.left_boundary.size(); i++) {
        for (int j = 0; j < route_info.left_boundary[i].point.size(); j++) {
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route_boundary";
            marker.id = i * 1000 + j;  // 각 점마다 고유한 ID를 부여
            // marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;  // SPHERE 형태로 설정
            marker.lifetime = ros::Duration(1.2);        
            marker.scale.x = 0.5;  // 구의 크기를 적절히 설정
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            geometry_msgs::Point point;
            point.x = route_info.left_boundary[i].point[j].x;
            point.y = route_info.left_boundary[i].point[j].y;
            point.z = 1.0;
            marker.pose.position = point;  // 위치를 지정

            marker_array.markers.push_back(marker);    
        }
    }

    for (int i = 0; i < route_info.right_boundary.size(); i++) {
        for (int j = 0; j < route_info.right_boundary[i].point.size(); j++) {
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route_boundary";
            marker.id = i * 1000 + j;  // 각 점마다 고유한 ID를 부여
            // marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;  // SPHERE 형태로 설정
            marker.lifetime = ros::Duration(1.2);        
            marker.scale.x = 0.5;  // 구의 크기를 적절히 설정
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            geometry_msgs::Point point;
            point.x = route_info.right_boundary[i].point[j].x;
            point.y = route_info.right_boundary[i].point[j].y;
            point.z = 1.0;
            marker.pose.position = point;  // 위치를 지정

            marker_array.markers.push_back(marker);    
        }
    }

    // Center line을 위한 SPHERE 시각화
    for (int i = 0; i < route_info.center_line.size(); i++) {
        for (int j = 0; j < route_info.center_line[i].point.size(); j++) {
            visualization_msgs::Marker marker;

            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "route_center";
            marker.id = i * 1000 + j;  // 각 점마다 고유한 ID를 부여
            // marker.action = visualization_msgs::Marker::ADD;
            marker.type = visualization_msgs::Marker::SPHERE;  // SPHERE 형태로 설정
            marker.lifetime = ros::Duration(1.2);        
            marker.scale.x = 0.5;  // 구의 크기를 적절히 설정
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            geometry_msgs::Point point;
            point.x = route_info.center_line[i].point[j].x;
            point.y = route_info.center_line[i].point[j].y;
            point.z = 1.0;
            marker.pose.position = point;  // 위치를 지정

            marker_array.markers.push_back(marker);    
        }
    }
    
    return marker_array;
}


                                                

} // namespace ros_bridge

#endif  // __ROS_BRIDGE_LANELET__
