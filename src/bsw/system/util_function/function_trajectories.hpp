/**
 * @file        function_trajectories.hpp
 * @brief       util functions for trajectories
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-11 updated by Yuseung Na: Refactoring 
 */

#ifndef __FUNCTION_TRAJECTORIES_HPP__
#define __FUNCTION_TRAJECTORIES_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// Utility header
#include <spline.h>

// Interface Header
#include "interface_constants.hpp"
#include "interface_trajectories.hpp"
#include "function_print.hpp"

using namespace interface;

namespace util_function {
    inline bool GenerateRoadMap(const std::vector<Point> points,
                                tk::Map& road_map) {
        if ((uint16_t)points.size() < 4) {            
            util_function::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)points.size(); i++) {
                double x = points[i].x;
                double y = points[i].y;
                double dx = i == 0 ? 0.0 : x - points[i - 1].x;
                double dy = i == 0 ? 0.0 : y - points[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }
            
            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }

    inline bool GenerateSparseRoadMap(const std::vector<Point> points,
                                      tk::Map& road_map,
                                      double distance_threshold) {
        if ((uint16_t)points.size() < 4) {            
            util_function::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)points.size(); i++) {
                double x = points[i].x;
                double y = points[i].y;
                double dx = i == 0 ? 0.0 : x - points[i - 1].x;
                double dy = i == 0 ? 0.0 : y - points[i - 1].y;
                double ds = sqrt(dx * dx + dy * dy);
                if(ds > distance_threshold){
                    sum_s += ds;
                    sx.push_back(x);
                    sy.push_back(y);
                    ss.push_back(sum_s);
                    sdx.push_back(dx);
                    sdy.push_back(dy);
                }else{
                    continue;
                }
            }

            if((uint16_t)ss.size() < 4){
                // util_function::DebugPrintError(
                //     "Cannot generate road map to spline!"
                // );
                return false;
            }
            
            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }

    inline bool GenerateSparseRoadMap(const Trajectory& trajectory,
                                      tk::Map& road_map,
                                      double distance_threshold) {
        if ((uint16_t)trajectory.point.size() < 4) {            
            util_function::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
                double x = trajectory.point[i].x;
                double y = trajectory.point[i].y;
                double dx = i == 0 ? 0.0 : x - trajectory.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - trajectory.point[i - 1].y;
                double ds = sqrt(dx * dx + dy * dy);
                if(ds > distance_threshold){
                    sum_s += ds;
                    sx.push_back(x);
                    sy.push_back(y);
                    ss.push_back(sum_s);
                    sdx.push_back(dx);
                    sdy.push_back(dy);
                }else{
                    continue;
                }
            }

            if((uint16_t)ss.size() < 4){
                // util_function::DebugPrintError(
                //     "Cannot generate road map to spline!"
                // );
                return false;
            }
            
            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }

    inline bool GenerateSparseRoadMap(const BehaviorTrajectory& trajectory,
                                      tk::Map& road_map,
                                      double distance_threshold) {
        if ((uint16_t)trajectory.point.size() < 4) {            
            util_function::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
                double x = trajectory.point[i].x;
                double y = trajectory.point[i].y;
                double dx = i == 0 ? 0.0 : x - trajectory.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - trajectory.point[i - 1].y;
                double ds = sqrt(dx * dx + dy * dy);
                if(ds > distance_threshold){
                    sum_s += ds;
                    sx.push_back(x);
                    sy.push_back(y);
                    ss.push_back(sum_s);
                    sdx.push_back(dx);
                    sdy.push_back(dy);
                }else{
                    continue;
                }
            }

            if((uint16_t)ss.size() < 4){
                // util_function::DebugPrintError(
                //     "Cannot generate road map to spline!"
                // );
                return false;
            }
            
            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }
    
    inline bool GenerateRoadMap(const Trajectory& trajectory,
                                tk::Map& road_map) {
        if ((uint16_t)trajectory.point.size() < 4) {            
            util_function::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
                double x = trajectory.point[i].x;
                double y = trajectory.point[i].y;
                double dx = i == 0 ? 0.0 : x - trajectory.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - trajectory.point[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }
            
            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }
    inline bool GenerateRoadMap(const BehaviorTrajectory& trajectory,
                                tk::Map& road_map) {
        if ((uint16_t)trajectory.point.size() < 4) {            
            util_function::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)trajectory.point.size(); i++) {
                double x = trajectory.point[i].x;
                double y = trajectory.point[i].y;
                double dx = i == 0 ? 0.0 : x - trajectory.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - trajectory.point[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }
            
            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }

    inline bool GenerateSearchSpaceMap(const SearchSpace& search_space,
                                       tk::Map& road_map) {
        if ((uint16_t)search_space.point.size() < 4) {
            util_function::DebugPrintError(
                "Cannot generate search space to road map!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)search_space.point.size(); i++) {
                double x = search_space.point[i].x;
                double y = search_space.point[i].y;
                double dx = i == 0 ? 0.0 : x - search_space.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - search_space.point[i - 1].y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist < 0.01) {
                    continue;
                }
                sum_s += dist;
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }

            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }

    inline bool GenerateSearchSpaceBoundaryMap(const std::vector<SearchSpaceBoundaryPoint>& search_space_boundary,
                                               tk::Map& road_map) {
        if ((uint16_t)search_space_boundary.size() < 3) {
            util_function::DebugPrintError(
                "Cannot generate search space boundary to road map!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)search_space_boundary.size(); i++) {
                double x = search_space_boundary[i].x;
                double y = search_space_boundary[i].y;
                double dx = i == 0 ? 0.0 : x - search_space_boundary[i - 1].x;
                double dy = i == 0 ? 0.0 : y - search_space_boundary[i - 1].y;
                double dist = sqrt(dx * dx + dy * dy);
                if (dist < 0.01) {
                    continue;
                }
                sum_s += dist;
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }

            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }

    inline bool GenerateBoundaryMap(const TrajectoryBoundary& trajectory_boundary,
                                    tk::Map& road_map) {
        if ((uint16_t)trajectory_boundary.point.size() < 4) {
            util_function::DebugPrintError(
                "Cannot generate road map to spline!"
            );
            return false;
        } else {
            std::vector<double> sx, sy, ss, sdx, sdy;
            double sum_s = 0.0;
            for (uint16_t i = 0; i < (uint16_t)trajectory_boundary.point.size(); i++) {
                double x = trajectory_boundary.point[i].x;
                double y = trajectory_boundary.point[i].y;
                double dx = i == 0 ? 0.0 : x - trajectory_boundary.point[i - 1].x;
                double dy = i == 0 ? 0.0 : y - trajectory_boundary.point[i - 1].y;
                sum_s += sqrt(dx * dx + dy * dy);
                sx.push_back(x);
                sy.push_back(y);
                ss.push_back(sum_s);
                sdx.push_back(dx);
                sdy.push_back(dy);
            }

            road_map = tk::Map(sx, sy, ss, sdx, sdy);
            return true;
        }
    }    

    inline bool GenerateBoundarySpline(const Trajectory& reference,
                                       tk::spline& left_boundary, 
                                       tk::spline& right_boundary) {
        if (reference.left_boundary.point.size() < 4 || reference.right_boundary.point.size() < 4) {            
            util_function::DebugPrintValueError(
                "Boundary points are not enough!",
                (uint16_t)reference.left_boundary.point.size()
            );            
            return false;
        }
        else {
            std::vector<double> left_s, left_n, right_s, right_n;
            for (auto lp : reference.left_boundary.point) {
                left_s.push_back(lp.s);
                left_n.push_back(lp.n);
            }
            for (auto rp : reference.right_boundary.point) {
                right_s.push_back(rp.s);
                right_n.push_back(rp.n);
            }

            left_boundary.set_points(left_s, left_n, false);
            right_boundary.set_points(right_s, right_n, false);

            return true;
        }
    }
    inline bool GenerateBoundarySpline(const BehaviorTrajectory& reference, 
                                       tk::Map& road_map,
                                       tk::spline& left_boundary, 
                                       tk::spline& right_boundary) {
        if (reference.left_boundary.size() < 4 || reference.right_boundary.size() < 4) {            
            util_function::DebugPrintValueError(
                "Boundary points are not enough!",
                (uint16_t)reference.left_boundary.size()
            );            
            return false;
        }
        else {
            std::vector<double> left_s, left_n, right_s, right_n;
            for (auto lp : reference.left_boundary) {
                std::vector<double> sn = road_map.ToFrenet(lp.x, lp.y);
                left_s.push_back(sn.at(0));
                left_n.push_back(sn.at(1));
            }
            for (auto rp : reference.right_boundary) {
                std::vector<double> sn = road_map.ToFrenet(rp.x, rp.y);
                right_s.push_back(sn.at(0));
                right_n.push_back(sn.at(1));
            }

            left_boundary.set_points(left_s, left_n, false);
            right_boundary.set_points(right_s, right_n, false);

            return true;
        }
    }

    inline bool GenerateBoundarySpline(const Trajectory& reference, 
                                       tk::Map& road_map,
                                       tk::spline& left_boundary, 
                                       tk::spline& right_boundary) {
        if (reference.left_boundary.point.size() < 4 || reference.right_boundary.point.size() < 4) {            
            util_function::DebugPrintValueError(
                "Boundary points are not enough!",
                (uint16_t)reference.left_boundary.point.size()
            );            
            return false;
        }
        else {
            std::vector<double> left_s, left_n, right_s, right_n;
            for (auto lp : reference.left_boundary.point) {
                std::vector<double> sn = road_map.ToFrenet(lp.x, lp.y);
                left_s.push_back(sn.at(0));
                left_n.push_back(sn.at(1));
            }
            for (auto rp : reference.right_boundary.point) {
                std::vector<double> sn = road_map.ToFrenet(rp.x, rp.y);
                right_s.push_back(sn.at(0));
                right_n.push_back(sn.at(1));
            }

            left_boundary.set_points(left_s, left_n, false);
            right_boundary.set_points(right_s, right_n, false);

            return true;
        }
    }

    inline Trajectory ConvertBehaviorToTrajectory(const BehaviorTrajectory& behavior_trajectory) {
        Trajectory trajectory;
        trajectory.header = behavior_trajectory.header;
        trajectory.id = behavior_trajectory.id;
        trajectory.left_boundary.point.clear();
        trajectory.right_boundary.point.clear();
        trajectory.point.clear();

        // Convert behavior trajectory to trajectory
        for (size_t i = 0; i < behavior_trajectory.point.size(); i++) {
            BehaviorTrajectoryPoint point = behavior_trajectory.point.at(i);
            TrajectoryPoint trajectory_point;

            trajectory_point.x = point.x;
            trajectory_point.y = point.y;
            trajectory_point.z = point.z;
            trajectory_point.yaw = point.yaw;
            trajectory_point.curvature = point.curvature;
            trajectory_point.speed = point.ref_speed;
            trajectory_point.acceleration = 0.0;

            // TODO: Direction
            trajectory_point.direction = TrajectoryDirection::FORWARD;

            double dx = i == 0 ? 0.0 : trajectory_point.x - behavior_trajectory.point.at(i-1).x;
            double dy = i == 0 ? 0.0 : trajectory_point.y - behavior_trajectory.point.at(i-1).y;
            double dist = sqrt(dx * dx + dy * dy);
            
            if (i == 0) {
                trajectory_point.distance = 0.0;
                trajectory_point.time = 0.0;
            } else {
                trajectory_point.distance = trajectory.point.back().distance + dist;

                double speed = (trajectory_point.speed + behavior_trajectory.point.at(i-1).speed) / 2.0;

                if (speed >= 0.1) {
                    trajectory_point.time = trajectory.point.back().time + dist / speed;
                } else {
                    trajectory_point.time = trajectory.point.back().time + 0.1;
                }
            }

            TrajectoryBoundaryPoint lb_point;
            lb_point.x = point.lb_point.x;
            lb_point.y = point.lb_point.y;
            lb_point.z = point.lb_point.z;

            TrajectoryBoundaryPoint rb_point;
            rb_point.x = point.rb_point.x;
            rb_point.y = point.rb_point.y;
            rb_point.z = point.rb_point.z;
            
            trajectory.point.push_back(trajectory_point);
            trajectory.left_boundary.point.push_back(lb_point);
            trajectory.right_boundary.point.push_back(rb_point);
        }

        return trajectory;
    }

    inline Trajectory RestrictLateralAccel(const Trajectory& trajectory,
                                           const float& max_lateral_accel,
                                           const float& max_longitudinal_accel,
                                           const float& dt) {
        Trajectory restricted_trajectory = trajectory;  

        float max_restricted_speed = std::numeric_limits<float>::max();

        for(uint32_t i = 1; i < restricted_trajectory.point.size(); i++) {
            // Apply speed limit
            float restricted_speed = sqrt(fabs(max_lateral_accel / restricted_trajectory.point[i].curvature));

            if ((restricted_speed - restricted_trajectory.point[i-1].speed)/dt <= -max_longitudinal_accel) {
                restricted_speed = restricted_trajectory.point[i-1].speed - max_longitudinal_accel * dt;
            }
            restricted_trajectory.point[i].speed = std::min(restricted_trajectory.point[i].speed, restricted_speed);
        }

        // Forward smoothing
        for (uint32_t i = 0; i < restricted_trajectory.point.size(); i++) {
            // Calculate acceleration
            double v = restricted_trajectory.point.at(i).speed;
            double v_old = i == 0 ? v : restricted_trajectory.point.at(i-1).speed;
            double a = (pow(v, 2) - pow(v_old, 2)) / (2);

            // Smoothen the speed not to exceed the maximum acceleration
            if (a > max_longitudinal_accel) {
                restricted_trajectory.point.at(i).speed = sqrt(pow(v_old, 2) + 2 * max_longitudinal_accel);
            }
        }

        // Backward smoothing
        for (uint32_t i = restricted_trajectory.point.size() - 1; i > 0; i--) {
            // Calculate acceleration
            double v = restricted_trajectory.point.at(i).speed;
            double v_old = restricted_trajectory.point.at(i-1).speed;
            double a = (pow(v, 2) - pow(v_old, 2)) / (2);

            // Backward smoothing according to minimum acceleration
            if (a < -max_longitudinal_accel) {  
                restricted_trajectory.point.at(i-1).speed = sqrt(pow(v, 2) - 2 * (-max_longitudinal_accel));
            }
        }

        // Recalculate time
        for(uint32_t i = 1; i < restricted_trajectory.point.size(); i++){
            TrajectoryPoint& current_point = restricted_trajectory.point[i];
            const TrajectoryPoint& previous_point = restricted_trajectory.point[i-1];

            // time = distance / speed
            double distance = current_point.distance - previous_point.distance;
            double speed = (current_point.speed + previous_point.speed) / 2.0;

            if (speed >= 0.1) {
                current_point.time = previous_point.time + (distance / speed);
            }
            else {
                current_point.time = previous_point.time + 0.1;
            }
        }

        return restricted_trajectory;
    }
} // namespace util_function

#endif  // __FUNCTION_TRAJECTORIES_HPP__