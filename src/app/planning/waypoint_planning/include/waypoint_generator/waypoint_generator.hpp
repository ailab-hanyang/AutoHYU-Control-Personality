#ifndef __WAYPOINT_GENERATOR_HPP__
#define __WAYPOINT_GENERATOR_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

// ROS header
#include <ros/ros.h>

// Utility header

#include "interface_trajectories.hpp"
#include "interface_vehicle_state.hpp"
#include "interface_constants.hpp"
#include <ini_parser.h>
#include <spline.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// Message header
#include <visualization_msgs/MarkerArray.h>

// Parameter Header
#include <waypoint_planning_config.hpp>

// Namespace
using namespace std;
using namespace interface;

class WaypointGenerator {
    public:
        // Constructor
        explicit WaypointGenerator();
        // Destructor
        virtual ~WaypointGenerator();

    public:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        bool Init(const WaypointPlanningParams& params);
        void ProcessINI();

        pair<Trajectory,Trajectory> GenerateWaypoint(VehicleState* vehicle_state, WaypointPlanningParams* params);
        lanelet::LaneletMapPtr GetLaneletMap(){return ll_lanelet_map_;};


    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        bool CheckLaneletMapUpdate();
        bool CheckSpeedUpdate();
        void UpdateLaneletMap();   
        void UpdateRaceTrajectory(); 
        lanelet::Lanelet FindClosestLanelet(lanelet::LaneletMap &map, const lanelet::BasicPoint2d &ego_point);
        
        Trajectory CalculateROITrajectory(const Trajectory& race_trajectory);
        vector<TrajectoryBoundaryPoint> CropTrajectoryBoundaryROI(const VehicleState& vehicle_state,
                                                            tk::Map& road_map,
                                                            const vector<TrajectoryBoundaryPoint> &tb, 
                                                            uint16_t &prev_closest_idx);
        void CalculateTime(Trajectory& race_trajectory);

        inline tk::Map GenerateRoadMap(const Trajectory& trajectory) {
            tk::Map road_map;
            if ((uint16_t)trajectory.point.size() < 4) {            
                std::cout << "Cannot generate road map to spline!" << std::endl;

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
            }

            return road_map;
        };    

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        uint32_t prev_idx_ = 0;
        uint16_t v_lb_last_idx_ = 0;
        uint16_t v_rb_last_idx_ = 0;
        WaypointPlanningParams prev_params_;
        vector<lanelet::ConstLanelet> v_reference_lanelet_;
        // Inputs       
        std::string i_map_file_path_ = "";
        double i_max_speed_;
        VehicleState i_vehicle_state_;
        uint8_t i_scene_state_;

        // Outputs

        // Environments
        IniParser util_ini_parser_;
        
        // Configuration paramters   
        WaypointPlanningParams params_;

        lanelet::GPSPoint ll_map_origin_;
        lanelet::LaneletMapPtr ll_lanelet_map_;
        Trajectory global_race_trajectory_;
        pcl::KdTreeFLANN<pcl::PointXY> reference_waypoints_kdtree_;
        pcl::KdTreeFLANN<pcl::PointXY> left_boundary_kdtree_;
        pcl::KdTreeFLANN<pcl::PointXY> right_boundary_kdtree_;

};

#endif  // TRAJECTORY_GENERATOR_HPP_
