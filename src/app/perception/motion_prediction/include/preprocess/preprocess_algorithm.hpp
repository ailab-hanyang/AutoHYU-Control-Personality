/**
 * @file        preprocess_algorithm.hpp
 * @brief       algorithm hpp file for preprocess algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-07-02 created by Yuseung Na
 * 
 */

#ifndef __PREPROCESS_ALGORITHM_HPP__
#define __PREPROCESS_ALGORITHM_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <unordered_map>

// Interface Header
#include "interface_objects.hpp"
#include "interface_lanelet.hpp"
#include "interface_trajectories.hpp"

// Util Function Header
#include "function_vehicle_state.hpp"
#include "function_objects.hpp"
#include "function_lanelet.hpp"

// Library Header
#include "spline.h"

// Parameter Header
#include "motion_prediction_config.hpp"

class Preprocess {
    public:
        explicit Preprocess();
        virtual ~Preprocess();

    public:
        // removing map loader dependency
        interface::PredictObjects RunAlgorithm(const double& current_time,
                                               const interface::TrackObjects& track_objects,
                                               const interface::Trajectory& trajectory,
                                            //    const lanelet::LaneletMapPtr& map_lanelet,
                                            //    const uint32_t& prev_map_time,
                                               const MotionPredictionConfig& cfg);
    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
	    // Functions
        interface::TrackObjects ManageTrackBuffer(const interface::TrackObjects& track_objects,
                                                  const MotionPredictionConfig& cfg);
        interface::TrackObject ProcessObject(const std::vector<interface::Object3DState>& i_state_vec,
                                             const MotionPredictionConfig& cfg);

        void ClearOldTrack(const double& current_time, const MotionPredictionConfig& cfg);

        interface::TrackObjects CompensateDelay(const interface::TrackObjects& processed_objects, 
                                                const double& current_time, 
                                                const MotionPredictionConfig& cfg);
        interface::TrackObject PredObjToDeltaTime(const interface::TrackObject& processed_object,
                                                  const double& delta_time);

        interface::PredictObjects AddReferenceTrajectory(const interface::TrackObjects& compensated_track_objects,
                                                         const lanelet::LaneletMapPtr& map_lanelet,
                                                         const lanelet::routing::RoutingGraphUPtr& routing_graph,
                                                         const MotionPredictionConfig& cfg);

        float GetPointDistance(float& x1, float& y1, float& x2, float& y2);
        float VectorDotProduct2D(const std::pair<float, float> v1, const std::pair<float, float> v2);
        
        
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables        

        // Outputs
        interface::PredictObjects o_processed_objects_;

        // Global Variables
        // ADD GLOBAL VARIABLES HERE
        std::unordered_map<int, std::vector<interface::Object3DState>> track_state_buffer_; 
        lanelet::routing::RoutingGraphUPtr routing_graph_;
        uint32_t prev_map_seq_;
};

#endif // __PREPROCESS_ALGORITHM_HPP__