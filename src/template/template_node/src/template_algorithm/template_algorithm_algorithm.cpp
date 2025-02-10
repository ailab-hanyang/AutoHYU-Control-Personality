/**
 * @file        template_algorithm_algorithm.cpp
 * @brief       template algorithm cpp file for template node algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#include "template_algorithm/template_algorithm_algorithm.hpp"

TemplateAlgorithm::TemplateAlgorithm() {
    // Initialize
}

TemplateAlgorithm::~TemplateAlgorithm() {
    // Terminate
}

double TemplateAlgorithm::DummyAlgorithm1(const interface::VehicleState& vehicle_state,
                                          const interface::DetectObjects3D& detect_objects,
                                          const TemplateNodeConfig& cfg) {
    // Dummy Algorithm 1
    double dummy_out1 = 0.0;

    // Return
    return dummy_out1;
}

int TemplateAlgorithm::DummyAlgorithm2(const interface::VehicleState& vehicle_state,
                                       const interface::TrackObjects& track_objects,
                                       const TemplateNodeConfig& cfg) {
    // Dummy Algorithm 2
    int dummy_out2 = 0;

    // Return
    return dummy_out2;
}

interface::Trajectory TemplateAlgorithm::DummyAlgorithm3(const interface::VehicleState& vehicle_state,
                                                         const interface::PredictObjects& predict_objects,
                                                         const TemplateNodeConfig& cfg) {
    // Dummy Algorithm 3
    interface::Trajectory dummy_out3;

    // Return
    return dummy_out3;
}

interface::Trajectory TemplateAlgorithm::DummyAlgorithm4(const interface::VehicleState& vehicle_state,
                                                         const interface::Trajectory& trajectory,
                                                         const TemplateNodeConfig& cfg) {
    // Dummy Algorithm 4
    interface::Trajectory dummy_out4;

    // Return
    return dummy_out4;
}

interface::Trajectory TemplateAlgorithm::RunAlgorithm(const interface::VehicleState& vehicle_state,
                                                       const interface::DetectObjects3D& detect_objects,
                                                       const interface::TrackObjects& track_objects,
                                                       const interface::PredictObjects& predict_objects,
                                                       const interface::Trajectory& trajectory,
                                                       const TemplateNodeConfig& cfg) {
    // Run Algorithm
    double dummy_out1 = DummyAlgorithm1(vehicle_state, detect_objects, cfg);
    int dummy_out2 = DummyAlgorithm2(vehicle_state, track_objects, cfg);
    interface::Trajectory dummy_out3 = DummyAlgorithm3(vehicle_state, predict_objects, cfg);
    interface::Trajectory dummy_out4 = DummyAlgorithm4(vehicle_state, trajectory, cfg);

    // Check the output is valid
    if (dummy_out4.header.stamp != 0) {
        o_trajectory_ = dummy_out4;
    }

    // Return
    return o_trajectory_;
}