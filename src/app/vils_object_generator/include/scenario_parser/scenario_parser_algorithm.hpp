#ifndef __VEHICLE_INFO_GENERATOR__
#define __VEHICLE_INFO_GENERATOR__

// STD Header
#include <spline.h>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <ros/ros.h>

// Interface Header
#include "function_trajectories.hpp"
#include "interface_constants.hpp"
#include "interface_objects.hpp"
#include "interface_trajectories.hpp"
#include "interface_vehicle_state.hpp"

// Parameter Header
#include "virtual_object_generator_config.hpp"

using namespace std;
using namespace interface;

class ScenarioParser {
public:
    explicit ScenarioParser();
    virtual ~ScenarioParser();

    bool         Init(const VirtualObjectGeneratorParams& params);
    TrackObjects ParseScenario(const VehicleState& vehicle_state, const vector<double>& ego_frenet,
                               tk::Map& road_map, tk::spline& left_boundary, tk::spline& right_boundary,
                               const VirtualObjectGeneratorParams& params);

private:
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Functions

    std::vector<std::vector<double>> ReadCSV(const std::string& filename);
    bool                             isCSVDataValid(const std::vector<std::vector<double>>& csv_data);
    TrackObject                      GetVirtualObjectWithNoise(const std::vector<std::vector<double>>& csv_data, const int& index,
                                                               const VirtualObjectGeneratorParams& params);
    TrackObject                      GetVirtualObjectWithTrackingNoise(const std::vector<std::vector<double>>& csv_data, const int& index,
                                                                       const VirtualObjectGeneratorParams&     params,
                                                                       const std::vector<std::vector<double>>& noise_csv_data);

    double GenerateGaussianNoise(double mean, double std_dev);

private:
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Variables

    // Inputs
    std::vector<std::vector<std::vector<double>>> i_csv_data_;
    std::vector<std::vector<std::vector<double>>> i_csv_noise_data_;

    // Variables
    int time_elapsed_ = 0;
};

#endif // __VEHICLE_INFO_GENERATOR__