/**
 * @file        function_control.hpp
 * @brief       util functions for control
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2023-07-14 created by Yuseung Na
 *              2024-04-11 updated by Yuseung Na: Refactoring 
 */

#ifndef __FUNCTION_CONTROL_HPP__
#define __FUNCTION_CONTROL_HPP__
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
#include "function_print.hpp"

using namespace interface;

namespace util_function {
    inline double CalculateMaxWheelTorque(const double& speed) {
        double torque = 0.0;

        if (speed <= 35.0) {
            torque = speed          * 0.0       + 6443.0;
        }
        else if (speed <= 40.0) {
            torque = (speed - 35.0) * (-28.116) + 6443.0;
        }
        else if (speed <= 50.0) {
            torque = (speed - 40.0) * (-31.631) + 6302.42;
        }
        else if (speed <= 60.0) {
            torque = (speed - 50.0) * (-44.816) + 5986.11;
        }
        else if (speed <= 70.0) {
            torque = (speed - 60.0) * (-71.095) + 5537.95;
        }
        else if (speed <= 80.0) {
            torque = (speed - 70.0) * (-69.0)   + 4827.0;
        }
        else if (speed <= 90.0) {
            torque = (speed - 80.0) * (-52.4)   + 4137.0;
        }
        else if (speed <= 100.0) {
            torque = (speed - 90.0) * (-41.8)   + 3613.0;
        }
        else if (speed <= 120.0) {
            torque = (speed - 100.0) * (-16.65) + 3195.0;
        }
        else if (speed <= 140.0) {
            torque = (speed - 120.0) * (-26.5)  + 2862.0;
        }
        else if (speed <= 160.0) {
            torque = (speed - 140.0) * (-20.25) + 2332.0;
        }
        else if (speed <= 180.0) {
            torque = (speed - 160.0) * (-15.2)  + 1927.0;
        }
        else if (speed <= 185.0) {
            torque = (speed - 180.0) * (-47.0)  + 1623.0;
        }
        else {
            torque = std::max(0.0, (speed - 185.0) * (-201.3134) + 1388.0);
        }

        return torque;
    }
} // namespace util_function

#endif  // __FUNCTION_CONTROL_HPP__