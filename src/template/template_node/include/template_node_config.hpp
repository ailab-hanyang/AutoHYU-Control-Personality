/**
 * @file        template_node_config.hpp
 * @brief       template configuration hpp file for template node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#ifndef __TEMPLATE_NODE_CONFIG_HPP__
#define __TEMPLATE_NODE_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    std::string mode{"real"};           // real, carla, morai, carmaker, ...
    std::string location{"kcity"};      // kcity, katri, grandpark, ...
    std::string map{"/resources/map/kcity/kcity.osm"};
    double reference_latitude{0.0};
    double reference_longitude{0.0};
    
    std::string node_config1{""};
    double node_config2{0.0};
    int node_config3{0};

    // Algorithm 1 Config
    std::string algorithm1_config1{""};
    double algorithm1_config2{0.0};
    int algorithm1_config3{0};

    // Algorithm 2 Config
    std::string algorithm2_config1{""};
    double algorithm2_config2{0.0};
    int algorithm2_config3{0};

} TemplateNodeConfig;

#endif // __TEMPLATE_NODE_CONFIG_HPP__