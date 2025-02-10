/**
 * @file        can_parser_config.hpp
 * @brief       configuration hpp file for can parser node
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-04-12 created by Yuseung Na
 * 
 */

#ifndef __CAN_PARSER_CONFIG_HPP__
#define __CAN_PARSER_CONFIG_HPP__
#pragma once

// STD Header
#include <string>

typedef struct {  
    // Node Config
    std::string mode{"real"};           // real, carla, morai, carmaker, ...
    std::string vehicle{"lab_ioniq"};   // hmg_ioniq
} CANParserConfig;

#endif // __CAN_PARSER_CONFIG_HPP__