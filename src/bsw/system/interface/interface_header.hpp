/**
 * @file        interface_header.hpp
 * @brief       interface for header
 * 
 * @authors     Seheon Ha (seheonha@hayang.ac.kr)          
 * 
 * @date        2024-04-24 created by Seheon Ha
 *              
 */

#ifndef __INTERFACE_HEADER_HPP__
#define __INTERFACE_HEADER_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

namespace interface {
    typedef struct{
        uint32_t    seq;
        double      stamp{0.0};
        std::string frame_id;
    } Header;
} // namespace interface

#endif // __INTERFACE_HEADER_HPP__