/**
 * @file        function_lanelet.hpp
 * @brief       util functions for lanelet
 * 
 * @authors     Junhee Lee (998jun@gmail.com)          
 *              Seheon Ha (seheonha@hanyang.ac.kr)
 * 
 * @date        2024-04-11 created by Junhee Lee 
 *              2024-06-21 modified by Seheon Ha: add CheckAndBuildRoutingGraph function
 * 
 */
#ifndef __FUNCTION_LANELET_HPP__
#define __FUNCTION_LANELET_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <sstream>

// Utility header
#include <lanelet2_core/geometry/LaneletMap.h>
#include "lanelet2_io/io_handlers/Serialize.h"
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

// Interface Header
#include "interface_lanelet.hpp"
#include "function_print.hpp"

#include "spline.h"
using namespace interface;

namespace util_function {
    inline std::vector<int8_t> EncodeLaneletMapBin(const lanelet::LaneletMapPtr& map){
        std::stringstream ss;
        boost::archive::binary_oarchive oa(ss);
        oa << *map;
        std::string str = ss.str();
        std::vector<int8_t> data(str.begin(), str.end());
        return data;
    } 

    inline lanelet::LaneletMapPtr DecodeLaneletMapBin(const std::vector<int8_t>& data){
        std::string str(data.begin(), data.end());
        std::stringstream ss(str);
        lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
        util_function::DebugPrintWarn(
            "Decode Lanelet Map Bin File ... "
        );
        boost::archive::binary_iarchive ia(ss);
        ia >> *map;
        return map;
    } 

    inline lanelet::routing::RoutingGraphUPtr BuildRoutingGraph(const lanelet::LaneletMapPtr& map){
        lanelet::traffic_rules::TrafficRulesPtr trafficRules =
                lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
        return lanelet::routing::RoutingGraph::build(*map, *trafficRules);
    }

    inline void CheckAndBuildRoutingGraph(const lanelet::LaneletMapPtr& lanelet_map, 
                                          lanelet::routing::RoutingGraphUPtr& routing_graph,
                                          uint32_t& prev_map_seq,
                                          const uint32_t& map_seq){
        if (map_seq != prev_map_seq) {
            util_function::DebugPrintWarn(
                "Map has been updated. Build Routing Graph"
            );
            routing_graph = BuildRoutingGraph(lanelet_map);
            prev_map_seq = map_seq;
        }
    }
}

#endif // __FUNCTION_LANELET_HPP__
