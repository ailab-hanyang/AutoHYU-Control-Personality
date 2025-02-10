/**
 * @file        state_using_novatel_algorithm.cpp
 * @brief       algorithm cpp file for state using novatel algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)
 * 
 * @date        2024-04-01 created by Yuseung Na
 * 
 */

#include "state_using_novatel/state_using_novatel_algorithm.hpp"

StateUsingNovatel::StateUsingNovatel() {
    // Initialize
}

StateUsingNovatel::~StateUsingNovatel() {
    // Terminate
}

interface::VehicleState StateUsingNovatel::MergeInspvaxCorrimu(const interface::INSPVAX& inspvax,
                                                               const interface::CORRIMU& corrimu,
                                                               const StateEstimationConfig& cfg) {
    interface::VehicleState vehicle_state;

    vehicle_state.header = inspvax.header;
    vehicle_state.reference.projection         = cfg.projector;
    vehicle_state.reference.wgs84.latitude     = cfg.reference_latitude;
    vehicle_state.reference.wgs84.longitude    = cfg.reference_longitude;
    vehicle_state.reference.wgs84.altitude     = 0.0;

    vehicle_state.pos_type           = inspvax.pos_type;

    vehicle_state.gnss.latitude      = inspvax.latitude;
    vehicle_state.gnss.longitude     = inspvax.longitude;
    vehicle_state.gnss.altitude      = inspvax.height;

    vehicle_state.gnss_stdev.latitude     = inspvax.latitude_stdev;
    vehicle_state.gnss_stdev.longitude    = inspvax.longitude_stdev;
    vehicle_state.gnss_stdev.altitude     = inspvax.height_stdev;

    // Position projection
    if (cfg.projector == "local_cartesian") {
        GeographicLib::LocalCartesian proj(cfg.reference_latitude, 
                                           cfg.reference_longitude, 
                                           0.0, 
                                           GeographicLib::Geocentric::WGS84());
        proj.Forward(vehicle_state.gnss.latitude, vehicle_state.gnss.longitude, vehicle_state.gnss.altitude, 
                     vehicle_state.x, vehicle_state.y, vehicle_state.z);
    } else if (cfg.projector == "utm") {
        // Set UTM zone
        int zone_{}; bool isInNorthernHemisphere_{true}, throwInPaddingArea_{};
        double utm_x; double utm_y;
        GeographicLib::UTMUPS::Forward(cfg.reference_latitude, cfg.reference_longitude, 
                                       zone_, isInNorthernHemisphere_, utm_x, utm_y);

        // Transfer UTM zone
        int zone{};  bool northp{};
        try {
            GeographicLib::UTMUPS::Forward(vehicle_state.gnss.latitude, vehicle_state.gnss.longitude, 
                                           zone, northp, vehicle_state.x, vehicle_state.y);
        } catch (GeographicLib::GeographicErr& e) {
            util_function::DebugPrintError("GeographicLib::UTMUPS::Forward() error: " + std::string(e.what()));
        }

        if (zone != zone_ || northp != isInNorthernHemisphere_) {
            if (throwInPaddingArea_) {
                util_function::DebugPrintError("You have left the UTM zone or changed the hemisphere!");
            }
            
            // try to transfer to the desired zone
            double xAfterTransfer = 0;
            double yAfterTransfer = 0;
            int zoneAfterTransfer = 0;
            try {
                GeographicLib::UTMUPS::Transfer(zone, northp, vehicle_state.x, vehicle_state.y, 
                                                zone_, isInNorthernHemisphere_, xAfterTransfer, yAfterTransfer, zoneAfterTransfer);
            } catch (GeographicLib::GeographicErr& e) {
                util_function::DebugPrintError("GeographicLib::UTMUPS::Transfer() error: " + std::string(e.what()));
            }

            if (zoneAfterTransfer != zone_) {
                util_function::DebugPrintError("You have left the padding area of the UTM zone!");
            }
            vehicle_state.x = xAfterTransfer;
            vehicle_state.y = yAfterTransfer;
        }
    } else {
        vehicle_state.x = 0.0;
        vehicle_state.y = 0.0;
        vehicle_state.z = 0.0;

        std::string msg = "Invalid projector type!!! : " + cfg.projector;
        util_function::DebugPrintError(msg);
    }
    vehicle_state.z = 0.0;

    vehicle_state.vx = (inspvax.north_velocity * cos(inspvax.azimuth * DEG2RAD) + 
                        inspvax.east_velocity  * sin(inspvax.azimuth * DEG2RAD)) * cos(-inspvax.pitch*DEG2RAD);
    vehicle_state.vy = (-inspvax.north_velocity * sin(inspvax.azimuth * DEG2RAD) + 
                         inspvax.east_velocity  * cos(inspvax.azimuth * DEG2RAD)) * cos(inspvax.roll*DEG2RAD);
    vehicle_state.vz = inspvax.up_velocity * cos(-inspvax.pitch*DEG2RAD) * cos(inspvax.roll*DEG2RAD);

    vehicle_state.ax = corrimu.longitudinal_acc*100.0;
    vehicle_state.ay = corrimu.lateral_acc*100.0;
    vehicle_state.az = corrimu.vertical_acc*100.0;

    vehicle_state.roll     = inspvax.roll * DEG2RAD;
    vehicle_state.pitch    = inspvax.pitch * DEG2RAD;
    vehicle_state.yaw      = (90.0 - inspvax.azimuth) * DEG2RAD;

    vehicle_state.roll_vel     = corrimu.roll_rate * 100.0;
    vehicle_state.pitch_vel    = corrimu.pitch_rate * 100.0;
    vehicle_state.yaw_vel      = corrimu.yaw_rate * 100.0;    

    vehicle_state.roll_stdev   = inspvax.roll_stdev * DEG2RAD;
    vehicle_state.pitch_stdev  = inspvax.pitch_stdev * DEG2RAD;
    vehicle_state.yaw_stdev    = inspvax.azimuth_stdev * DEG2RAD;
    
    return vehicle_state;
}

interface::VehicleState StateUsingNovatel::MergeNovatelCAN(const interface::VehicleState& vehicle_state,
                                                           const interface::VehicleCAN& vehicle_can,
                                                           const StateEstimationConfig& cfg) {
    interface::VehicleState o_vehicle_state = vehicle_state;
    o_vehicle_state.vehicle_can = vehicle_can;

    return o_vehicle_state;
}

interface::VehicleState StateUsingNovatel::RunAlgorithm(const interface::INSPVAX& inspvax,
                                                        const interface::CORRIMU& corrimu,
                                                        const interface::VehicleCAN& vehicle_can,
                                                        const StateEstimationConfig& cfg) {
    // INSPVAX + CORRIMU
    interface::VehicleState vehicle_state = MergeInspvaxCorrimu(inspvax, corrimu, cfg);

    // NovAtel + VehicleCAN
    vehicle_state = MergeNovatelCAN(vehicle_state, vehicle_can, cfg);

    // Check the output is valid
    if (vehicle_state.header.stamp != 0.0) {
        o_vehicle_state_ = vehicle_state;
    }

    return o_vehicle_state_;
}