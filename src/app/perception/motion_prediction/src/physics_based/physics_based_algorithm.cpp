/**
 * @file        physics_based_algorithm.cpp
 * @brief       physics based motion prediction algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)        
 * 
 * @date        2024-04-13 created by Yuseung Na
 * 
 */

#include "physics_based/physics_based_algorithm.hpp"

PhysicsBased::PhysicsBased() {
    // Initialize
}

PhysicsBased::~PhysicsBased() {
    // Terminate
}

interface::Object3DState PhysicsBased::PredictWithCV(const interface::Object3DState& prev_state,
                                                     const MotionPredictionConfig& cfg) {
    interface::Object3DState object_state;

    // Constant Velocity Model
    object_state.v_x = prev_state.v_x;
    object_state.v_y = prev_state.v_y;
    object_state.v_z = prev_state.v_z;

    // Update States
    object_state.x = prev_state.x + object_state.v_x * cfg.dt;
    object_state.y = prev_state.y + object_state.v_y * cfg.dt;
    object_state.z = prev_state.z + object_state.v_z * cfg.dt;

    object_state.roll  = prev_state.roll;
    object_state.pitch = prev_state.pitch;
    object_state.yaw   = prev_state.yaw;

    object_state.a_x = prev_state.a_x;
    object_state.a_y = prev_state.a_y;
    object_state.a_z = prev_state.a_z;

    object_state.roll_rate  = prev_state.roll_rate;
    object_state.pitch_rate = prev_state.pitch_rate;
    object_state.yaw_rate   = prev_state.yaw_rate;

    return object_state;
}

interface::Object3DState PhysicsBased::PredictWithCA(const interface::Object3DState& prev_state,
                                                     const MotionPredictionConfig& cfg) {
    interface::Object3DState object_state;

    // Constant Acceleration Model
    object_state.a_x = prev_state.a_x;
    object_state.a_y = prev_state.a_y;
    object_state.a_z = prev_state.a_z;

    object_state.v_x = prev_state.v_x + object_state.a_x * cfg.dt;
    object_state.v_y = prev_state.v_y + object_state.a_y * cfg.dt;
    object_state.v_z = prev_state.v_z + object_state.a_z * cfg.dt;

    // Update States
    object_state.x = prev_state.x + object_state.v_x * cfg.dt;
    object_state.y = prev_state.y + object_state.v_y * cfg.dt;
    object_state.z = prev_state.z + object_state.v_z * cfg.dt;

    object_state.roll  = prev_state.roll;
    object_state.pitch = prev_state.pitch;

    if (fabs(object_state.v_x <= 0.1 && fabs(object_state.v_y) <= 0.1)) {
        object_state.yaw = prev_state.yaw;
    }
    else if (object_state.v_x != 0.0) {
        object_state.yaw = atan2(object_state.v_y, object_state.v_x);
    }
    else {
        object_state.yaw = prev_state.yaw;
    }

    object_state.roll_rate  = prev_state.roll_rate;
    object_state.pitch_rate = prev_state.pitch_rate;
    object_state.yaw_rate   = prev_state.yaw_rate;

    return object_state;
}

interface::Object3DState PhysicsBased::PredictWithCTRV(const interface::Object3DState& prev_state,
                                                       const MotionPredictionConfig& cfg) {
    interface::Object3DState object_state;

    // Constant Turn Rate and Velocity Model
    object_state.yaw_rate = prev_state.yaw_rate;

    object_state.v_x = prev_state.v_x * cos(object_state.yaw_rate * cfg.dt)
                         - prev_state.v_y * sin(object_state.yaw_rate * cfg.dt);
    object_state.v_y = prev_state.v_y * cos(object_state.yaw_rate * cfg.dt)
                         + prev_state.v_x * sin(object_state.yaw_rate * cfg.dt);
    object_state.v_z = prev_state.v_z;

    // Update States
    object_state.x = prev_state.x + object_state.v_x * cfg.dt;
    object_state.y = prev_state.y + object_state.v_y * cfg.dt;
    object_state.z = prev_state.z + object_state.v_z * cfg.dt;

    object_state.roll  = prev_state.roll;
    object_state.pitch = prev_state.pitch;
    object_state.yaw   = prev_state.yaw + object_state.yaw_rate * cfg.dt;

    object_state.a_x = prev_state.a_x;
    object_state.a_y = prev_state.a_y;
    object_state.a_z = prev_state.a_z;

    object_state.roll_rate  = prev_state.roll_rate;
    object_state.pitch_rate = prev_state.pitch_rate;

    return object_state;
}

interface::Object3DState PhysicsBased::PredictWithCTRA(const interface::Object3DState& prev_state,
                                                       const MotionPredictionConfig& cfg) {
    interface::Object3DState object_state;

    // Constant Turn Rate and Acceleration Model
    object_state.yaw_rate = prev_state.yaw_rate;

    object_state.a_x = prev_state.a_x;
    object_state.a_y = prev_state.a_y;
    object_state.a_z = prev_state.a_z;

    object_state.v_x = (prev_state.v_x + object_state.a_x * cfg.dt) * cos(object_state.yaw_rate * cfg.dt)
                         - (prev_state.v_y + object_state.a_y * cfg.dt) * sin(object_state.yaw_rate * cfg.dt);
    object_state.v_y = (prev_state.v_y + object_state.a_y * cfg.dt) * cos(object_state.yaw_rate * cfg.dt)
                         + (prev_state.v_x + object_state.a_x * cfg.dt) * sin(object_state.yaw_rate * cfg.dt);
    object_state.v_z = prev_state.v_z + object_state.a_z * cfg.dt;

    // Update States
    object_state.x = prev_state.x + object_state.v_x * cfg.dt;
    object_state.y = prev_state.y + object_state.v_y * cfg.dt;
    object_state.z = prev_state.z + object_state.v_z * cfg.dt;

    object_state.roll  = prev_state.roll;
    object_state.pitch = prev_state.pitch;
    object_state.yaw   = prev_state.yaw + object_state.yaw_rate * cfg.dt;

    object_state.roll_rate  = prev_state.roll_rate;
    object_state.pitch_rate = prev_state.pitch_rate;

    return object_state;
}

interface::PredictObjects PhysicsBased::PredictMotion(const interface::PredictObjects& processed_objects,
                                                      const MotionPredictionConfig& cfg) {
    interface::PredictObjects predict_objects;
    predict_objects.header = processed_objects.header;

    uint16_t predict_horizon_size = static_cast<uint16_t>(std::round(cfg.prediction_horizon / cfg.dt));

    for (const auto& object : processed_objects.object) {
        interface::PredictObject predict_object;
        predict_object.id               = object.id;
        predict_object.classification   = object.classification;
        predict_object.dynamic_state    = object.dynamic_state;
        predict_object.dimension        = object.dimension;
        predict_object.reference        = object.reference;

        // Current Motion
        PredictObjectMultimodal state_multi;
        state_multi.probability = 1.0;
        state_multi.state.push_back(object.state.at(0));

        // Predict Motion
        // ADD PREDICTION ALGORITHM HERE
        for (uint16_t i = 1; i < predict_horizon_size; i++) {
            interface::Object3DState object_state;

            if (cfg.physics_model == "CV") {
                object_state = PredictWithCV(state_multi.state.at(i-1), cfg);
            }
            else if (cfg.physics_model == "CA") {
                object_state = PredictWithCA(state_multi.state.at(i-1), cfg);
            }
            else if (cfg.physics_model == "CTRV") {
                object_state = PredictWithCTRV(state_multi.state.at(i-1), cfg);
            }
            else if (cfg.physics_model == "CTRA") {
                object_state = PredictWithCTRA(state_multi.state.at(i-1), cfg);
            }
            else {
                std::cout << "Invalid Physics Model!!! Please check the configuration." << std::endl;
                object_state = state_multi.state.at(i-1);
            }

            object_state.header.stamp = state_multi.state.at(i-1).header.stamp + cfg.dt;            
            state_multi.state.push_back(object_state);
        }
        predict_object.state_multi.push_back(state_multi);
        predict_objects.object.push_back(predict_object);
    }

    return predict_objects;
}

interface::PredictObjects PhysicsBased::RunAlgorithm(const interface::PredictObjects& processed_objects,
                                                     const MotionPredictionConfig& cfg) {
    // Run Algorithm
    interface::PredictObjects predict_objects = PredictMotion(processed_objects, cfg);

    // Check the output is valid
    if (predict_objects.object.size() == processed_objects.object.size()) {
        o_predict_objects_ = predict_objects;
    }    
    else {
        util_function::DebugPrintError(
            "Input track object size and Output predict object size is different!!!"
        );
    }

    // Return
    return o_predict_objects_;
}