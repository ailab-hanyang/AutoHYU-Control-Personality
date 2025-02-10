/**
 * @file        preprocess_algorithm.cpp
 * @brief       algorithm cpp file for preprocess algorithm
 * 
 * @authors     Yuseung Na (yuseungna@hanyang.ac.kr)          
 * 
 * @date        2024-07-02 created by Yuseung Na
 * 
 */

#include "preprocess/preprocess_algorithm.hpp"

Preprocess::Preprocess() {
    // Initialize
}

Preprocess::~Preprocess() {
    // Terminate
}

interface::TrackObject Preprocess::ProcessObject(const std::vector<interface::Object3DState>& object_states, 
                                                 const MotionPredictionConfig& cfg) {
    interface::TrackObject processed_object;

    // If not enough object states to filter, return
    if(object_states.size() <= 1) {
        return processed_object;
    }

    // Filtering values
    double yaw_rate_sum{0.0};
    double estimated_yaw_rate;
    double cur_yaw_diff;
    interface::Object3DState temp_object_state;    

    for(int i = 0; i < object_states.size(); i++){
        if(i == 0){
            temp_object_state = object_states.front();
            continue;
        }

        temp_object_state.v_x = (1.0 - cfg.track_filter_alpha) * temp_object_state.v_x + 
                                            cfg.track_filter_alpha * object_states[i].v_x;
        temp_object_state.v_y = (1.0 - cfg.track_filter_alpha) * temp_object_state.v_y + 
                                            cfg.track_filter_alpha * object_states[i].v_y;
        temp_object_state.a_x = (1.0 - cfg.track_filter_alpha) * temp_object_state.a_x + 
                                            cfg.track_filter_alpha * object_states[i].a_x;
        temp_object_state.a_y = (1.0 - cfg.track_filter_alpha) * temp_object_state.a_y + 
                                            cfg.track_filter_alpha * object_states[i].a_y;

        cur_yaw_diff = util_function::NormalizeAngleDiffRad(object_states[i].yaw, object_states[i-1].yaw);
        estimated_yaw_rate = cur_yaw_diff / (object_states[i].header.stamp - object_states[i-1].header.stamp);
        yaw_rate_sum += estimated_yaw_rate;
    }
    double avg_yaw_rate = yaw_rate_sum / object_states.size();

    // Sigmoid Function (for continuity) to make yawrate goes zero under cfg_.yawrate_zero_vel_ms
    avg_yaw_rate = avg_yaw_rate * 1.0/(1 + exp(-5.0*(sqrt(temp_object_state.v_x * temp_object_state.v_x 
                                                        + temp_object_state.v_y * temp_object_state.v_y)
                                                    - cfg.yawrate_zero_vel_ms)));

    // Update object state
    temp_object_state.header.stamp = object_states.back().header.stamp;
    temp_object_state.x = object_states.back().x;
    temp_object_state.y = object_states.back().y;
    temp_object_state.z = object_states.back().z;
    temp_object_state.yaw = object_states.back().yaw;
    temp_object_state.yaw_rate = avg_yaw_rate;

    processed_object.state = temp_object_state;

    return processed_object;
}

interface::TrackObjects Preprocess::ManageTrackBuffer(const interface::TrackObjects& track_objects,
                                                      const MotionPredictionConfig& cfg) {
    interface::TrackObjects processed_objects;
    processed_objects.header = track_objects.header;

    // If no object, return    
    if(track_objects.object.empty() == true) {
        return processed_objects;
    }

    // For each object
    for(const auto& object : track_objects.object) {
        interface::TrackObject   tmp_track_object;
        interface::Object3DState cur_track_state = object.state;        
        std::unordered_map<int, std::vector<interface::Object3DState>>::iterator iter_track_history;

        int object_id = object.id;

        // Check if track exist
        iter_track_history = track_state_buffer_.find(object_id);

        // If no track, make new track
        if(track_state_buffer_.find(object_id) == track_state_buffer_.end()) {   
            std::vector<interface::Object3DState> vec_new_object_state;

            vec_new_object_state.push_back(cur_track_state);
            track_state_buffer_[object_id] = vec_new_object_state; 

            tmp_track_object = object;
        }
        // If track exist, process
        else {
            interface::TrackObject processed_track_object;

            // Check if new data
            if(fabs(track_state_buffer_[object_id].back().header.stamp - cur_track_state.header.stamp) > FLT_MIN){
                track_state_buffer_[object_id].push_back(cur_track_state);     
            }

            // Erase too old data
            while(track_state_buffer_[object_id].size() > cfg.track_buffer_size){
                track_state_buffer_[object_id].erase(track_state_buffer_[object_id].begin());
            }

            // Process object 
            processed_track_object = ProcessObject(track_state_buffer_[object_id], cfg);
            processed_track_object.id = object_id;
            processed_track_object.classification = object.classification;
            processed_track_object.dynamic_state = object.dynamic_state;
            processed_track_object.dimension = object.dimension;

            tmp_track_object = processed_track_object;
        }

        // Push back
        processed_objects.object.push_back(tmp_track_object);
    }
    
    return processed_objects;
}

void Preprocess::ClearOldTrack(const double& last_vehicle_state_time,
                               const MotionPredictionConfig& cfg) {
    for (auto it = track_state_buffer_.begin(); it != track_state_buffer_.end(); ) {
        int key = it->first;
        std::vector<interface::Object3DState>& value = it->second;

        if (last_vehicle_state_time - value.back().header.stamp > cfg.track_erase_time_sec){
            it = track_state_buffer_.erase(it);
            
            continue;
        }  
        ++it;      
    }

    return;
}

interface::TrackObjects Preprocess::CompensateDelay(const interface::TrackObjects& processed_objects, 
                                                   const double& current_time,
                                                   const MotionPredictionConfig& cfg) {
    interface::TrackObjects compensated_objects;
    compensated_objects.header.stamp = current_time;

    if(processed_objects.object.empty() != true) {
        for(const auto& object : processed_objects.object) {
            double delta_time = current_time - object.state.header.stamp;
            interface::TrackObject compensated_object = PredObjToDeltaTime(object, delta_time);
            
            compensated_objects.object.push_back(compensated_object);
        }
    }
    
    return compensated_objects;
}

interface::TrackObject Preprocess::PredObjToDeltaTime(const interface::TrackObject& processed_object, 
                                                      const double& delta_time) {
    interface::TrackObject compensated_object;
    
    // Copy object_property
    compensated_object.id                   = processed_object.id;
    compensated_object.classification       = processed_object.classification;
    compensated_object.dynamic_state        = processed_object.dynamic_state;
    compensated_object.dimension            = processed_object.dimension;
    compensated_object.state                = processed_object.state;
    compensated_object.state.header.stamp   = processed_object.state.header.stamp + delta_time;
    
    if(compensated_object.dynamic_state == interface::ObjectDynamicState::STATIC){
        // if static, do not change
        return compensated_object;
    }

    // Calculate next step pose, vel, acc. With ConstanceVelocity Model
    double prev_x = compensated_object.state.x;
    double prev_y = compensated_object.state.y;
    double prev_vx = compensated_object.state.v_x;
    double prev_vy = compensated_object.state.v_y;

    // Predict properties
    double curr_x = prev_x + delta_time * prev_vx;
    double curr_y = prev_y + delta_time * prev_vy;

    // Update propertiexs
    compensated_object.state.x = curr_x;
    compensated_object.state.y = curr_y;

    return compensated_object;
}

float Preprocess::GetPointDistance(float& x1, float& y1 , float& x2, float& y2){
    float distance = sqrt(pow((x1-x2),2) + pow((y1-y2),2));
    return distance;
}

float Preprocess::VectorDotProduct2D(const std::pair<float, float> v1, const std::pair<float, float> v2){
    float ans;
    float v1_size = sqrt(pow(v1.first,2)+pow(v1.second,2));
    float v2_size = sqrt(pow(v2.first,2)+pow(v2.second,2));
    
    float v1_x = v1.first / v1_size;
    float v1_y = v1.second / v1_size;

    float v2_x = v2.first / v2_size;
    float v2_y = v2.second / v2_size; 

    ans = v1_x * v2_x + v1_y * v2_y;
    return ans;
}



interface::PredictObjects Preprocess::AddReferenceTrajectory(const interface::TrackObjects& compensated_track_objects,
                                                             const lanelet::LaneletMapPtr& map_lanelet,
                                                             const lanelet::routing::RoutingGraphUPtr& routing_graph,
                                                             const MotionPredictionConfig& cfg) {
    interface::PredictObjects objects_with_reference;
    // For loop for each object
    for (const auto& object : compensated_track_objects.object) {

        lanelet::BasicPoint2d cur_point = lanelet::BasicPoint2d(object.state.x, object.state.y);
        std::vector<std::pair<double, lanelet::Lanelet>> cur_lanelets = lanelet::geometry::findNearest(map_lanelet->laneletLayer, cur_point , 5);
        lanelet::ConstLanelet object_lanelet;
        object_lanelet = map_lanelet->laneletLayer.get(cur_lanelets[0].second.id());


        if (routing_graph->conflicting(map_lanelet->laneletLayer.get(object_lanelet.id())).size() >= 2){
            std::pair<float, float> v_ego;
            v_ego.first = std::cos(object.state.yaw);
            v_ego.second = std::sin(object.state.yaw);
            std::pair<float, float> v_ll;

        
            v_ll.first = object_lanelet.centerline()[object_lanelet.centerline().size()-1].x() - object_lanelet.centerline()[0].x();
            v_ll.second = object_lanelet.centerline()[object_lanelet.centerline().size()-1].y() - object_lanelet.centerline()[0].y();

            float v_result_prev = VectorDotProduct2D(v_ego, v_ll);
            float v_result;
            int max_dotproduct = 0;
            for (int i = 1; i < cur_lanelets.size(); i++){

                object_lanelet = map_lanelet->laneletLayer.get(cur_lanelets[i].second.id());

                v_ll.first = object_lanelet.centerline()[object_lanelet.centerline().size()-1].x() - object_lanelet.centerline()[0].x();
                v_ll.second = object_lanelet.centerline()[object_lanelet.centerline().size()-1].y() - object_lanelet.centerline()[0].y();

                v_result = VectorDotProduct2D(v_ego, v_ll);
                if (v_result > v_result_prev){
                    max_dotproduct = i;
                    v_result_prev = v_result;
                }

            }
            object_lanelet = map_lanelet->laneletLayer.get(cur_lanelets[max_dotproduct].second.id());
        }

        
        lanelet::ConstLanelets previous_lanelets = routing_graph->previous(map_lanelet->laneletLayer.get(object_lanelet.id()));
        lanelet::ConstLanelets following_lanelets = routing_graph->following(map_lanelet->laneletLayer.get(object_lanelet.id()));

        interface::PredictObject predict_object;
        predict_object.id = object.id;
        predict_object.classification = object.classification;
        predict_object.dynamic_state = object.dynamic_state;
        predict_object.dimension = object.dimension;

        // Reference trajectory generation
        interface::ReferenceTrajectory reference_trajectory;
        interface::ReferenceTrajectoryPoint reference_trajectory_point;

        float prev_point_x = object.state.x;
        float prev_point_y = object.state.y;
        float centerline_x;
        float centerline_y;
        // previous 
        if (previous_lanelets.size() >= 1){
            for (int i = 0; i < previous_lanelets[0].centerline().size(); i++){
            centerline_x = previous_lanelets[0].centerline()[i].x();
            centerline_y = previous_lanelets[0].centerline()[i].y();
            if (GetPointDistance(prev_point_x, prev_point_y, centerline_x, centerline_y) < 5){
                reference_trajectory_point.x = centerline_x;
                reference_trajectory_point.y = centerline_y;
                reference_trajectory.point.push_back(reference_trajectory_point);
                }
            }
        }
        
        //now
        float ref_distance = std::max(50.0, abs(object.state.v_x)*cfg.prediction_horizon);
        std::pair<float, float> v_prev2end;
        v_prev2end.first = object_lanelet.centerline()[object_lanelet.centerline().size()-1].x() - prev_point_x;
        v_prev2end.second = object_lanelet.centerline()[object_lanelet.centerline().size()-1].y() - prev_point_y;
        for(int i = 0; i < object_lanelet.centerline().size(); i++){
            centerline_x = object_lanelet.centerline()[i].x();
            centerline_y = object_lanelet.centerline()[i].y();
            std::pair<float, float> v_prev2center;
            v_prev2center.first = object_lanelet.centerline()[i].x() - prev_point_x;
            v_prev2center.second = object_lanelet.centerline()[i].y() - prev_point_y;
            if (v_prev2end.first * v_prev2center.first + v_prev2end.second * v_prev2center.second < 0){
                if (GetPointDistance(prev_point_x, prev_point_y, centerline_x, centerline_y) < 5){
                    reference_trajectory_point.x = centerline_x;
                    reference_trajectory_point.y = centerline_y;
                    reference_trajectory.point.push_back(reference_trajectory_point);
                }
            }
            else{
                if (GetPointDistance(prev_point_x, prev_point_y, centerline_x, centerline_y) < ref_distance){
                    reference_trajectory_point.x = centerline_x;
                    reference_trajectory_point.y = centerline_y;
                    reference_trajectory.point.push_back(reference_trajectory_point);
                }
            }
        }

        if (following_lanelets.size() >= 1){
            for(int i =0; i < following_lanelets.size(); i++){
                interface::ReferenceTrajectory start_trajectory = reference_trajectory;
                for (int k = 0; k < following_lanelets[i].centerline().size(); k++){
                    centerline_x = following_lanelets[i].centerline()[k].x();
                    centerline_y = following_lanelets[i].centerline()[k].y();
                    if (GetPointDistance(prev_point_x, prev_point_y, centerline_x, centerline_y) < ref_distance){
                        reference_trajectory_point.x = centerline_x;
                        reference_trajectory_point.y = centerline_y;
                        start_trajectory.point.push_back(reference_trajectory_point);
                    }
                }

                predict_object.reference.push_back(start_trajectory);                
            }
        }
        predict_object.state.push_back(object.state);
        objects_with_reference.object.push_back(predict_object);
    }


    return objects_with_reference;
}

interface::PredictObjects Preprocess::RunAlgorithm(const double& current_time,
                                                   const interface::TrackObjects& track_objects,
                                                   const interface::Trajectory& trajectory,
                                                //    const lanelet::LaneletMapPtr& map_lanelet,
                                                //    const uint32_t& map_seq,
                                                   const MotionPredictionConfig& cfg) {
                                                    // removing map loader dependency

    // Manage track buffer
    interface::TrackObjects processed_objects = ManageTrackBuffer(track_objects, cfg);
    ClearOldTrack(current_time, cfg);

    // Compensate time delay
    interface::TrackObjects compensated_track_objects = CompensateDelay(processed_objects, current_time, cfg);

    // Add reference trajectory
    // util_function::CheckAndBuildRoutingGraph(map_lanelet, routing_graph_, prev_map_seq_, map_seq);
    // interface::PredictObjects objects_with_reference = AddReferenceTrajectory(track_objects, map_lanelet, routing_graph_, cfg);    
    // objects_with_reference.header = track_objects.header;

    // // Check the output is valid
    // if (objects_with_reference.object.size() == track_objects.object.size()) {
    //     o_processed_objects_ = objects_with_reference;
    // }    
    // else {
    //     util_function::DebugPrintError(
    //         "Input track object size and Output predict object size is different!!!"
    //     );
    // }

    // Return
    interface::ReferenceTrajectory ref_traj;
    for (int i = 0; i < trajectory.point.size(); i++) {
        interface::ReferenceTrajectoryPoint ref_point;
        ref_point.x = trajectory.point[i].x;
        ref_point.y = trajectory.point[i].y;
        ref_traj.point.push_back(ref_point);
    }

    interface::PredictObjects output;
    output.header.stamp = current_time;
    output.header.frame_id = "world";
    for (int i = 0; i < compensated_track_objects.object.size(); i++){
        interface::PredictObject predict_object;
        predict_object.id = compensated_track_objects.object[i].id;
        predict_object.classification = compensated_track_objects.object[i].classification;
        predict_object.dynamic_state = compensated_track_objects.object[i].dynamic_state;
        predict_object.reference.push_back(ref_traj);
        predict_object.dimension = compensated_track_objects.object[i].dimension;
        predict_object.state.push_back(compensated_track_objects.object[i].state);
        output.object.push_back(predict_object);
    }
    o_processed_objects_ = output;

    return o_processed_objects_;    
}



// typedef struct {
//         uint32_t            id;
//         ObjectClass         classification;
//         ObjectDynamicState  dynamic_state;
//         std::vector<ReferenceTrajectory> reference;
        
//         ObjectDimension                 dimension;

//         // Multi-modal Prediction
//         std::vector<PredictObjectMultimodal> state_multi;

//         // Highest Probability
//         std::vector<Object3DState>      state;
//     } PredictObject;


// typedef struct {
//         uint32_t            id;
//         ObjectClass         classification;
//         ObjectDynamicState  dynamic_state;

//         ObjectDimension dimension;
//         Object3DState   state;
//         Object3DState   state_covariance;
//     } TrackObject;