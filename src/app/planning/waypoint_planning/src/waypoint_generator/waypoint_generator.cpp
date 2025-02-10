#include <waypoint_generator/waypoint_generator.hpp>

WaypointGenerator::WaypointGenerator() {}

WaypointGenerator::~WaypointGenerator() {}

bool WaypointGenerator::Init(const WaypointPlanningParams& params) {
    // Ini initialization
    std::string dir(getenv("PWD"));
    std::string ini_dir("/config/planning.ini");
    util_ini_parser_.Init((dir + ini_dir).c_str());
    params_= params;

    ProcessINI();
    CheckLaneletMapUpdate();
    CheckSpeedUpdate();

    return true;
}

pair<Trajectory,Trajectory> WaypointGenerator::GenerateWaypoint(VehicleState* vehicle_state, WaypointPlanningParams* params) {
    i_vehicle_state_ = *vehicle_state;
    params_.map_file_path= params->map_file_path;
    ProcessINI();   

    if(CheckLaneletMapUpdate() == false){
        CheckSpeedUpdate();
    }

    return {CalculateROITrajectory(global_race_trajectory_), global_race_trajectory_};
}

bool WaypointGenerator::CheckLaneletMapUpdate(){
    if( params_.map_file_path != prev_params_.map_file_path){
        ROS_INFO_STREAM("New map! : "<< params_.map_file_path);
        prev_params_.map_file_path = params_.map_file_path;
        UpdateLaneletMap();
        UpdateRaceTrajectory();
        return true;
    }  
    else{
        return false;
    }
}

bool WaypointGenerator::CheckSpeedUpdate(){
    if( params_.use_lanelet_speed != prev_params_.use_lanelet_speed || 
        params_.max_speed_kph != prev_params_.max_speed_kph || 
        params_.max_ay_mps2 != prev_params_.max_ay_mps2 ||
        params_.use_backward_forward_smoothing != prev_params_.use_backward_forward_smoothing ||
        params_.max_ax_mps2 != prev_params_.max_ax_mps2 ||
        params_.min_ax_mps2 != prev_params_.min_ax_mps2 ||
        params_.smooth_start_and_stop != prev_params_.smooth_start_and_stop ||
        params_.lookback_index != prev_params_.lookback_index ){
        
        ROS_INFO_STREAM("New Speed! : "<< params_.max_speed_kph);
        
        prev_params_.use_lanelet_speed  = params_.use_lanelet_speed ;
        prev_params_.max_speed_kph  = params_.max_speed_kph ;
        prev_params_.max_ay_mps2    = params_.max_ay_mps2;
        prev_params_.use_backward_forward_smoothing    = params_.use_backward_forward_smoothing;
        prev_params_.max_ax_mps2    = params_.max_ax_mps2;
        prev_params_.min_ax_mps2    = params_.min_ax_mps2;
        prev_params_.smooth_start_and_stop    = params_.smooth_start_and_stop;
        prev_params_.lookback_index    = params_.lookback_index;

        UpdateRaceTrajectory();
        return true;
    }  
    else{
        return false;
    }
}

void WaypointGenerator::UpdateLaneletMap() {    
    // Set map origin
    ll_map_origin_=lanelet::GPSPoint({params_.ref_lat,params_.ref_lon});
    ROS_INFO_STREAM("ref lat/lon <lat :"<<ll_map_origin_.lat << ", lon :"<<ll_map_origin_.lon<<">" );

    // load map file from resources folder
    ll_lanelet_map_ = load(params_.map_file_path, lanelet::projection::LocalCartesianProjector(lanelet::Origin(ll_map_origin_)));
    
    // Routing graph is created from [map, traffic rules, routing cost]
    // Various ways to create routing graph.
    // Default : by distance and time (using the speed limit of each lanelet)
    lanelet::traffic_rules::TrafficRulesPtr trafficRules =
        lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,
                                                            lanelet::Participants::Vehicle);

    lanelet::routing::RoutingGraphUPtr v_routing_graph = lanelet::routing::RoutingGraph::build(*ll_lanelet_map_, *trafficRules);
    
    auto error = v_routing_graph->checkValidity();
    
    // call calculatedCenterline3d() for storing calculated centerline on cache
    for (auto ll : ll_lanelet_map_->laneletLayer) {
        // ll.calculatedCenterline3d();
        ll.centerline3d();
    }
    ROS_INFO_STREAM("Loading Lanelet End!");
    
    lanelet::BasicPoint2d ego_point = {i_vehicle_state_.x, i_vehicle_state_.y}; 
    lanelet::ConstLanelet start_lanelet = FindClosestLanelet(*ll_lanelet_map_, ego_point);
    lanelet::Optional<lanelet::routing::Route> route 
        = v_routing_graph->getRoute(start_lanelet, start_lanelet, 0, true);    
    
    v_reference_lanelet_.clear();
    for(auto fll : route->fullLane(start_lanelet)){
            v_reference_lanelet_.push_back(fll);
    }

    if(v_reference_lanelet_.size() != 1){
        ROS_ERROR_STREAM("Number of lanelets : [" << ll_lanelet_map_->laneletLayer.size() << "]  Only one lanelet must be used!!");
        return;
    }

}

lanelet::Lanelet WaypointGenerator::FindClosestLanelet(lanelet::LaneletMap &map, const lanelet::BasicPoint2d &ego_point) {   
    std::vector<std::pair<double, lanelet::Lanelet>> lanelets 
        = lanelet::geometry::findNearest(map.laneletLayer, ego_point, 1);

    if (lanelets.size() <= 0) {
        ROS_ERROR_STREAM("Can't find closest lanelet!!");
        lanelet::Lanelet empty_lanelet;
        return empty_lanelet;
    }
    return lanelets[0].second;
}


void WaypointGenerator::UpdateRaceTrajectory() {
    Trajectory race_trajectory;
    TrajectoryBoundary right_boundary, left_boundary;

    vector<lanelet::ConstLanelet> reference_lanelets = v_reference_lanelet_;

    float sum_d = 0.0;
    for(auto ll :reference_lanelets){
        
        for (uint16_t i = 0 ; i < ll.centerline3d().size()-1; i++){

            TrajectoryPoint clp;
            clp.x             = (float)ll.centerline3d()[i].basicPoint2d().x();
            clp.y             = (float)ll.centerline3d()[i].basicPoint2d().y();
            clp.z             = 0.0;
            clp.yaw           = atan2(ll.centerline3d()[i+1].basicPoint2d().y() - ll.centerline3d()[i].basicPoint2d().y() , ll.centerline3d()[i+1].basicPoint2d().x() - ll.centerline3d()[i].basicPoint2d().x());
            if ( i != 0){
                float d1 = sqrt(pow(ll.centerline3d()[i].basicPoint2d().x() - ll.centerline3d()[i-1].basicPoint2d().x(),2)+pow(ll.centerline3d()[i].basicPoint2d().y() - ll.centerline3d()[i-1].basicPoint2d().y(),2));
                sum_d += d1;
                float d2 = sqrt(pow(ll.centerline3d()[i+1].basicPoint2d().x() - ll.centerline3d()[i].basicPoint2d().x(),2)+pow(ll.centerline3d()[i+1].basicPoint2d().y() - ll.centerline3d()[i].basicPoint2d().y(),2));
                float d3 = sqrt(pow(ll.centerline3d()[i+1].basicPoint2d().x() - ll.centerline3d()[i-1].basicPoint2d().x(),2)+pow(ll.centerline3d()[i+1].basicPoint2d().y() - ll.centerline3d()[i-1].basicPoint2d().y(),2));
                
                // float area = (ll.centerline3d()[i].basicPoint2d().x() -ll.centerline3d()[i-1].basicPoint2d().x())*(ll.centerline3d()[i+1].basicPoint2d().y() - ll.centerline3d()[i-1].basicPoint2d().y())
                //             -(ll.centerline3d()[i].basicPoint2d().y() -ll.centerline3d()[i-1].basicPoint2d().y())*(ll.centerline3d()[i+1].basicPoint2d().x() - ll.centerline3d()[i-1].basicPoint2d().x());
                float x1 = ll.centerline3d()[i-1].basicPoint2d().x();
                float x2 = ll.centerline3d()[i].basicPoint2d().x();
                float x3 = ll.centerline3d()[i+1].basicPoint2d().x();
                float y1 = ll.centerline3d()[i-1].basicPoint2d().y();
                float y2 = ll.centerline3d()[i].basicPoint2d().y();
                float y3 = ll.centerline3d()[i+1].basicPoint2d().y();
                float area = (1/2.) * fabs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));

                float k = 4*area/(d1*d2*d3);
                clp.curvature =  k;
                clp.distance = sum_d;
            }
            else{
                clp.curvature     = 0.0;
                clp.distance      = 0.0;
            }

            if(params_.use_lanelet_speed == true && ll.centerline3d().front().hasAttribute("vx_mps")){
                clp.speed = ll.centerline3d()[i].attributeOr("vx_mps",0.0);
                if(clp.speed> params_.max_speed_kph*KPH2MPS){
                    clp.speed = params_.max_speed_kph*KPH2MPS;
                }    
            }
            else{
                clp.speed = sqrt(params_.max_ay_mps2 / fabs(clp.curvature));
                if(clp.speed> params_.max_speed_kph*KPH2MPS){
                    clp.speed = params_.max_speed_kph*KPH2MPS;
                }    
            }
            
            clp.time = -1.0;
            clp.acceleration = 0.0;

            race_trajectory.point.push_back(clp);
        }

        for (uint16_t i = 0; i < ll.rightBound3d().size()-1; i++) {
                TrajectoryBoundaryPoint rbp;
                rbp.x = (float)ll.rightBound3d()[i].basicPoint2d().x();
                rbp.y = (float)ll.rightBound3d()[i].basicPoint2d().y();
                rbp.z = 0.0;
                right_boundary.point.push_back(rbp);             
            }

        for (uint16_t i = 0; i < ll.leftBound3d().size()-1; i++) {
            TrajectoryBoundaryPoint lbp;
            lbp.x = (float)ll.leftBound3d()[i].basicPoint2d().x();
            lbp.y = (float)ll.leftBound3d()[i].basicPoint2d().y();
            lbp.z = 0.0;

            left_boundary.point.push_back(lbp);             
        }

        race_trajectory.right_boundary = right_boundary;
        race_trajectory.left_boundary = left_boundary;
    }

    if(params_.smooth_start_and_stop == true){
        for(int i = 0; i<race_trajectory.point.size(); i++ ){
            race_trajectory.point[i].speed = 0.0;
            race_trajectory.point[race_trajectory.point.size()-i].speed = 0.0;
            if(race_trajectory.point[i].distance > 5.0){
                break;
            }
        }
    }

    if(params_.use_backward_forward_smoothing == true){
        double max_acceleration = params_.max_ax_mps2;
        double min_acceleration = params_.min_ax_mps2;

        for (uint16_t i = 0; i < (uint16_t)race_trajectory.point.size(); i++) {
            // Calculate acceleration
            double v = race_trajectory.point.at(i).speed;
            double v_old = i == 0 ? v : race_trajectory.point.at(i-1).speed;
            // double ds = 0.5;
            double ds = i == 0 ? 0.5 : sqrt(pow(race_trajectory.point.at(i).x - race_trajectory.point.at(i-1).x, 2)
                                            + pow(race_trajectory.point.at(i).y - race_trajectory.point.at(i-1).y, 2));
            double a = (pow(v, 2) - pow(v_old, 2)) / (2*ds);

            // Smoothen the speed not to exceed the maximum acceleration
            if (a > max_acceleration) {
            race_trajectory.point.at(i).speed =
                sqrt(pow(v_old, 2) + 2 * max_acceleration*ds);
            }
        }

        // Backward smoothing
        for (uint16_t i = (uint16_t)race_trajectory.point.size()-1; i > 0; i--) {
            // Calculate acceleration
            double v = race_trajectory.point.at(i).speed;
            double v_old = race_trajectory.point.at(i-1).speed;
            double ds = 0.5;
            double a = (pow(v, 2) - pow(v_old, 2)) / (2*ds);

            // Backward smoothing according to minimum acceleration
            if (a < min_acceleration) {  
            race_trajectory.point.at(i-1).speed = sqrt(pow(v, 2) - 2 * min_acceleration*ds ) ;
            }
        }
    }
    
    pcl::PointCloud<pcl::PointXY>::Ptr ptr_reference_waypoints_pcl;
    ptr_reference_waypoints_pcl.reset(new pcl::PointCloud<pcl::PointXY>());

    
    for(auto ref_point : race_trajectory.point){
        pcl::PointXY tmp_point;
        tmp_point.x = ref_point.x;
        tmp_point.y = ref_point.y;
        ptr_reference_waypoints_pcl->points.push_back(tmp_point);
    }

    reference_waypoints_kdtree_.setInputCloud(ptr_reference_waypoints_pcl);

    global_race_trajectory_ = race_trajectory;
}

Trajectory WaypointGenerator::CalculateROITrajectory(const Trajectory& race_trajectory) {
    Trajectory race_trajectory_roi;

    // TODO : ROI wrt distance
    // Find nearest point from Centerline
    vector<int> near_idxes;
    vector<float> near_sqr_dists;
    pcl::PointXY query_position;
    query_position.x = i_vehicle_state_.x;
    query_position.y = i_vehicle_state_.y;
    reference_waypoints_kdtree_.nearestKSearch(query_position, params_.max_overlap_region, near_idxes, near_sqr_dists);
    
    float search_distance = params_.min_search_distance_m + fabs(i_vehicle_state_.vx)*0.02; // min distance + vx * global planning rate
    uint32_t min_index = UINT32_MAX;    
    float min_distance = HUGE_VALF;
    for(uint32_t i = 0; i < near_idxes.size(); i++ ){
        uint32_t target_idx = near_idxes[i];
        float prev_curr_dist = fabs(race_trajectory.point[target_idx].distance - race_trajectory.point[prev_idx_].distance);
        if(prev_curr_dist < search_distance && sqrt(near_sqr_dists[i]) < min_distance){
            min_distance = sqrt(near_sqr_dists[i]);
            min_index = target_idx;
        }
        else{
            continue;
        }
    }

    if(min_index == UINT32_MAX){
        ROS_ERROR_STREAM("Position Jumped Multiple Indices!! Search the Closest!!");
        if(prev_idx_!=0){
            min_index = *min_element(near_idxes.begin(), near_idxes.end());
        }
        else{
            min_index = *max_element(near_idxes.begin(), near_idxes.end());
        }
    } 

    if((int)min_index - params_.lookback_index >= 0){
        min_index -= params_.lookback_index;
    }
    
    
    prev_idx_ = min_index;
    // ROS_WARN_STREAM("dist : "<< race_trajectory.point[min_index].distance << " ind: "<< min_index);

    float sum_dist = 0.0;
    uint32_t prev_index = min_index;
    for(uint16_t i = 0; i < params_.max_iteration; i++){
        if(min_index >= race_trajectory.point.size()){
            min_index = 0;
        }

        float dx = race_trajectory.point[min_index].x -race_trajectory.point[prev_index].x;
        float dy = race_trajectory.point[min_index].y -race_trajectory.point[prev_index].y;

        float prev_curr_dist = sqrt(dx * dx + dy * dy);
        sum_dist += prev_curr_dist;
        // if(sum_dist < params_.min_roi_distance_m){
        if(sum_dist < params_.min_roi_distance_m + fabs(i_vehicle_state_.vx)*params_.trajectory_time_horizon_sec){
            TrajectoryPoint cp;
            cp.x = race_trajectory.point[min_index].x;
            cp.y = race_trajectory.point[min_index].y;
            cp.z = race_trajectory.point[min_index].z;
            cp.yaw = race_trajectory.point[min_index].yaw;
            cp.curvature = race_trajectory.point[min_index].curvature;
            cp.distance = race_trajectory.point[min_index].distance;
            // cp.s= race_trajectory.point[min_index].distance; 
            cp.speed = race_trajectory.point[min_index].speed;
            cp.acceleration = race_trajectory.point[min_index].acceleration;
            cp.time = race_trajectory.point[min_index].time;
            race_trajectory_roi.point.push_back(cp);
        }
        else{
            break;
        }
        prev_index = min_index;
        min_index++;
    
    }

    double t = 0.0;
    double s = 0.;
    for(uint16_t i = 0; i < race_trajectory_roi.point.size(); i ++){

        double dx = i == 0 ? 0.0 : race_trajectory_roi.point[i].x -race_trajectory_roi.point[i-1].x;
        double dy = i == 0 ? 0.0 : race_trajectory_roi.point[i].y -race_trajectory_roi.point[i-1].y;
        s += sqrt(dx * dx + dy * dy);

        race_trajectory_roi.point[i].distance = s;

        if (race_trajectory_roi.point[i].speed > 0.0 || i == 0) {
            if (i != 0) {
                t += sqrt(dx * dx + dy * dy) / race_trajectory_roi.point[i].speed ;
            }
            race_trajectory_roi.point[i].time = t;
        } 
        else {
            t += 0.3;

            race_trajectory_roi.point[i].time = t;
            break;
        }

        // for(uint16_t i = 0 ; i< race_trajectory_roi.point.size(); i++){
        //     if (race_trajectory_roi.point.at(i).speed > params_.max_speed_kph*KPH2MPS){
        //         race_trajectory_roi.point.at(i).speed = params_.max_speed_kph*KPH2MPS;
        //     }
        // }
    }

    CalculateTime(race_trajectory_roi);

    tk::Map road_map = GenerateRoadMap(race_trajectory_roi);

    race_trajectory_roi.left_boundary.point   = CropTrajectoryBoundaryROI(i_vehicle_state_, road_map, race_trajectory.left_boundary.point, v_lb_last_idx_);
    race_trajectory_roi.right_boundary.point  = CropTrajectoryBoundaryROI(i_vehicle_state_, road_map, race_trajectory.right_boundary.point, v_rb_last_idx_);
    


    return race_trajectory_roi;

}


vector<TrajectoryBoundaryPoint> WaypointGenerator::CropTrajectoryBoundaryROI(const VehicleState& vehicle_state,
                                                            tk::Map& road_map,
                                                            const vector<TrajectoryBoundaryPoint> &tb, 
                                                            uint16_t &prev_closest_idx){
    // Find nearest point from Boundary
    deque<TrajectoryBoundaryPoint> tb_roi;
    uint16_t min_index = 0;
    float min_distance = HUGE_VALF;
    
    uint16_t target_idx = 0;

    if((uint16_t)prev_closest_idx > (uint16_t)tb.size()){
        target_idx = (uint16_t)tb.size()-1;
    }
    else{
        target_idx = (uint16_t)prev_closest_idx;
    }

    float dx = vehicle_state.x - tb.at(target_idx).x;
    float dy = vehicle_state.y - tb.at(target_idx).y;

    float dist = sqrt(pow(dx,2)+pow(dy,2));

    uint16_t max_iteration = 0;
    if (dist > params_.min_search_distance_m) {
        max_iteration = tb.size();
    } 
    else {
        // if (tb.size() > params_.max_iteration) {
        //     max_iteration = params_.max_iteration;
        // } 
        // else {
        //     max_iteration = tb.size();
        // }

        max_iteration = params_.max_overlap_region;
    }

    

    uint16_t sgn_cnt = 0;
    for (uint16_t i = 0; i < max_iteration; i++) {
        int focusing_idx;
        if(i%2==0){
            focusing_idx = (int)(prev_closest_idx-sgn_cnt);
            sgn_cnt++;
        }
        else{
            focusing_idx = (int)(prev_closest_idx+sgn_cnt);
        }                
        if(focusing_idx < 0){
            focusing_idx += tb.size();
        }
        if(focusing_idx >= tb.size()){
            focusing_idx -= tb.size();
        }

        float dx = tb[focusing_idx].x - vehicle_state.x;
        float dy = tb[focusing_idx].y - vehicle_state.y;
        float dist = sqrt(dx * dx + dy * dy);

        if (dist < min_distance) {
            min_distance = dist;
            min_index = focusing_idx;
        }
    }

    if(min_distance > params_.min_roi_distance_m){
        for (uint16_t i = 0; i < (uint16_t)tb.size(); i++) {
            float dx = tb[i].x - vehicle_state.x;
            float dy = tb[i].y - vehicle_state.y;

            float dist = sqrt(dx * dx + dy * dy);

            if (dist < min_distance) {
                min_distance = dist;
                min_index = i;
            }
        }
    }

    prev_closest_idx = min_index;
    
    sgn_cnt=0;
    for(uint16_t i = 0; i < params_.max_iteration; i++){
        int focusing_idx;
        if(i%2==0){
            focusing_idx = (int)(min_index-sgn_cnt);
            sgn_cnt++;
        }
        else{
            focusing_idx = (int)(min_index+sgn_cnt);
        }                

        if(focusing_idx < 0){
            focusing_idx += tb.size();
        }
        if(focusing_idx >= tb.size()){
            focusing_idx -= tb.size();
        }

        float dx = tb[focusing_idx].x - tb[min_index].x;
        float dy = tb[focusing_idx].y - tb[min_index].y;
        float dist = sqrt(dx * dx + dy * dy);

        if(dist < params_.min_roi_distance_m){
            TrajectoryBoundaryPoint tbp;
            tbp.x = tb[focusing_idx].x;
            tbp.y = tb[focusing_idx].y;
            tbp.z = tb[focusing_idx].z;
            std::vector<double> tbp_sn = road_map.ToFrenet(tbp.x, tbp.y);
            if(tbp_sn[0]>params_.min_roi_distance_m){
                break;
            }
            if(i % 2 != 0)
                tb_roi.push_back(tbp);
            else    
                tb_roi.push_front(tbp);
        }
        else{
            // if map is not closed loop, we need to look forward
            // if(i % 2 == 0){
                break;
            // }        
        }
    }
    tb_roi.erase(tb_roi.begin(), tb_roi.begin() + tb_roi.size() / 2 - params_.lookback_index);

    std::vector<TrajectoryBoundaryPoint> v_tb_roi(std::make_move_iterator(tb_roi.begin()),
                             std::make_move_iterator(tb_roi.end()));

    return v_tb_roi;
}



void WaypointGenerator::CalculateTime(Trajectory& race_trajectory){
  
  float tp_time = 0.0;
  race_trajectory.point.front().time = 0.0;
  for(uint32_t i = 1; i< race_trajectory.point.size(); i++){
    race_trajectory.point.at(i).time = race_trajectory.point.at(i-1).time +
                                      2*(race_trajectory.point.at(i).distance-race_trajectory.point.at(i-1).distance)
                                       /(race_trajectory.point.at(i-1).speed + race_trajectory.point.at(i).speed) ;
    
    double v = race_trajectory.point.at(i).speed;
    double v_old = i == 0 ? v : race_trajectory.point.at(i-1).speed;
    race_trajectory.point.at(i).acceleration = (pow(v, 2) - pow(v_old, 2)) / (2);

  }
}


void WaypointGenerator::ProcessINI() {
    if (util_ini_parser_.IsFileUpdated()) { 
        util_ini_parser_.ParseConfig("Waypoint planning", "global_time_horizon", params_.trajectory_time_horizon_sec);
        util_ini_parser_.ParseConfig("Waypoint planning", "min_search_distance", params_.min_search_distance_m);
        util_ini_parser_.ParseConfig("Waypoint planning", "min_roi_distance", params_.min_roi_distance_m);  
        util_ini_parser_.ParseConfig("Waypoint planning", "max_iteration", params_.max_iteration); 
        util_ini_parser_.ParseConfig("Waypoint planning", "use_lanelet_speed", params_.use_lanelet_speed); 
        util_ini_parser_.ParseConfig("Waypoint planning", "max_speed", params_.max_speed_kph); 
        util_ini_parser_.ParseConfig("Waypoint planning", "min_ax", params_.min_ax_mps2); 
        util_ini_parser_.ParseConfig("Waypoint planning", "use_backward_forward_smoothing", params_.use_backward_forward_smoothing); 
        util_ini_parser_.ParseConfig("Waypoint planning", "max_ax", params_.max_ax_mps2); 
        util_ini_parser_.ParseConfig("Waypoint planning", "max_ay", params_.max_ay_mps2); 
        util_ini_parser_.ParseConfig("Waypoint planning", "max_overlap_region", params_.max_overlap_region); 
        util_ini_parser_.ParseConfig("Waypoint planning", "smooth_start_and_stop", params_.smooth_start_and_stop); 
        util_ini_parser_.ParseConfig("Waypoint planning", "lookback_index", params_.lookback_index); 
        ROS_WARN("[Waypoint planning] Ini file is updated!");
    }
}
