#include "scenario_parser/scenario_parser_algorithm.hpp"

ScenarioParser::ScenarioParser() {}

ScenarioParser::~ScenarioParser() {}

bool ScenarioParser::Init(const VirtualObjectGeneratorParams& params) {
    for ( const auto& path : params.csv_path ) {
        std::vector<std::vector<double>> csv_data = ReadCSV(path);

        if ( !isCSVDataValid(csv_data) ) {
            std::cout << "CSV Data is invalid!!" << std::endl;
            return false;
        }
        i_csv_data_.push_back(csv_data);
    }

    for ( const auto& path : params.csv_noise_path ) {

        ROS_WARN("BEFORE NOISE READ");
        std::vector<std::vector<double>> csv_noise_data = ReadCSV(path);
        ROS_WARN("AFTER NOISE READ");

        if ( !isCSVDataValid(csv_noise_data) ) {
            std::cout << "CSV Data is invalid!!" << std::endl;
            return false;
        }
        i_csv_noise_data_.push_back(csv_noise_data);
    }

    time_elapsed_ = 0;

    return true;
}

TrackObjects ScenarioParser::ParseScenario(const VehicleState& vehicle_state, const vector<double>& ego_frenet,
                                           tk::Map& road_map, tk::spline& left_boundary,
                                           tk::spline& right_boundary, const VirtualObjectGeneratorParams& params) {
    TrackObjects virtual_objects;
    virtual_objects.header.stamp = vehicle_state.header.stamp;

    if ( (GenerateMode)params.generate_mode == GenerateMode::OFF ) {
        time_elapsed_ = 0;
        return virtual_objects;
    }
    else if ( (GenerateMode)params.generate_mode == GenerateMode::INIT ) {
        time_elapsed_ = 0;
    }

    int  id      = 0;
    bool is_stop = true;

    // for (const auto& csv_data : i_csv_data_) {
    //     TrackObject virtual_object;

    //     // Make mode with INI

    //     if (params.noise_mode == 1) {
    //         virtual_object = GetVirtualObjectWithNoise(csv_data, time_elapsed_, params);
    //     }
    //     else {
    //         virtual_object = GetVirtualObjectWithTrackingNoise(csv_data, time_elapsed_, params, i_csv_noise_data_);
    //     }

    //     virtual_object.id = id++;
    //     virtual_object.state.time_stamp = vehicle_state.time_stamp;

    //     std::vector<double> object_frenet = road_map.ToFrenet(virtual_object.state.x, virtual_object.state.y);
    //     if (object_frenet.at(0) < ego_frenet.at(0) + params.roi_s_front*2.0) {
    //         is_stop = false;
    //     }
    //     virtual_objects.object.push_back(virtual_object);
    // }
    // util_function::DebugPrintInfo(
    //     "Size of CSV data : " + std::to_string(i_csv_data_.size())
    // );
    for ( int i = 0; i < i_csv_data_.size(); i++ ) {
        TrackObject virtual_object;
        // util_function::DebugPrintInfo(
        //     "i : " + std::to_string(i)
        // );
        // Make mode with INI

        // if ( params.noise_mode == 1 ) {
        virtual_object = GetVirtualObjectWithNoise(i_csv_data_.at(i), time_elapsed_, params);
        // }
        // else {
        //     virtual_object = GetVirtualObjectWithTrackingNoise(i_csv_data_.at(i), time_elapsed_, params, i_csv_noise_data_.at(i));
        // }

        virtual_object.id               = id++;
        virtual_object.state.header.stamp = vehicle_state.header.stamp;

        std::vector<double> object_frenet = road_map.ToFrenet(virtual_object.state.x, virtual_object.state.y);
        if ( object_frenet.at(0) < ego_frenet.at(0) + params.roi_s_front * 2.0 ) {
            is_stop = false;
        }
        virtual_objects.object.push_back(virtual_object);
    }

    // Auto stop
    if ( (GenerateMode)params.generate_mode == GenerateMode::START ) {
        if ( is_stop == false ) {
            time_elapsed_++;
        }
    }

    return virtual_objects;
}

TrackObject ScenarioParser::GetVirtualObjectWithNoise(const std::vector<std::vector<double>>& csv_data, const int& index,
                                                      const VirtualObjectGeneratorParams& params) {
    TrackObject front_vehicle;

    front_vehicle.classification = ObjectClass::CAR;

    front_vehicle.dimension.length = VEHICLE_LENGTH;
    front_vehicle.dimension.width  = VEHICLE_WIDTH;
    front_vehicle.dimension.height = VEHICLE_HEIGHT;

    int process_index = index % csv_data.size();

    front_vehicle.state.x   = csv_data[process_index][2] + GenerateGaussianNoise(0.0, params.noise_x);
    front_vehicle.state.y   = csv_data[process_index][3] + GenerateGaussianNoise(0.0, params.noise_y);
    front_vehicle.state.yaw = csv_data[process_index][4] + GenerateGaussianNoise(0.0, params.noise_yaw);
    front_vehicle.state.v_x = csv_data[process_index][6] * cos(csv_data[process_index][4]) + GenerateGaussianNoise(0.0, params.noise_v_x);
    front_vehicle.state.v_y = csv_data[process_index][6] * sin(csv_data[process_index][4]);
    front_vehicle.state.a_x = csv_data[process_index][7] * cos(csv_data[process_index][4]) + GenerateGaussianNoise(0.0, params.noise_a_x);
    front_vehicle.state.a_y = csv_data[process_index][7] * sin(csv_data[process_index][4]);

    if ( sqrt(pow(front_vehicle.state.v_x, 2) + pow(front_vehicle.state.v_y, 2)) < 1.0 ) {
        front_vehicle.dynamic_state = ObjectDynamicState::STATIC;
    }
    else {
        front_vehicle.dynamic_state = ObjectDynamicState::DYNAMIC;
    }

    return front_vehicle;
}

TrackObject ScenarioParser::GetVirtualObjectWithTrackingNoise(const std::vector<std::vector<double>>& csv_data, const int& index,
                                                              const VirtualObjectGeneratorParams&     params,
                                                              const std::vector<std::vector<double>>& noise_csv_data) {
    TrackObject front_vehicle;

    front_vehicle.classification = ObjectClass::CAR;

    front_vehicle.dimension.length = VEHICLE_LENGTH;
    front_vehicle.dimension.width  = VEHICLE_WIDTH;
    front_vehicle.dimension.height = VEHICLE_HEIGHT;

    int process_index = index % csv_data.size();
    int noise_index = index % noise_csv_data.size();

    front_vehicle.state.x   = csv_data[process_index][2] + noise_csv_data[noise_index][1];  
    front_vehicle.state.y   = csv_data[process_index][3] + noise_csv_data[noise_index][2];
    front_vehicle.state.yaw = csv_data[process_index][4] + 0.0174533 * noise_csv_data[noise_index][7];
    front_vehicle.state.v_x = csv_data[process_index][6] * cos(csv_data[process_index][4]) + noise_csv_data[noise_index][3];
    front_vehicle.state.v_y = csv_data[process_index][6] * sin(csv_data[process_index][4]) + noise_csv_data[noise_index][4];
    front_vehicle.state.a_x = csv_data[process_index][7] * cos(csv_data[process_index][4]) + noise_csv_data[noise_index][5];
    front_vehicle.state.a_y = csv_data[process_index][7] * sin(csv_data[process_index][4]) + noise_csv_data[noise_index][6];

    if ( sqrt(pow(front_vehicle.state.v_x, 2) + pow(front_vehicle.state.v_y, 2)) < 2.0 ) {
        front_vehicle.dynamic_state = ObjectDynamicState::STATIC;
    }
    else {
        front_vehicle.dynamic_state = ObjectDynamicState::DYNAMIC;
    }

    return front_vehicle;
}

double ScenarioParser::GenerateGaussianNoise(double mean, double std_dev) {
    // This function is for generating gaussian noise
    static std::random_device        rd;
    std::default_random_engine       gen(rd());
    std::normal_distribution<double> dist(mean, std_dev);

    return dist(gen);
}

std::vector<std::vector<double>> ScenarioParser::ReadCSV(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream                    file(filename);
    char                             delimiter = ',';

    if ( !file ) {
        std::cerr << "Cannot Open File : " << filename << std::endl;
        return data;
    }

    std::cerr << "Opened File : " << filename << std::endl;

    std::string line;

    // Skip the first line
    std::getline(file, line);

    // Process the rest of the lines
    while ( std::getline(file, line) ) {
        std::vector<double> row;
        std::istringstream  iss(line);
        std::string         field;

        while ( std::getline(iss, field, delimiter) ) {
            row.push_back(std::stod(field));
        }

        data.push_back(row);
    }

    file.close();
    return data;
}

bool ScenarioParser::isCSVDataValid(const std::vector<std::vector<double>>& csv_data) {
    for ( const auto& row : csv_data ) {
        for ( const auto& value : row ) {
            if ( std::isnan(value) ) {
                return false;
            }
        }
    }
    return true;
}