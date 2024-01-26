#include "json_parser.h"

#include <fstream>

#include "sglog/sglog.h"

namespace jarvis {
namespace control_lib {
bool JsonParser::ParserVehicleConf(const std::string &path) {
    Json::Reader reader;
    Json::Value root;

    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        SG_ERROR("vehicle_parameter file open failed!");
    }
    if (reader.parse(in, root)) {
        vehicle_params_.vehicle_id = root["vehicle_id"].asString();
        vehicle_params_.cf = root["cf"].asDouble();
        vehicle_params_.cr = root["cr"].asDouble();
        vehicle_params_.iz = root["iz"].asDouble();
        vehicle_params_.mass_fl = root["mass_fl"].asDouble();
        vehicle_params_.mass_fr = root["mass_fr"].asDouble();
        vehicle_params_.mass_rl = root["mass_rl"].asDouble();
        vehicle_params_.mass_rr = root["mass_rr"].asDouble();
        vehicle_params_.wheelbase = root["wheelbase"].asDouble();
        vehicle_params_.steer_ratio = root["steer_ratio"].asDouble();
    }

    return true;
}

bool JsonParser::ParserLatConf(const std::string &path) {
    Json::Reader reader;
    Json::Value root;

    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        SG_ERROR("lat_controller_conf file open failed!");
    }

    if (reader.parse(in, root)) {
        lat_conf_.ts = root["ts"].asDouble();
        lat_conf_.eps = root["eps"].asDouble();
        lat_conf_.max_steering = root["max_steering"].asDouble();
        lat_conf_.min_steering = root["min_steering"].asDouble();
        lat_conf_.max_delta_steering = root["max_delta_steering"].asDouble();
        lat_conf_.min_delta_steering = root["min_delta_steering"].asDouble();
        for (int i = 0; i < root["weights_q"].size(); ++i) {
            lat_conf_.weights_q.emplace_back(root["weights_q"][i].asDouble());
        }
        for (int i = 0; i < root["weights_r"].size(); ++i) {
            lat_conf_.weights_r.emplace_back(root["weights_r"][i].asDouble());
        }
        lat_conf_.cutoff_freq = root["cutoff_freq"].asInt();
        lat_conf_.mean_filter_window_size =
            root["mean_filter_window_size"].asInt();
        lat_conf_.max_lateral_acceleration =
            root["max_lateral_acceleration"].asDouble();
        lat_conf_.standstill_acceleration =
            root["standstill_acceleration"].asDouble();
        lat_conf_.controls_step = root["controls_step"].asInt();
        lat_conf_.horizon = root["horizon"].asInt();
        lat_conf_.compensation_angle_rad =
            root["compensation_angle_rad"].asDouble();
    }
    return true;
}

bool JsonParser::ParserLonConf(const std::string &path) {
    Json::Reader reader;
    Json::Value root;

    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        SG_ERROR("lon_controller_conf file open failed!");
    }

    if (reader.parse(in, root)) {
        lon_conf_.ts = root["ts"].asDouble();
        lon_conf_.brake_minimum_action =
            root["brake_minimum_action"].asDouble();

        lon_conf_.throttle_minimum_action =
            root["brake_minimum_action"].asDouble();
        lon_conf_.brake_maximum_action =
            root["brake_maximum_action"].asDouble();
        lon_conf_.throttle_maximum_action =
            root["brake_maximum_action"].asDouble();

        lon_conf_.speed_controller_input_limit =
            root["speed_controller_input_limit"].asDouble();
        lon_conf_.station_error_limit = root["station_error_limit"].asDouble();
        lon_conf_.preview_window = root["preview_window"].asDouble();
        lon_conf_.standstill_acceleration =
            root["standstill_acceleration"].asDouble();
        lon_conf_.switch_speed = root["switch_speed"].asDouble();

        lon_conf_.acceleration_output_limit =
            root["acceleration_output_limit"].asDouble();

        lon_conf_.station_pid_conf.integrator_enable =
            root["station_pid_conf"]["integrator_enable"].asBool();
        lon_conf_.station_pid_conf.integrator_saturation_level =
            root["station_pid_conf"]["integrator_saturation_level"].asDouble();
        lon_conf_.station_pid_conf.kp =
            root["station_pid_conf"]["kp"].asDouble();
        lon_conf_.station_pid_conf.ki =
            root["station_pid_conf"]["ki"].asDouble();
        lon_conf_.station_pid_conf.kd =
            root["station_pid_conf"]["kd"].asDouble();
        lon_conf_.station_pid_conf.kaw =
            root["station_pid_conf"]["kaw"].asDouble();
        lon_conf_.station_pid_conf.output_saturation_level =
            root["station_pid_conf"]["output_saturation_level"].asDouble();
        lon_conf_.station_pid_conf.integrator_saturation_high =
            root["station_pid_conf"]["integrator_saturation_high"].asDouble();
        lon_conf_.station_pid_conf.integrator_saturation_low =
            root["station_pid_conf"]["integrator_saturation_low"].asDouble();

        lon_conf_.lo_pos_pid_conf.integrator_enable =
            root["lo_pos_pid_conf"]["integrator_enable"].asBool();
        lon_conf_.lo_pos_pid_conf.integrator_saturation_level =
            root["lo_pos_pid_conf"]["integrator_saturation_level"].asDouble();
        lon_conf_.lo_pos_pid_conf.kp = root["lo_pos_pid_conf"]["kp"].asDouble();
        lon_conf_.lo_pos_pid_conf.ki = root["lo_pos_pid_conf"]["ki"].asDouble();
        lon_conf_.lo_pos_pid_conf.kd = root["lo_pos_pid_conf"]["kd"].asDouble();
        lon_conf_.lo_pos_pid_conf.kaw =
            root["lo_pos_pid_conf"]["kaw"].asDouble();
        lon_conf_.lo_pos_pid_conf.output_saturation_level =
            root["lo_pos_pid_conf"]["output_saturation_level"].asDouble();
        lon_conf_.lo_pos_pid_conf.integrator_saturation_high =
            root["station_pid_conf"]["integrator_saturation_high"].asDouble();
        lon_conf_.lo_pos_pid_conf.integrator_saturation_low =
            root["station_pid_conf"]["integrator_saturation_low"].asDouble();

        lon_conf_.mi_pos_pid_conf.integrator_enable =
            root["mi_pos_pid_conf"]["integrator_enable"].asBool();
        lon_conf_.mi_pos_pid_conf.integrator_saturation_level =
            root["mi_pos_pid_conf"]["integrator_saturation_level"].asDouble();
        lon_conf_.mi_pos_pid_conf.kp = root["mi_pos_pid_conf"]["kp"].asDouble();
        lon_conf_.mi_pos_pid_conf.ki = root["mi_pos_pid_conf"]["ki"].asDouble();
        lon_conf_.mi_pos_pid_conf.kd = root["mi_pos_pid_conf"]["kd"].asDouble();
        lon_conf_.mi_pos_pid_conf.kaw =
            root["mi_pos_pid_conf"]["kaw"].asDouble();
        lon_conf_.mi_pos_pid_conf.output_saturation_level =
            root["mi_pos_pid_conf"]["output_saturation_level"].asDouble();
        lon_conf_.mi_pos_pid_conf.integrator_saturation_high =
            root["station_pid_conf"]["integrator_saturation_high"].asDouble();
        lon_conf_.mi_pos_pid_conf.integrator_saturation_low =
            root["station_pid_conf"]["integrator_saturation_low"].asDouble();

        lon_conf_.hi_pos_pid_conf.integrator_enable =
            root["hi_pos_pid_conf"]["integrator_enable"].asBool();
        lon_conf_.hi_pos_pid_conf.integrator_saturation_level =
            root["hi_pos_pid_conf"]["integrator_saturation_level"].asDouble();
        lon_conf_.hi_pos_pid_conf.kp = root["hi_pos_pid_conf"]["kp"].asDouble();
        lon_conf_.hi_pos_pid_conf.ki = root["hi_pos_pid_conf"]["ki"].asDouble();
        lon_conf_.hi_pos_pid_conf.kd = root["hi_pos_pid_conf"]["kd"].asDouble();
        lon_conf_.hi_pos_pid_conf.kaw =
            root["hi_pos_pid_conf"]["kaw"].asDouble();
        lon_conf_.hi_pos_pid_conf.output_saturation_level =
            root["hi_pos_pid_conf"]["output_saturation_level"].asDouble();
        lon_conf_.hi_pos_pid_conf.integrator_saturation_high =
            root["station_pid_conf"]["integrator_saturation_high"].asDouble();
        lon_conf_.hi_pos_pid_conf.integrator_saturation_low =
            root["station_pid_conf"]["integrator_saturation_low"].asDouble();

        lon_conf_.lo_neg_pid_conf.integrator_enable =
            root["lo_neg_pid_conf"]["integrator_enable"].asBool();
        lon_conf_.lo_neg_pid_conf.integrator_saturation_level =
            root["lo_neg_pid_conf"]["integrator_saturation_level"].asDouble();
        lon_conf_.lo_neg_pid_conf.kp = root["lo_neg_pid_conf"]["kp"].asDouble();
        lon_conf_.lo_neg_pid_conf.ki = root["lo_neg_pid_conf"]["ki"].asDouble();
        lon_conf_.lo_neg_pid_conf.kd = root["lo_neg_pid_conf"]["kd"].asDouble();
        lon_conf_.lo_neg_pid_conf.kaw =
            root["lo_neg_pid_conf"]["kaw"].asDouble();
        lon_conf_.lo_neg_pid_conf.output_saturation_level =
            root["lo_neg_pid_conf"]["output_saturation_level"].asDouble();
        lon_conf_.lo_neg_pid_conf.integrator_saturation_high =
            root["station_pid_conf"]["integrator_saturation_high"].asDouble();
        lon_conf_.lo_neg_pid_conf.integrator_saturation_low =
            root["station_pid_conf"]["integrator_saturation_low"].asDouble();

        lon_conf_.mi_neg_pid_conf.integrator_enable =
            root["mi_neg_pid_conf"]["integrator_enable"].asBool();
        lon_conf_.mi_neg_pid_conf.integrator_saturation_level =
            root["mi_neg_pid_conf"]["integrator_saturation_level"].asDouble();
        lon_conf_.mi_neg_pid_conf.kp = root["mi_neg_pid_conf"]["kp"].asDouble();
        lon_conf_.mi_neg_pid_conf.ki = root["mi_neg_pid_conf"]["ki"].asDouble();
        lon_conf_.mi_neg_pid_conf.kd = root["mi_neg_pid_conf"]["kd"].asDouble();
        lon_conf_.mi_neg_pid_conf.kaw =
            root["mi_neg_pid_conf"]["kaw"].asDouble();
        lon_conf_.mi_neg_pid_conf.output_saturation_level =
            root["mi_neg_pid_conf"]["output_saturation_level"].asDouble();
        lon_conf_.mi_neg_pid_conf.integrator_saturation_high =
            root["station_pid_conf"]["integrator_saturation_high"].asDouble();
        lon_conf_.mi_neg_pid_conf.integrator_saturation_low =
            root["station_pid_conf"]["integrator_saturation_low"].asDouble();

        lon_conf_.hi_neg_pid_conf.integrator_enable =
            root["hi_neg_pid_conf"]["integrator_enable"].asBool();
        lon_conf_.hi_neg_pid_conf.integrator_saturation_level =
            root["hi_neg_pid_conf"]["integrator_saturation_level"].asDouble();
        lon_conf_.hi_neg_pid_conf.kp = root["hi_neg_pid_conf"]["kp"].asDouble();
        lon_conf_.hi_neg_pid_conf.ki = root["hi_neg_pid_conf"]["ki"].asDouble();
        lon_conf_.hi_neg_pid_conf.kd = root["hi_neg_pid_conf"]["kd"].asDouble();
        lon_conf_.hi_neg_pid_conf.kaw =
            root["hi_neg_pid_conf"]["kaw"].asDouble();
        lon_conf_.hi_neg_pid_conf.output_saturation_level =
            root["hi_neg_pid_conf"]["output_saturation_level"].asDouble();
        lon_conf_.hi_neg_pid_conf.integrator_saturation_high =
            root["station_pid_conf"]["integrator_saturation_high"].asDouble();
        lon_conf_.hi_neg_pid_conf.integrator_saturation_low =
            root["station_pid_conf"]["integrator_saturation_low"].asDouble();
    }
    return true;
}
bool JsonParser::ParserGearConf(const std::string &path) {
    Json::Reader reader;
    Json::Value root;

    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        SG_ERROR("gear_state_machine_conf file open failed!");
    }

    if (reader.parse(in, root)) {
        gear_conf_.duration_upper_limit = root["duration_upper_limit"].asInt();
        gear_conf_.duration_upper_limit_brake =
            root["duration_upper_limit_brake"].asInt();
        gear_conf_.brake_threshold_value =
            root["brake_threshold_value"].asDouble();
        gear_conf_.epsilon_v = root["epsilon_v"].asDouble();
        gear_conf_.epsilon_a = root["epsilon_a"].asDouble();
    }
    return true;
}
}  // namespace control_lib
}  // namespace jarvis