#pragma once
#include <sgtime/sgtime.h>
#include <stdint.h>

#include <cmath>
#include <functional>
#include <tuple>
#include <vector>

#include "control_interface.h"
#include "sglog/sglog.h"

namespace jarvis {
namespace control_lib {
// enum GEAR { NONE = 0, P = 1, R = 2, N = 3, D = 4 };

struct LatControllerConf {
    double ts = 0.0;            // sample time(dt) 0.01 sec
    double eps = 0.0;           // converge threshold
    double max_steering;        // max steering value
    double min_steering;        // min steering value
    double max_delta_steering;  // max delta steering value
    double min_delta_steering;  // min delta steering value
    std::vector<double> weights_q;
    std::vector<double> weights_r;
    int32_t cutoff_freq = 0.0;
    int32_t mean_filter_window_size = 0.0;
    double max_lateral_acceleration = 0.0;
    double standstill_acceleration = 0.0;
    int32_t controls_step = 0;
    int32_t horizon = 0;
    double compensation_angle_rad = 0;
};

struct VehicleParams {
    std::string vehicle_id;
    double cf;
    double cr;
    double iz;
    double mass_fl;
    double mass_fr;
    double mass_rl;
    double mass_rr;
    double wheelbase;
    double steer_ratio;
};

struct Constraint {
    std::vector<double> lower_bound;
    std::vector<double> upper_bound;
    std::vector<std::vector<double>> constraint_mat;
};

struct PidConf {
    bool integrator_enable = false;
    double integrator_saturation_level = 0.0;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double kaw = 0.0;
    double output_saturation_level = 0.0;
    double integrator_saturation_high = 0.0;
    double integrator_saturation_low = 0.0;
};

struct FilterConf {
    int32_t cutoff_freq = 0;
};

struct LonControllerConf {
    double ts = 0.0;
    double brake_minimum_action = 0.0;
    double throttle_minimum_action = 0.0;
    double brake_maximum_action = 0.0;
    double throttle_maximum_action = 0.0;
    double speed_controller_input_limit = 0.0;
    double station_error_limit = 0.0;
    double acceleration_output_limit = 0.0;
    double preview_window = 0.0;
    double standstill_acceleration = 0.0;
    double switch_speed = 0.0;

    PidConf station_pid_conf;

    PidConf lo_pos_pid_conf;
    PidConf mi_pos_pid_conf;
    PidConf hi_pos_pid_conf;

    PidConf lo_neg_pid_conf;
    PidConf mi_neg_pid_conf;
    PidConf hi_neg_pid_conf;

    PidConf reverse_station_pid_conf;
    PidConf reverse_speed_filter_conf;
    FilterConf pitch_angle_filter_conf;
};

struct GearStateMachineConf {
    int32_t duration_upper_limit = 0;
    int32_t duration_upper_limit_brake = 0;
    double brake_threshold_value = 0.0;
    double epsilon_v = 0.0;
    double epsilon_a = 0.0;
};

struct ControlConf {
    VehicleParams vehicle_params;
    LatControllerConf lat_conf;
    LonControllerConf lon_conf;
    GearStateMachineConf gear_conf;
};

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double kappa = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double heading = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    double linear_acceleration = 0.0;
    double gear = 0.0;
    double steering_percentage = 0.0;
    double lat_jerk;
    double lon_jerk;
};

// void Wgs84ToLocalCoord(const double longitude, const double latitude,
//                        const double altitude, double *local_x, double
//                        *local_y, double *local_z);

// inline double AngleToRadian(double yaw_degree) {
//     return yaw_degree * 3.14159265358979323846 / 180;
// }

template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
    if (std::abs(t1 - t0) <= 1.0e-6) {
        SG_ERROR("input time difference is too small");
        return x0;
    }
    const double r = (t - t0) / (t1 - t0);
    const T x = x0 + r * (x1 - x0);
    return x;
}

template <typename T>
T Clamp(const T value, T bound1, T bound2) {
    if (bound1 > bound2) {
        std::swap(bound1, bound2);
    }
    if (value < bound1) {
        return bound1;
    } else if (value > bound2) {
        return bound2;
    }
    return value;
}
double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol = 1e-6);

// uint8_t GearCommandValue(GEAR gear);
}  // namespace control_lib
}  // namespace jarvis