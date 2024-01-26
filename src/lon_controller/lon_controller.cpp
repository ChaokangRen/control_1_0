#include "lon_controller.h"

#include <cmath>

#include "lon_controller_interface.h"
#include "sglog/sglog.h"
#include "sgtime/sgtime.h"

using namespace jarvis;
namespace jarvis {
namespace control_lib {
double GRA_ACC = 9.8;

LonControllerInterface *LonControllerInterface::CreateInstance() {
    return new LonController();
}

bool LonController::Init(const ControlConf *control_conf) {
    SG_INFO("LonController Init!");
    control_conf_ = control_conf;
    if (control_conf_ == nullptr) {
        SG_ERROR("get longitudinal param nullptr");
        return false;
    }
    const auto &lon_controller_conf = control_conf_->lon_conf;
    SG_INFO("kp = %lf , ki = %lf", control_conf_->lon_conf.lo_pos_pid_conf.kp,
            control_conf_->lon_conf.lo_pos_pid_conf.ki);

    speed_pid_controller_.Init(lon_controller_conf.lo_pos_pid_conf);
    station_pid_controller_.Init(lon_controller_conf.station_pid_conf);
    speed_state_machine_.Init(lon_controller_conf);

    ts_ = lon_controller_conf.ts;
    lon_controller_initialized_ = true;

    std::vector<double> denominators;
    std::vector<double> numerators;
    LpfCoefficients(ts_, control_conf->lat_conf.cutoff_freq, &denominators,
                    &numerators);
    digital_filter_pitch_angle_.SetCoefficients(denominators, numerators);
    return true;
}

bool LonController::ComputeControlCommand(
    const LocalizationMsg &localization_msg, const SpeedMsg &speed_msg,
    const GearMsg &gear_msg, const TrajectoryMsg &trajectory_msg,
    ControlCmd *control_cmd, std::string &debug_info) {
    // ready to stop
    if (speed_msg.velocity < 0.9 &&
        trajectory_msg.trajectory.front().acceleration < 0.01) {
        // SG_INFO("stop");
        control_cmd->acc_cmd = -0.8;

        debug_info = debug_info +
                     "acc_cmd=" + std::to_string(control_cmd->acc_cmd) + ";" +
                     "vel_ref=" + std::to_string(0.0) + ";" +
                     "station_error=" + std::to_string(0.0) + ";" +
                     "speed_error=" + std::to_string(-speed_msg.velocity) +
                     ";" + "lon0=" + std::to_string(0.0) + ";" +
                     "lon1=" + std::to_string(0.0) + ";" +
                     "lon2=" + std::to_string(0.0) + ";";
        return true;
    }

    double station_error = 0.0;
    double speed_error = 0.0;
    double preview_time = control_conf_->lon_conf.preview_window;

    // 1. Update vheicle status
    vehicle_state_.x = localization_msg.x;
    vehicle_state_.y = localization_msg.y;
    vehicle_state_.heading = localization_msg.yaw_rad;
    vehicle_state_.linear_velocity = speed_msg.velocity;
    vehicle_state_.gear = gear_msg.gear;
    // SG_INFO("x=%lf,y=%lf,yaw = %lf", vehicle_state_.x,
    // vehicle_state_.y,vehicle_state_.heading);

    if (preview_time < 0.0) {
        // SG_ERROR("preview time set as %f,less than 0", preview_time);
        preview_time = 0.01;
    }

    // 2.Calculated longitudinal error and speed error
    TrajecotoryAnalyzer trajectory_analyzer(trajectory_msg);

    ComputeLongitudinalError(trajectory_analyzer, preview_time, &station_error,
                             &speed_error);
    // SG_INFO("station_error=%lf,speed_error=%lf", station_error, speed_error);
    double station_error_limit = control_conf_->lon_conf.station_error_limit;
    double station_error_limited = 0.0;

    // 3. Setting PID Parameters
    if (is_use_preview_time_) {
        station_error_limited =
            Clamp(station_error, -station_error_limit, station_error_limit);
    } else {
        station_error_limited =
            Clamp(station_error, -station_error_limit, station_error_limit);
    }
    if (trajectory_msg.gear == GEAR::R) {
        // empty
    } else {
        PidConf pid_conf;
        bool is_rest = speed_state_machine_.Process(
            vehicle_state_.linear_velocity, speed_error, pid_conf);
        if (is_rest == true) {
            speed_pid_controller_.Reset();
        }
        speed_pid_controller_.SetPID(pid_conf);
        // SG_INFO("kp=%lf,ki=%lf", pid_conf.kp, pid_conf.ki);
    }

    // 4.Calculate the station PID
    if (station_error_limited > 0) {
        station_error_limited = 0;
    }
    double speed_offset =
        station_pid_controller_.Control(station_error_limited, ts_);

    double speed_controller_input = 0.0;
    double speed_controller_input_limit =
        control_conf_->lon_conf.speed_controller_input_limit;
    double speed_controller_input_limited = 0.0;
    // // speed_offset = 0;
    if (speed_offset > 0) {
        speed_offset = 0;
    }
    if (is_use_preview_time_) {
        speed_controller_input = speed_offset + speed_error;
    } else {
        speed_controller_input = speed_offset + speed_error;
    }

    // SG_INFO("speed_offset = %lf,speed_error = %lf", speed_offset,
    // speed_error);

    speed_controller_input_limited =
        Clamp(speed_controller_input, -speed_controller_input_limit,
              speed_controller_input_limit);
    speed_controller_input_limited = speed_controller_input;
    // 5.Calculate the speed PID
    double acceleration_cmd_closeloop = 0.0;
    if (vehicle_state_.linear_velocity < 0.01 &&
        vehicle_state_.linear_velocity > -0.01) {
        speed_pid_controller_.Reset();
        // SG_WARN("Reset Pid");
    }
    // SG_INFO("speed_error_2 = %lf", speed_controller_input_limited);

    acceleration_cmd_closeloop =
        speed_pid_controller_.Control(speed_controller_input_limited, ts_);
    double slope_offset_compensation = 0.0;

    slope_offset_compensation = digital_filter_pitch_angle_.Filter(
        GRA_ACC * std::sin(localization_msg.pitch_rad));
    slope_offset_compensation = GRA_ACC * std::sin(localization_msg.pitch_rad);
    if (std::isnan(slope_offset_compensation)) {
        slope_offset_compensation = 0;
    }
    // first road cross gravity offset
    slope_offset_compensation = 0;
    double qidi_nwy_x = 3901.82;
    double qidi_nwy_y = 2359.51;
    double qidi_nwy_yaw = -3.606;
    if (std::fabs(vehicle_state_.heading - qidi_nwy_yaw) < M_2_PI) {
        double dx = qidi_nwy_x - vehicle_state_.x;
        double dy = qidi_nwy_y - vehicle_state_.y;
        if (dx * dx + dy * dy < 225) {
            slope_offset_compensation = -0.2;
            // SG_WARN("slope_offset_compensation = %lf",
            //         slope_offset_compensation);
        }
    }

    acc_cmd_ = acceleration_cmd_closeloop + slope_offset_compensation;
    // SG_INFO(
    //     "acc_cmd_0 = %lf,acceleration_cmd_closeloop = "
    //     "%lf,slope_offset_compensation = %lf",
    //     acc_cmd_, acceleration_cmd_closeloop, slope_offset_compensation);
    double acc_output_limit = control_conf_->lon_conf.acceleration_output_limit;
    acc_cmd_ = Clamp(acc_cmd_, -acc_output_limit, acc_output_limit);
    // SG_INFO("acc_cmd_1 = %lf,acc_output_limit = %lf", acc_cmd_,
    //         acc_output_limit);
    control_cmd->acc_cmd = acc_cmd_;

    debug_info =
        debug_info + "acc_cmd=" + std::to_string(control_cmd->acc_cmd) + ";" +
        "vel_ref=" + std::to_string(speed_error + speed_msg.velocity) + ";" +
        "station_error=" + std::to_string(station_error) + ";" +
        "speed_error=" + std::to_string(speed_error) + ";" +
        "lon0=" + std::to_string(0.0) + ";" + "lon1=" + std::to_string(0.0) +
        ";" + "lon2=" + std::to_string(0.0) + ";";

    return true;
}

void LonController::ComputeLongitudinalError(
    const TrajecotoryAnalyzer &trajectory_analyzer, const double preview_time,
    double *station_error, double *speed_error) {
    double s_matched = 0.0;
    double s_dot_matched = 0.0;
    double d_matched = 0.0;
    double d_dot_matched = 0.0;

    // 1.Find the closest point on the trajectory to the current position
    auto mathced_point = trajectory_analyzer.QueryMatchedPathPoint(
        vehicle_state_.x, vehicle_state_.y);

    // trajectory_analyzer.ToTrajctoryFrame(
    //     vehicle_state_.x, vehicle_state_.y, vehicle_state_.heading,
    //     vehicle_state_.linear_velocity, mathced_point, &s_matched,
    //     &s_dot_matched, &d_matched, &d_dot_matched);

    double current_control_time = SgTimeUtil::now_secs();
    double preview_control_time = current_control_time + preview_time;
    // SG_INFO("current_control_time=%lf,preview_time=%lf",
    // current_control_time,
    //         preview_time);

    // 2. Find the closest point on the trajectory to the current time
    // TrajectoryPoint refernece_point =
    //     trajectory_analyzer.QueryNearestPointByRelativeTime1(vehicle_state_.x,
    //                                                          vehicle_state_.y);
    s_matched =
        trajectory_analyzer.MatchedS(vehicle_state_.x, vehicle_state_.y);

    // SG_INFO("s_matched = %lf,vel = %lf,preview_time = %lf", s_matched,
    //         vehicle_state_.linear_velocity, preview_time);

    // s_matched += vehicle_state_.linear_velocity * preview_time * 0.1;

    TrajectoryPoint reference_point =
        trajectory_analyzer.QueryNearestPointByAbsoulteTime(
            current_control_time);

    TrajectoryPoint preview_point =
        trajectory_analyzer.QueryNearestPointByAbsoulteTime(
            preview_control_time);

    // SG_INFO("preview_time = %lf", preview_time);

    // SG_INFO("ref x = %lf,y = %lf,s = %lf,t = %lf,speed = %lf,vx = %lf,vy =%lf
    // ",
    //         preview_point.path_point.x, preview_point.path_point.y,
    //         preview_point.path_point.s, preview_point.relative_time,
    //         preview_point.velocity, vehicle_state_.x, vehicle_state_.y);
    // 3. Cacluate station_error and speed error
    is_use_preview_time_ = true;
    if (is_use_preview_time_ == true) {
        *station_error = preview_point.path_point.s - s_matched;
        *speed_error = preview_point.velocity - vehicle_state_.linear_velocity;
        SG_INFO("ref_v = %lf,real_v = %lf", preview_point.velocity,
                vehicle_state_.linear_velocity);
        SG_INFO("preview_point.velocity = %lf", preview_point.velocity);
        SG_INFO("point_s = %lf,match = %lf,station_error = %lf",
                preview_point.path_point.s, s_matched, *station_error);
        // *speed_error = 3 - vehicle_state_.linear_velocity;
    } else {
        *station_error = reference_point.path_point.s - s_matched;
        *speed_error =
            reference_point.velocity - vehicle_state_.linear_velocity;
        // SG_INFO("ref_v = %lf,real_v = %lf", reference_point.velocity,
        //         vehicle_state_.linear_velocity);
        // *speed_error = 3 - vehicle_state_.linear_velocity;
    }
}

bool LonController::Reset() {
    speed_pid_controller_.Reset();
    station_pid_controller_.Reset();
    return true;
}
}  // namespace control_lib
}  // namespace jarvis