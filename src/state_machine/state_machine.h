#pragma once
#include "common.h"
namespace jarvis {
namespace control_lib {
class StateMachine {
public:
    StateMachine() = default;

    void Init(const ControlConf *control_conf);

    bool Execute(const SpeedMsg &speed_msg, const GearMsg &gear_msg,
                 const TrajectoryMsg &trajectory_msg, ControlCmd *control_cmd,
                 std::string &control_info);

    double Brake() {
        return brake_cmd_value_;
    }

    GEAR Gear() {
        return gear_;
    }

private:
    void SetGear(GEAR gear);

    void SetBrake(ControlCmd *control_cmd);

    void Clear();

private:
    int32_t p_to_r_duration_count_ = 0;
    int32_t p_to_d_duration_count_ = 0;
    int32_t r_to_p_duration_count_ = 0;
    int32_t d_to_p_duration_count_ = 0;
    int32_t brake_duration_count_ = 0;

    int32_t duration_upper_limit_ = 50;
    int32_t duration_upper_limit_brake_ = 50;
    double brake_threshold_value_ = 30;

    double brake_cmd_value_ = 0;

    double epsilon_v_ = 0.01;
    double epsilon_a_ = 0.005;

    const double standstill_velocity_ = 0.3;

    GEAR gear_;
    GEAR chassis_gear_pre_;
};
}  // namespace control_lib
}  // namespace jarvis