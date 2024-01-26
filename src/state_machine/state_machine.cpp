#include "state_machine.h"

#include <jsoncpp/json/json.h>

#include "sglog/sglog.h"

namespace jarvis {
namespace control_lib {
void StateMachine::Init(const ControlConf *control_conf) {
    duration_upper_limit_ = control_conf->gear_conf.duration_upper_limit;
    // brake when changing gear
    duration_upper_limit_ = 0;
    duration_upper_limit_brake_ =
        control_conf->gear_conf.duration_upper_limit_brake;
    duration_upper_limit_brake_ = 0;
    brake_threshold_value_ = control_conf->gear_conf.brake_threshold_value;
    epsilon_v_ = control_conf->gear_conf.epsilon_v;
    epsilon_a_ = control_conf->gear_conf.epsilon_a;
}

bool StateMachine::Execute(const SpeedMsg &speed_msg, const GearMsg &gear_msg,
                           const TrajectoryMsg &trajectory_msg,
                           ControlCmd *control_cmd, std::string &debug_info) {
    // gear none when changing gear
    if (!gear_msg.gear == GEAR::NONE) {
        gear_ = gear_msg.gear;
    }

    double velocity = speed_msg.velocity;
    double desire_acc = trajectory_msg.trajectory.front().acceleration;

    if (gear_ != chassis_gear_pre_) {
        Clear();
    }

    chassis_gear_pre_ = gear_;

    if (gear_ == GEAR::P) {
        if (velocity <= epsilon_v_ && velocity >= -epsilon_v_) {
            if (desire_acc < -epsilon_a_) {
                // set r gear
                ++p_to_r_duration_count_;
                if (p_to_r_duration_count_ > duration_upper_limit_) {
                    SetGear(GEAR::R);
                }

            } else if (desire_acc > epsilon_a_) {
                // set d gear
                ++p_to_d_duration_count_;
                if (p_to_d_duration_count_ > duration_upper_limit_) {
                    SetGear(GEAR::D);
                }
            } else if (desire_acc > -epsilon_a_ && desire_acc < epsilon_a_) {
                // do nothing
            }
        } else if (velocity > epsilon_v_) {
            // SG_ERROR("Abnormal velocity in P gear(positive)");
        } else if (velocity < -epsilon_v_) {
            // SG_ERROR("Abnormal velocity in P gear(negative)");
        }
    } else if (gear_ == GEAR::D) {
        if (velocity <= epsilon_v_ && velocity >= -epsilon_v_) {
            // SG_INFO("desire_acc=%lf", desire_acc);
            if (desire_acc < epsilon_a_) {
                ++d_to_p_duration_count_;
                ++brake_duration_count_;
                if (d_to_p_duration_count_ > duration_upper_limit_) {
                    SetBrake(control_cmd);
                }
                if (d_to_p_duration_count_ > duration_upper_limit_ &&
                    brake_duration_count_ > duration_upper_limit_brake_) {
                    SetGear(GEAR::D);
                }
            } else if (desire_acc > epsilon_a_) {
            }
        } else if (velocity > epsilon_v_) {
        } else if (velocity < -epsilon_v_) {
            // SG_ERROR("Abnormal velocity in D gear");
        }
    } else if (gear_ == GEAR::R) {
        if (velocity <= epsilon_v_ && velocity >= -epsilon_v_) {
            if (desire_acc < -epsilon_a_) {
                // do nothing
            } else if (desire_acc > -epsilon_a_) {
                ++r_to_p_duration_count_;
                ++brake_duration_count_;
                if (r_to_p_duration_count_ > duration_upper_limit_) {
                    SetBrake(control_cmd);
                }
                if (r_to_p_duration_count_ > duration_upper_limit_ &&
                    brake_duration_count_ > duration_upper_limit_brake_) {
                    SetGear(GEAR::D);
                }
            }
        } else if (velocity > epsilon_v_) {
            // SG_ERROR("Abnormal velocity in D gear");
        } else if (velocity < -epsilon_v_) {
            // do nothing
        }
    } else if (gear_ == GEAR::N) {
        ++brake_duration_count_;
        SetBrake(control_cmd);
        if (brake_duration_count_ > duration_upper_limit_brake_) {
            if (desire_acc < -epsilon_a_) {
                // set r gear
                SetGear(GEAR::R);

            } else if (desire_acc > epsilon_a_) {
                // set d  gear
                SetGear(GEAR::D);

            } else if (desire_acc > -epsilon_a_ && desire_acc < epsilon_a_) {
                SetGear(GEAR::D);
            }
        }
    }

    if (speed_msg.velocity < standstill_velocity_ && gear_ != gear_msg.gear) {
        SetBrake(control_cmd);
        control_cmd->gear_cmd = gear_;
        // SG_INFO("gear change");
    } else {
        control_cmd->gear_cmd = gear_msg.gear;
    }
    return true;
}

void StateMachine::SetBrake(ControlCmd *control_cmd) {
    control_cmd->acc_cmd = -1;
}

void StateMachine::SetGear(GEAR gear) {
    gear_ = gear;
}

void StateMachine::Clear() {
    p_to_r_duration_count_ = 0;
    p_to_d_duration_count_ = 0;
    r_to_p_duration_count_ = 0;
    d_to_p_duration_count_ = 0;
    brake_duration_count_ = 0;
    brake_cmd_value_ = 0;
    // SG_INFO("sm clear");
}
}  // namespace control_lib
}  // namespace jarvis
