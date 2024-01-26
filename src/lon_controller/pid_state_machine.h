#pragma once

#include "common.h"

namespace jarvis {
namespace control_lib {

enum class SpeedState {
    LoVelPosEr = 0,
    MiVelPosEr = 1,
    HiVelPosEr = 2,
    LoVelNegEr = 3,
    MiVelNegEr = 4,
    HiVelNegEr = 5
};

class SpeedPidStateMachine {
public:
    SpeedPidStateMachine();
    bool Process(const double speed, const double error, PidConf &pid_conf);
    void Init(const LonControllerConf &lon_conf);

private:
    void SetPidConf(const int32_t match_value, PidConf &pid_conf);
    double low_speed_ = 5.0;
    double mid_speed_ = 13.0;
    double lo_error_ = 0.2;

    SpeedState speed_state_ = SpeedState::LoVelPosEr;

    PidConf lo_pos_pid_;
    PidConf mi_pos_pid_;
    PidConf hi_pos_pid_;

    PidConf lo_neg_pid_;
    PidConf mi_neg_pid_;
    PidConf hi_neg_pid_;
};
}  // namespace control_lib
}  // namespace jarvis