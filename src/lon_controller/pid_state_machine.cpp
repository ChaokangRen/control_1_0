#include "pid_state_machine.h"

#include "sglog/sglog.h"

namespace jarvis {
namespace control_lib {
SpeedPidStateMachine::SpeedPidStateMachine() {
    speed_state_ == SpeedState::LoVelPosEr;
}

void SpeedPidStateMachine::Init(const LonControllerConf &lon_conf) {
    lo_pos_pid_ = lon_conf.lo_pos_pid_conf;
    mi_pos_pid_ = lon_conf.mi_pos_pid_conf;
    hi_pos_pid_ = lon_conf.hi_pos_pid_conf;
    lo_neg_pid_ = lon_conf.lo_neg_pid_conf;
    mi_neg_pid_ = lon_conf.mi_neg_pid_conf;
    hi_neg_pid_ = lon_conf.hi_neg_pid_conf;
}

bool SpeedPidStateMachine::Process(const double speed, const double error,
                                   PidConf &pid_conf) {
    // SG_INFO("speed_state = %d", speed_state_);
    if (speed_state_ == SpeedState::LoVelPosEr) {
        if (error >= 0) {
            if (speed <= low_speed_) {
                // 低速 PID+
                SetPidConf(01, pid_conf);
                speed_state_ = SpeedState::LoVelPosEr;
            } else if (speed > low_speed_) {
                // 中速 PID
                SetPidConf(11, pid_conf);
                speed_state_ = SpeedState::MiVelPosEr;
            }
            // no need reset
            return false;
        } else if (error < 0) {
            if (speed <= low_speed_) {
                // 低速 PID-
                SetPidConf(00, pid_conf);
                speed_state_ = SpeedState::LoVelNegEr;
            } else if (speed > low_speed_) {
                // 中速 PID-
                SetPidConf(10, pid_conf);
                speed_state_ = SpeedState::MiVelNegEr;
            }
            // need reset
            return true;
        }
    } else if (speed_state_ == SpeedState::MiVelPosEr) {
        if (error >= 0) {
            if (speed < low_speed_) {
                // 低速 PID+
                SetPidConf(01, pid_conf);
                speed_state_ = SpeedState::LoVelPosEr;
            } else if (speed >= low_speed_ && speed < mid_speed_) {
                // 中速PID+
                SetPidConf(11, pid_conf);
                speed_state_ = SpeedState::MiVelPosEr;
            } else if (speed >= mid_speed_) {
                // 高速PID+
                SetPidConf(21, pid_conf);
                speed_state_ = SpeedState::HiVelPosEr;
            }
            return false;
        } else if (error < 0) {
            if (speed < low_speed_) {
                // 低速 PID-
                SetPidConf(00, pid_conf);
                speed_state_ = SpeedState::LoVelNegEr;
            } else if (speed >= low_speed_ && speed < mid_speed_) {
                // 中速 PID-
                SetPidConf(10, pid_conf);
                speed_state_ = SpeedState::MiVelNegEr;
            } else if (speed >= mid_speed_) {
                // 高速 PID-
                SetPidConf(20, pid_conf);
                speed_state_ = SpeedState::HiVelNegEr;
            }
            return true;
        }
    } else if (speed_state_ == SpeedState::HiVelPosEr) {
        if (error >= 0) {
            if (speed >= mid_speed_) {
                // 高速 PID+
                SetPidConf(21, pid_conf);
                speed_state_ = SpeedState::HiVelPosEr;
            } else if (speed < mid_speed_) {
                // 中速 PID+
                SetPidConf(11, pid_conf);
                speed_state_ = SpeedState::MiVelPosEr;
            }
            // no need reset
            return false;
        } else if (error < 0) {
            if (speed >= mid_speed_) {
                // 高速 PID-
                SetPidConf(20, pid_conf);
                speed_state_ = SpeedState::HiVelNegEr;
            } else if (speed < mid_speed_) {
                // 中速 PID-
                SetPidConf(10, pid_conf);
                speed_state_ = SpeedState::MiVelNegEr;
            }
            // need reset
            return true;
        }
    } else if (speed_state_ == SpeedState::LoVelNegEr) {
        if (error >= 0) {
            if (speed <= low_speed_) {
                // 低速 PID+
                SetPidConf(01, pid_conf);
                speed_state_ = SpeedState::LoVelPosEr;
            } else if (speed > low_speed_) {
                // 中速 PID+
                SetPidConf(11, pid_conf);
                speed_state_ = SpeedState::MiVelPosEr;
            }
            // no need reset
            return true;
        } else if (error < 0) {
            if (speed <= low_speed_) {
                // 低速 PID-
                SetPidConf(00, pid_conf);
                speed_state_ = SpeedState::LoVelNegEr;
            } else if (speed > low_speed_) {
                // 中速 PID-
                SetPidConf(10, pid_conf);
                speed_state_ = SpeedState::MiVelNegEr;
            }
            // need reset
            return false;
        }
    } else if (speed_state_ == SpeedState::MiVelNegEr) {
        if (error >= 0) {
            if (speed < low_speed_) {
                // 低速 PID+
                SetPidConf(01, pid_conf);
                speed_state_ = SpeedState::LoVelPosEr;
            } else if (speed >= low_speed_ && speed < mid_speed_) {
                // 中速PID+
                SetPidConf(11, pid_conf);
                speed_state_ = SpeedState::MiVelPosEr;
            } else if (speed >= mid_speed_) {
                // 高速PID+
                SetPidConf(21, pid_conf);
                speed_state_ = SpeedState::HiVelPosEr;
            }
            return true;
        } else if (error < 0) {
            if (speed < low_speed_) {
                // 低速 PID-
                SetPidConf(00, pid_conf);
                speed_state_ = SpeedState::LoVelNegEr;
            } else if (speed >= low_speed_ && speed < mid_speed_) {
                // 中速 PID-
                SetPidConf(10, pid_conf);
                speed_state_ = SpeedState::MiVelNegEr;
            } else if (speed >= mid_speed_) {
                // 高速 PID-
                SetPidConf(20, pid_conf);
                speed_state_ = SpeedState::HiVelNegEr;
            }
            return false;
        }
    } else if (speed_state_ == SpeedState::HiVelNegEr) {
        if (error >= 0) {
            if (speed >= mid_speed_) {
                // 高速 PID+
                SetPidConf(21, pid_conf);
                speed_state_ = SpeedState::HiVelPosEr;
            } else if (speed < mid_speed_) {
                // 中速 PID+
                SetPidConf(11, pid_conf);
                speed_state_ = SpeedState::MiVelPosEr;
            }
            return true;
        } else if (error < 0) {
            if (speed >= mid_speed_) {
                // 高速 PID-
                SetPidConf(20, pid_conf);
                speed_state_ = SpeedState::HiVelNegEr;
            } else if (speed < mid_speed_) {
                // 中速 PID-
                SetPidConf(10, pid_conf);
                speed_state_ = SpeedState::MiVelNegEr;
            }
            return false;
        }
    }

    return false;
}
void SpeedPidStateMachine::SetPidConf(const int32_t match_value,
                                      PidConf &pid_conf) {
    switch (match_value) {
        case 01:
            pid_conf = lo_pos_pid_;
            break;

        case 11:
            pid_conf = mi_pos_pid_;
            break;

        case 21:
            pid_conf = hi_pos_pid_;
            break;

        case 00:
            pid_conf = lo_neg_pid_;
            break;

        case 10:
            pid_conf = mi_neg_pid_;
            break;

        case 20:
            pid_conf = hi_neg_pid_;
            break;

        default:
            pid_conf = lo_pos_pid_;
            break;
    }
}
}  // namespace control_lib
}  // namespace jarvis