#pragma once

#include "common.h"
#include "digital_filter.h"
#include "digital_filter_coefficients.h"
#include "interpolation_2d.h"
#include "lon_controller_interface.h"
#include "pid_controller.h"
#include "pid_state_machine.h"
#include "trajectory_analyzer.h"

namespace jarvis {
namespace control_lib {

class LonController : public LonControllerInterface {
public:
    LonController() = default;

    virtual ~LonController(){};

    bool Init(const ControlConf *control_conf) override;

    bool ComputeControlCommand(const LocalizationMsg &localization_msg,
                               const SpeedMsg &speed_msg,
                               const GearMsg &gear_msg,
                               const TrajectoryMsg &trajectory_msg,
                               ControlCmd *control_cmd,
                               std::string &control_info);
    virtual bool Reset() override;

private:
    void ComputeLongitudinalError(
        const TrajecotoryAnalyzer &trajectory_analyzer,
        const double preview_time, double *station_error, double *speed_error);

private:
    PIDController speed_pid_controller_;
    PIDController station_pid_controller_;

    Interpolation2D brake_table_;
    Interpolation2D throttle_table_;

    VehicleState vehicle_state_;

    const ControlConf *control_conf_ = nullptr;

    bool lon_controller_initialized_ = false;

    bool is_use_preview_time_ = true;

    double ts_ = 0.0;

    double acc_cmd_ = 0.0;

    double v_min_ = 15;

    DigitalFilter digital_filter_pitch_angle_;

    SpeedPidStateMachine speed_state_machine_;
};

}  // namespace control_lib
}  // namespace jarvis