#pragma once

#include <string>

#include "common.h"
#include "control/control_interface.h"
#include "json_parser.h"
#include "lat_controller_interface.h"
#include "lon_controller_interface.h"
#include "state_machine.h"
namespace jarvis {
namespace control_lib {

typedef std::shared_ptr<jarvis::control_lib::LatControllerInterface>
    LatControllerInterfacePtr;

typedef std::shared_ptr<jarvis::control_lib::LonControllerInterface>
    LonControllerInterfacePtr;

class ControlImpl : public ControlInterface {
public:
    ControlImpl();
    virtual ~ControlImpl();

    std::string get_version() override;
    bool init(const ConfigFile &config_file) override;
    bool set_localization_msg(const LocalizationMsg &localization_msg) override;
    bool set_speed_msg(const SpeedMsg &speed_msg) override;
    bool set_wheel_msg(const WheelMsg &wheel_msg) override;
    bool set_steer_msg(const SteerMsg &steer_msg) override;
    bool set_gear_msg(const GearMsg &gear_msg) override;
    bool set_acceleration_msg(const AccelerationMsg &acceleration_msg) override;
    bool set_trajectory_msg(const TrajectoryMsg &trajectory_msg) override;
    bool execute(ControlCmd *control_cmd, std::string &debuginfo) override;

private:
    ControlConf control_conf_;

    ControlCmd pre_control_cmd_;

    JsonParser json_parser_;

    LatControllerInterfacePtr lat_controller_ptr;
    LonControllerInterfacePtr lon_controller_ptr;
    StateMachine state_machine_;

    LocalizationMsg localization_msg_;
    SpeedMsg speed_msg_;
    WheelMsg wheel_msg_;
    SteerMsg steer_msg_;
    GearMsg gear_msg_;
    AccelerationMsg acceleration_msg_;
    TrajectoryMsg trajectory_msg_;

    int count_steering = 0;
    double jerk_pre_ = 0.0;
    double acc_pre_ = 0.0;
    double t_pre_ = 0.0;

    int count_control_ = 0;
};

}  // namespace control_lib
}  // namespace jarvis
