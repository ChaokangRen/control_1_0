#pragma once

#include <string>

#include "common.h"

namespace jarvis {
namespace control_lib {

class LatControllerInterface {
public:
    virtual ~LatControllerInterface(){};

    static LatControllerInterface *CreateInstance();

    virtual bool Init(const ControlConf *control_conf) = 0;

    virtual bool ComputeControlCommand(const LocalizationMsg &localization_msg,
                                       const SpeedMsg &speed_msg,
                                       const SteerMsg &steer_msg,
                                       const TrajectoryMsg &trajectory_msg,
                                       ControlCmd *control_cmd,
                                       std::string &control_info) = 0;

    // virtual bool MpcControl(const LocalizationMsg &localization_msg,
    //                         const ChassisMsg &chassis_msg,
    //                         const TrajectoryMsg &trajectory_msg,
    //                         ControlCmd *control_cmd);

    virtual bool Reset() = 0;
};

}  // namespace control_lib
}  // namespace jarvis