#pragma once

#include "common.h"

namespace jarvis {
namespace control_lib {

class LonControllerInterface {
public:
    virtual ~LonControllerInterface(){};

    static LonControllerInterface *CreateInstance();

    virtual bool Init(const ControlConf *control_conf) = 0;

    virtual bool ComputeControlCommand(const LocalizationMsg &localization_msg,
                                       const SpeedMsg &speed_msg,
                                       const GearMsg &gear_msg,
                                       const TrajectoryMsg &trajectory_msg,
                                       ControlCmd *control_cmd,
                                       std::string &control_info) = 0;

    virtual bool Reset() = 0;
};

}  // namespace control_lib
}  // namespace jarvis