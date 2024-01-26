#pragma once

#include <jsoncpp/json/json.h>

#include <string>

#include "common.h"

namespace jarvis {
namespace control_lib {
class JsonParser {
public:
    JsonParser() = default;

    bool ParserVehicleConf(const std::string &path);
    bool ParserLatConf(const std::string &path);
    bool ParserLonConf(const std::string &path);
    bool ParserGearConf(const std::string &path);

    const VehicleParams GetVehicleParams() {
        return vehicle_params_;
    }
    const LatControllerConf GetLatConf() {
        return lat_conf_;
    }
    const LonControllerConf GetLonConf() {
        return lon_conf_;
    }
    const GearStateMachineConf GetGearConf() {
        return gear_conf_;
    }

private:
    VehicleParams vehicle_params_;
    LatControllerConf lat_conf_;
    LonControllerConf lon_conf_;
    GearStateMachineConf gear_conf_;
};
}  // namespace control_lib
}  // namespace jarvis