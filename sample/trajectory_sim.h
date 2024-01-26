#pragma once
#include <string>

#include "control/common_struct.h"
namespace jarvis {
namespace control_lib {
class TrajecotrySim {
public:
    TrajecotrySim();
    TrajecotrySim(double time_gap, double velocity)
        : time_gap_(time_gap), velocity_(velocity) {
        TrajecotrySim();
    }

    TrajectoryMsg UpdateTrajectory(double x, double y, double vel);
    TrajectoryMsg UpdateTrajectory(double x, double y);

    TrajectoryMsg GetTrajectory() { return trajecotry_; }

    std::vector<std::vector<double>> GlobalPath() { return global_path_; }

private:
    bool InitGlobalPath();

private:
    TrajectoryMsg trajecotry_;
    std::vector<std::vector<double>> global_path_;
    int32_t local_len_ = 50;
    double time_gap_ = 0.01;
    double velocity_ = 10;
    const double vel_final_ = 12;
    int32_t count_ = 0;
};
}  // namespace control_lib
}  // namespace jarvis