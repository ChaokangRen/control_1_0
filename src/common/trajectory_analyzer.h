#pragma once

#include <vector>

#include "common.h"

namespace jarvis {
namespace control_lib {
class TrajecotoryAnalyzer {
public:
    TrajecotoryAnalyzer() = default;

    TrajecotoryAnalyzer(const TrajectoryMsg trajectory)
        : trajectory_points_(trajectory.trajectory) {
        seq_num_ = trajectory.trajectory.size();
        header_time_ = trajectory.header.stamp;
        ThetaSmooth();
    }

    ~TrajecotoryAnalyzer() = default;

    PathPoint QueryMatchedPathPoint(const double x, const double y) const;

    void ToTrajctoryFrame(const double x, const double y, const double theta,
                          const double v, const PathPoint &ref_point,
                          double *ptr_s, double *ptr_s_dot, double *ptr_d,
                          double *ptr_d_dot) const;

    TrajectoryPoint QueryNearestPointByRelativeTime(const double t) const;

    TrajectoryPoint QueryNearestPointByAbsoulteTime(const double t) const;
    TrajectoryPoint QueryNearestPointByRelativeTime1(const double veh_x,
                                                     const double veh_y) const;

    std::vector<std::tuple<double, double, double>> MpcReferenceTrajectory(
        const int32_t horizon, const double ts, const double veh_x,
        const double veh_y, const double vel, double &y_error,
        bool &has_enough_horizon);

    double MatchedS(const double veh_x, const double veh_y) const;

private:
    PathPoint FindMinDistancePoint(const TrajectoryPoint &p0,
                                   const TrajectoryPoint &p1, const double x,
                                   const double y) const;

    void ThetaSmooth();

private:
    std::vector<TrajectoryPoint> trajectory_points_;

    double header_time_ = 0.0;
    int32_t seq_num_ = 0;
};
}  // namespace control_lib
}  // namespace jarvis