#include "trajectory_analyzer.h"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace jarvis {
namespace control_lib {
double PointDistanceSquare(const TrajectoryPoint &point, const double x,
                           const double y) {
    double dx = point.path_point.x - x;
    double dy = point.path_point.y - y;
    return dx * dx + dy * dy;
}
PathPoint TrajecotoryAnalyzer::QueryMatchedPathPoint(const double x,
                                                     const double y) const {
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);

    uint32_t index_min = 0;

    for (uint32_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    uint32_t index_start = (index_min == 0 ? index_min : index_min - 1);
    uint32_t index_end =
        (index_min + 1 == trajectory_points_.size() ? index_min
                                                    : index_min + 1);
    const double epsilon = 0.001;
    if (index_start == index_end ||
        std::fabs(trajectory_points_[index_start].path_point.s -
                      trajectory_points_[index_end].path_point.s <=
                  epsilon)) {
        return trajectory_points_[index_start].path_point;
    }
    return FindMinDistancePoint(trajectory_points_[index_start],
                                trajectory_points_[index_end], x, y);
}
PathPoint TrajecotoryAnalyzer::FindMinDistancePoint(const TrajectoryPoint &p0,
                                                    const TrajectoryPoint &p1,
                                                    const double x,
                                                    const double y) const {
    auto dist_square = [&p0, &p1, &x, &y](const double s) {
        double px = lerp(p0.path_point.x, p0.path_point.s, p1.path_point.x,
                         p1.path_point.s, s);
        double py = lerp(p0.path_point.y, p0.path_point.s, p1.path_point.y,
                         p1.path_point.s, s);
        double dx = px - x;
        double dy = py - y;
        return dx * dx + dy * dy;
    };

    PathPoint p = p0.path_point;

    double s =
        GoldenSectionSearch(dist_square, p0.path_point.s, p1.path_point.s);

    p.s = s;
    p.x = lerp(p0.path_point.x, p0.path_point.s, p1.path_point.x,
               p1.path_point.s, s);
    p.y = lerp(p0.path_point.y, p0.path_point.s, p1.path_point.y,
               p1.path_point.s, s);
    p.theta_rad = lerp(p0.path_point.theta_rad, p0.path_point.s,
                       p1.path_point.theta_rad, p1.path_point.s, s);
    p.kappa = lerp(p0.path_point.kappa, p0.path_point.s, p1.path_point.kappa,
                   p1.path_point.s, s);
    return p;
}

void TrajecotoryAnalyzer::ToTrajctoryFrame(const double x, const double y,
                                           const double theta, const double v,
                                           const PathPoint &ref_point,
                                           double *ptr_s, double *ptr_s_dot,
                                           double *ptr_d,
                                           double *ptr_d_dot) const {
    double dx = x - ref_point.x;
    double dy = y - ref_point.y;

    double cos_ref_theta = std::cos(ref_point.theta_rad);
    double sin_ref_theta = std::sin(ref_point.theta_rad);

    // the sin of diff angle between vector (cos_theta,sin_theta) and (dx,dy)
    double cross_rd_nd = cos_ref_theta * dy - sin_ref_theta * dx;
    *ptr_d = cross_rd_nd;
    // the cos of diff angle between vector (cos_theta,sin_theta) and (dx,dy)
    double dot_rd_nd = dx * cos_ref_theta + dy * sin_ref_theta;
    *ptr_s = ref_point.s + dot_rd_nd;

    double delta_theta = theta - ref_point.theta_rad;
    double cos_delta_theta = std::cos(delta_theta);
    double sin_delta_theta = std::sin(delta_theta);

    *ptr_d_dot = v * sin_delta_theta;

    double one_minus_kappa_r_d = 1 - ref_point.kappa * (*ptr_d);

    if (one_minus_kappa_r_d <= 0.0) {
        // SG_ERROR(
        //     "TrajectoryAnalyzer::ToTrajectoryFrame found fatal reference and
        //     " "actual difference.control output might might be "
        //     "unstable,ref_point.kappa=%f,ref_point.x=%f,ref_point.y=%f,x=%f,y=%"
        //     "f,*ptr_d=%f,one_minus_kappa_r_d=%f",
        //     ref_point.kappa, ref_point.x, ref_point.y, x, y, *ptr_d,
        //     one_minus_kappa_r_d);
        one_minus_kappa_r_d = 0.01;
    }
    *ptr_s_dot = v * cos_delta_theta / one_minus_kappa_r_d;
}

TrajectoryPoint TrajecotoryAnalyzer::QueryNearestPointByAbsoulteTime(
    const double t) const {
    // SG_INFO("t = %lf,header_time = %lf,delta_t = %lf", t, header_time_,
    //         t - header_time_);
    return QueryNearestPointByRelativeTime(t - header_time_);
}

TrajectoryPoint TrajecotoryAnalyzer::QueryNearestPointByRelativeTime1(
    const double veh_x, const double veh_y) const {
    int32_t count_min = 0;
    double dist_min = 1e10;
    double dist = 0;
    for (int i = 0; i < trajectory_points_.size(); ++i) {
        dist = std::sqrt((veh_x - trajectory_points_[i].path_point.x) *
                             (veh_x - trajectory_points_[i].path_point.x) +
                         (veh_y - trajectory_points_[i].path_point.y) *
                             (veh_y - trajectory_points_[i].path_point.y));
        if (dist < dist_min) {
            dist_min = dist;
            count_min = i;
        }
    }
    auto point = trajectory_points_[count_min + 1];
    if (count_min + 70 < trajectory_points_.size()) {
        count_min += 70;
    } else {
        count_min = trajectory_points_.size() - 1;
    }
    // SG_INFO("count = %d,s = %lf,vel = %lf", count_min,
    //         trajectory_points_[count_min - 30].path_point.s,
    //         trajectory_points_[count_min].velocity);
    point.velocity = trajectory_points_[count_min].velocity;
    // return trajectory_points_[count_min];
    return point;
}

double TrajecotoryAnalyzer::MatchedS(const double veh_x,
                                     const double veh_y) const {
    int32_t count_min = 0;
    double dist_min = 1e10;
    double dist = 0;
    for (int i = 0; i < trajectory_points_.size(); ++i) {
        dist = std::sqrt((veh_x - trajectory_points_[i].path_point.x) *
                             (veh_x - trajectory_points_[i].path_point.x) +
                         (veh_y - trajectory_points_[i].path_point.y) *
                             (veh_y - trajectory_points_[i].path_point.y));

        if (dist < dist_min) {
            dist_min = dist;
            count_min = i;
        }
        // SG_INFO("i = %d,dist = %lf,dist min = %lf,count_min = %d,s = %lf", i,
        //         dist, dist_min, count_min,
        //         trajectory_points_[i].path_point.s);
    }
    return trajectory_points_[count_min].path_point.s;
}

TrajectoryPoint TrajecotoryAnalyzer::QueryNearestPointByRelativeTime(
    const double t) const {
    auto func_comp = [](const TrajectoryPoint &point,
                        const double relative_time) {
        return point.relative_time < relative_time;
    };

    auto it_low = std::lower_bound(trajectory_points_.begin(),
                                   trajectory_points_.end(), t, func_comp);

    if (it_low == trajectory_points_.begin()) {
        return trajectory_points_.front();
    }
    if (it_low == trajectory_points_.end()) {
        return trajectory_points_.back();
    }

    auto it_lower = it_low - 1;
    if (it_low->relative_time - t < t - it_lower->relative_time) {
        return *it_low;
    }
    return *it_lower;
}

std::vector<std::tuple<double, double, double>>
TrajecotoryAnalyzer::MpcReferenceTrajectory(const int32_t horizon,
                                            const double ts, const double veh_x,
                                            const double veh_y,
                                            const double vel, double &y_error,
                                            bool &has_enough_horizon) {
    std::vector<std::tuple<double, double, double>> ref_trajectory;
    // for (int32_t i = 0; i < horizon; ++i) {
    //   ref_trajectory.emplace_back(
    //       std::make_tuple(trajectory_points_[i + 1].path_point.y,
    //                       trajectory_points_[i + 1].path_point.x,
    //                       trajectory_points_[i + 1].path_point.theta));
    // }
    int32_t count_min = 0;
    double dist_min = 1e10;
    double dist = 0;
    for (int i = 0; i < trajectory_points_.size(); ++i) {
        dist = std::sqrt((veh_x - trajectory_points_[i].path_point.x) *
                             (veh_x - trajectory_points_[i].path_point.x) +
                         (veh_y - trajectory_points_[i].path_point.y) *
                             (veh_y - trajectory_points_[i].path_point.y));
        if (dist < dist_min) {
            dist_min = dist;
            count_min = i;
        }
    }
    // count_min += 1;
    int32_t xy_count_min = count_min;

    // SG_INFO("dist_min = %lf,count_min = %d", dist_min, count_min);
    y_error = dist_min;
    static double max_dist = -1;
    static double aver_dist = 0;
    static int32_t count_ave = 0;
    aver_dist = aver_dist * count_ave + dist_min;
    count_ave++;
    aver_dist = aver_dist / count_ave;
    // SG_INFO("aver_dist = %lf", aver_dist);
    if (dist_min > max_dist) {
        max_dist = dist_min;
    }
    // SG_INFO("max dist = %lf", max_dist);
    double preview = 3.5;
    double overtake_x = 3172.58;
    double overtake_y = 2806.40;

    double ov_dx = overtake_x - veh_x;
    double ov_dy = overtake_y - veh_y;
    double overtake_dist_sqr = ov_dx * ov_dx + ov_dy * ov_dy;

    if (overtake_dist_sqr < 1225) {
        preview = 1.5;
    }
    // SG_INFO("vx %lf,vy = %lf,ox = %lf,oy = %lf", veh_x, veh_y, overtake_x,
    //         overtake_y);
    // SG_INFO("preview dist = %lf,overtake_dist_sqr = %lf", preview,
    //         overtake_dist_sqr);
    // if(5 < vel < 7.5){
    //     preview = 1.5;
    // }
    // preview = std::fmax(2.0, 0.1 * vel);
    // if (vel > 12) {
    //     preview = std::fmax(1.5, 0.05 * vel);
    // } else {
    //     preview = std::fmax(3.5, 0.2 * vel);
    // }

    double step_length = 0;
    for (int i = count_min + 1; i < trajectory_points_.size(); ++i) {
        double dx = trajectory_points_[i].path_point.x -
                    trajectory_points_[i - 1].path_point.x;
        double dy = trajectory_points_[i].path_point.y -
                    trajectory_points_[i - 1].path_point.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        step_length += dist;
        count_min = i;
        if (step_length > preview) {
            break;
        }
    }

    double delta_s = vel * ts;
    if (vel < 1) {
        delta_s = 1 * ts;
    }

    double dist_len = 0;
    int32_t np = 1;
    for (int i = count_min; i < trajectory_points_.size(); ++i) {
        double dx = trajectory_points_[i].path_point.x -
                    trajectory_points_[i - 1].path_point.x;
        double dy = trajectory_points_[i].path_point.y -
                    trajectory_points_[i - 1].path_point.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        while (dist_len + dist > np * delta_s) {
            double ref_x = lerp(trajectory_points_[i - 1].path_point.x,
                                dist_len, trajectory_points_[i].path_point.x,
                                dist + dist_len, np * delta_s);
            double ref_y = lerp(trajectory_points_[i - 1].path_point.y,
                                dist_len, trajectory_points_[i].path_point.y,
                                dist + dist_len, np * delta_s);
            double ref_theta =
                lerp(trajectory_points_[i - 1].path_point.theta_rad, dist_len,
                     trajectory_points_[i].path_point.theta_rad,
                     dist + dist_len, np * delta_s);
            // SG_INFO("ref_x = %lf,ref_y = %lf,ref_theta = %lf", ref_x, ref_y,
            //         ref_theta);
            ref_trajectory.emplace_back(
                std::make_tuple(ref_y, ref_x, ref_theta));
            np++;
            if (np > horizon) break;
        }
        dist_len += dist;
        if (np > horizon) break;
    }
    np = 0;
    dist_len = 0;
    for (int i = xy_count_min + 1; i < trajectory_points_.size(); ++i) {
        if (ref_trajectory.empty()) {
            break;
        }
        double dx = trajectory_points_[i].path_point.x -
                    trajectory_points_[i - 1].path_point.x;
        double dy = trajectory_points_[i].path_point.y -
                    trajectory_points_[i - 1].path_point.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        while (dist_len + dist > np * delta_s) {
            double ref_x = lerp(trajectory_points_[i - 1].path_point.x,
                                dist_len, trajectory_points_[i].path_point.x,
                                dist + dist_len, np * delta_s);
            double ref_y = lerp(trajectory_points_[i - 1].path_point.y,
                                dist_len, trajectory_points_[i].path_point.y,
                                dist + dist_len, np * delta_s);
            ref_trajectory[np] =
                std::make_tuple(ref_y, ref_x, std::get<2>(ref_trajectory[np]));
            np++;
            if (np > horizon || np >= ref_trajectory.size()) break;
        }
        dist_len += dist;
        if (np > horizon || np >= ref_trajectory.size()) break;
    }

    if (ref_trajectory.size() < horizon && ref_trajectory.size() != 0) {
        if (ref_trajectory.size() < 10) {
            has_enough_horizon = false;
        }
        double back_x = std::get<1>(ref_trajectory.back());
        double back_y = std::get<0>(ref_trajectory.back());
        double back_yaw = std::get<2>(ref_trajectory.back());
        double cos_yaw = std::cos(back_yaw);
        double sin_yaw = std::sin(back_yaw);
        for (int i = 0; horizon - ref_trajectory.size() > 0; i++) {
            back_x = back_x + delta_s * cos_yaw;
            back_y = back_y + delta_s * sin_yaw;
            ref_trajectory.emplace_back(
                std::make_tuple(back_y, back_x, back_yaw));
        }
        // SG_WARN("ref_trajectory less");
    }
    if (ref_trajectory.size() == 0) {
        double back_x = trajectory_points_[count_min].path_point.x;
        double back_y = trajectory_points_[count_min].path_point.y;
        back_x = veh_x;
        back_y = veh_y;
        double back_yaw = trajectory_points_[count_min].path_point.theta_rad;
        double cos_yaw = std::cos(back_yaw);
        double sin_yaw = std::sin(back_yaw);
        back_x = back_x + 0.5 * cos_yaw;
        back_y = back_y + 0.5 * sin_yaw;
        for (int i = 0; i < horizon; i++) {
            back_x = back_x + delta_s * cos_yaw;
            back_y = back_y + delta_s * sin_yaw;
            ref_trajectory.emplace_back(
                std::make_tuple(back_y, back_x, back_yaw));
        }
        has_enough_horizon = false;
    }
    return ref_trajectory;
}

void TrajecotoryAnalyzer::ThetaSmooth() {
    static const double theta_dist = M_PI;
    for (int i = 1; i < trajectory_points_.size(); ++i) {
        double theta_error = trajectory_points_[i].path_point.theta_rad -
                             trajectory_points_[i - 1].path_point.theta_rad;
        if (theta_error > theta_dist) {
            trajectory_points_[i].path_point.theta_rad -= 2 * M_PI;
        } else if (theta_error < -theta_dist) {
            trajectory_points_[i].path_point.theta_rad += 2 * M_PI;
        }
    }
}
}  // namespace control_lib
}  // namespace jarvis