#include "trajectory_sim.h"

#include <jsoncpp/json/json.h>

#include <cmath>
#include <fstream>
#include <limits>

#include "sglog/sglog.h"
#include "sgtime/sgtime.h"

namespace jarvis {
namespace control_lib {
TrajecotrySim::TrajecotrySim() { InitGlobalPath(); }

bool TrajecotrySim::InitGlobalPath() {
  Json::Reader reader;
  Json::Value root;

  std::string rtk_trajectory_path =
      "/usr/share/jarvis/control/resource/rtk_trajectory/reference_line.json";
  std::ifstream in(rtk_trajectory_path, std::ios::binary);
  if (!in.is_open()) {
    SG_ERROR("json file open failed!");
  }
  if (reader.parse(in, root)) {
    int32_t path_size = root["reference_line"].size();
    const auto &root_ref = root["reference_line"];
    for (int32_t i = 0; i < path_size; ++i) {
      const auto &point = root_ref[i];
      std::vector<double> path_point;
      for (const auto &ite : point) {
        path_point.emplace_back(ite.asDouble());
      }
      global_path_.emplace_back(path_point);
    }
  }
  return true;
}

TrajectoryMsg TrajecotrySim::UpdateTrajectory(double x, double y, double vel) {
  if (vel + 1 < vel_final_) {
    velocity_ = vel + 1;
  }
  return UpdateTrajectory(x, y, );
}

TrajectoryMsg TrajecotrySim::UpdateTrajectory(double x, double y, ) {
  if (count_ >= global_path_.size() - local_len_) {
    SG_INFO("reference line over");
    return TrajectoryMsg();
  }
  int32_t start = count_ > 100 ? count_ - 100 : 0;
  int32_t end =
      count_ + 1000 > global_path_.size() ? global_path_.size() : count_ + 1000;

  double dist_min = std::numeric_limits<double>::max();
  int32_t count_min = -1;
  for (int32_t i = start; i < end; ++i) {
    double delta_x = global_path_[i][0] - x;
    double delta_y = global_path_[i][1] - y;

    double dist = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    if (dist_min > dist) {
      dist_min = dist;
      count_min = i;
    }
  }
  // static double init_count = count_min;
  // count_min = init_count + 500;

  if (dist_min + local_len_ > global_path_.size()) {
    dist_min = global_path_.size() - local_len_;
  }
  trajecotry_.trajectory.clear();
  std::vector<TrajectoryPoint> traj;
  if (std::fabs(velocity_ - 10) < 1e-3) {
    for (int32_t i = count_min; i < count_min + local_len_; ++i) {
      TrajectoryPoint point;
      point.path_point.x = global_path_[i][0];
      point.path_point.y = global_path_[i][1];
      point.path_point.theta = global_path_[i][2];
      point.relative_time = i * time_gap_;
      point.path_point.s = i * velocity_ * time_gap_;
      point.velocity = velocity_;
      traj.emplace_back(point);
    }
  } else {
    double delta_s = vel_final_ * time_gap_;
    double length = delta_s * local_len_;
    double dist = delta_s;
    double ref_dist = 0;
    int32_t count = 0;
    int32_t tem_count = 0;
    int32_t flag = 0;
    while (dist <= length) {
      double dx = global_path_[count_min + count + 1][0] -
                  global_path_[count_min + count][0];
      double dy = global_path_[count_min + count + 1][1] -
                  global_path_[count_min + count][1];
      double dtheta = global_path_[count_min + count + 1][2] -
                      global_path_[count_min + count][2];
      double delta_gap = std::sqrt(dx * dx + dy * dy);
      if (dist > ref_dist + delta_gap) {
        ref_dist += delta_gap;
        count++;
      } else {
        double ratio = (dist - ref_dist) / delta_gap;
        TrajectoryPoint point;
        point.path_point.x = global_path_[count_min + count][0] + dx * ratio;
        point.path_point.y = global_path_[count_min + count][1] + dy * ratio;
        point.path_point.theta =
            global_path_[count_min + count][2] + dtheta * ratio;
        point.relative_time = tem_count * time_gap_;
        point.path_point.s = tem_count * delta_s;
        if (dist_min > 3) {
          point.velocity = velocity_;
        } else {
          point.velocity = 0;
        }
        point.velocity = velocity_;
        tem_count++;
        traj.emplace_back(point);

        dist = dist + delta_s;
      }
    }
  }

  trajecotry_.trajectory = traj;
  trajecotry_.timestamp = jarvis::SgTimeUtil::now_secs();
  trajecotry_.is_update = true;
  count_ = count_min;
  return trajecotry_;
}
}  // namespace control_lib
}  // namespace jarvis