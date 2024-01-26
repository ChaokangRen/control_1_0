#include "trajectory_gen.h"

#include <cmath>

//#include <iostream>
#include "sglog/sglog.h"
#include <fstream>
#include <iostream>
#include <string>
using namespace std;

static vector<double> refline_x;
static vector<double> refline_y;
static vector<double> refline_yaw;
const double kYawDiffThrold = 0.1;
inline double DegreeToRad(double yaw_degree) { return yaw_degree * M_PI / 180; }
double ProcessYaw(double yaw) {
  // if (yaw > 0)
  //     while (yaw > 0) yaw -= 2 * M_PI;
  // if (yaw < -2 * M_PI)
  //     while (yaw < -2 * M_PI) yaw += 2 * M_PI;
  double process_yaw = 0;

  // if (yaw >= -0.5 * M_PI && yaw < 0) {
  //     process_yaw = M_PI + yaw;
  // } else if (yaw >= -1.5 * M_PI && yaw < -M_PI) {
  //     process_yaw = yaw + 0.5 * M_PI;
  // } else if (yaw >= -2 * M_PI && yaw < -1.5 * M_PI) {
  //     process_yaw = yaw + 2.5 * M_PI;
  // }
  process_yaw = yaw + M_PI / 2;
  return process_yaw;
}
void LoadRefline(void) {
  ifstream ifs_x;
  ifs_x.open(
      "/usr/share/jarvis/control/resource/rtk_trajectory/refline_x1229_2.txt",
      ios::in);
  if (!ifs_x.is_open()) {
    cout << "line_x.txt open failed" << endl;
    return;
  }
  string buf;
  while (getline(ifs_x, buf)) {
    refline_x.push_back(stod(buf));
  }
  ifs_x.close();
  ifstream ifs_y;
  ifs_y.open(
      "/usr/share/jarvis/control/resource/rtk_trajectory/refline_y1229_2.txt",
      ios::in);
  if (!ifs_y.is_open()) {
    cout << "line_y.txt open failed" << endl;
    return;
  }
  while (getline(ifs_y, buf)) {
    refline_y.push_back(stod(buf));
  }
  ifs_y.close();
  ifstream ifs_yaw;
  // ifs_yaw.open("src/latlong_ctrl_main/src/refline_yaw1229_2.txt", ios::in);
  ifs_yaw.open(
      "/usr/share/jarvis/control/resource/rtk_trajectory/realtime_phi_ren.txt",
      ios::in);
  if (!ifs_yaw.is_open()) {
    cout << "line_yaw.txt open failed" << endl;
    return;
  }
  DegreeToRad(23);
  while (getline(ifs_yaw, buf)) {
    double yaw_d = stod(buf);
    // yaw_d = DegreeToRad(yaw_d);
    // yaw_d = ProcessYaw(yaw_d);
    refline_yaw.push_back(yaw_d);
  }
  ifs_yaw.close();
}

void GetRealtimeTrajectoryInNpStep(const double np_step, const double current_x,
                                   const double current_y,
                                   const double current_yaw, const double vel,
                                   const double y_dot, const double period_t,
                                   vector<vector<double>> &refline) {
  static const uint16_t ref_len = refline_x.size();
  static uint16_t current_count = 0;
  static const uint16_t max_find_num = 1000;
  double min_dist = 100000;
  double dist_iter = 0;
  uint16_t iter_count = 0;
  double yumiao_t = 0.2;
  double yumiao_x = current_x + vel * yumiao_t;
  double yumiao_y = current_y + y_dot * yumiao_t;

  for (int i = current_count; i < current_count + max_find_num; ++i) {
    dist_iter = (refline_x[i] - yumiao_x) * (refline_x[i] - yumiao_x) +
                (refline_y[i] - yumiao_y) * (refline_y[i] - yumiao_y);
    if (min_dist > dist_iter /*&&
             abs(current_yaw - refline_yaw[i]) < kYawDiffThrold */) {
      min_dist = dist_iter;
      iter_count = i;
    }
  }
  current_count = iter_count;
  double dist_sum = 0;
  for (int i = 0; i < np_step; ++i) {
    double dist_i = vel * period_t * (i + 1);
    double dist_gap = 0;
    while (dist_sum < dist_i) {
      dist_gap = sqrt((refline_x[iter_count + 1] - refline_x[iter_count]) *
                          (refline_x[iter_count + 1] - refline_x[iter_count]) +
                      (refline_y[iter_count + 1] - refline_y[iter_count]) *
                          (refline_y[iter_count + 1] - refline_y[iter_count]));
      dist_sum += dist_gap;
      ++iter_count;
    }
    // using linear interpolation to find precise value
    double need_add_len = dist_gap - (dist_sum - dist_i);
    // double theta = ProcessYaw(refline_yaw[iter_count - 1]);
    double theta = refline_yaw[iter_count - 1];
    double current_ref_x =
        refline_x[iter_count - 1] + need_add_len * cos(theta);
    double current_ref_y =
        refline_y[iter_count - 1] + need_add_len * sin(theta);
    double current_ref_phi = refline_yaw[iter_count - 1];
    refline[0][i] = current_ref_y;
    refline[1][i] = current_ref_x;
    refline[2][i] = current_ref_phi;
  }
}