#ifndef _TRAJECTORYGEN_H_
#define _TRAJECTORYGEN_H_
#include <vector>
using namespace std;
typedef unsigned int uint32_t;
void LoadRefline(void);
double ProcessYaw(double yaw);
double DegreeToRad(double yaw_degree);
void GetRealtimeTrajectoryInNpStep(const double np_step, const double current_x,
                                   const double current_y,
                                   const double current_yaw, const double vel,const double y_dot,
                                   const double period_t,
                                   vector<vector<double>>& refline);
#endif