#pragma once
#include <cmath>
#include <vector>

namespace jarvis {
namespace control_lib {
class RungeKutta {
public:
    std::vector<double> Slover(const std::vector<double> &kesi, double delta_f,
                               double acc, double time_len);
    typedef unsigned int uint32_t;

private:
    std::vector<double> KineticsEqu(const std::vector<double> &kesi,
                                    double delta_f, double acc);

private:
    const uint32_t mass_ = 2080;
    const uint32_t iz_ = 4175;
    const uint32_t cf_ = 155494.66;
    const uint32_t cr_ = 155494.66;
    const double lf_ = 1.4224;
    const double lr_ = 1.4224;

    const double epsilon_ = 1.0;
};
}  // namespace pnc_simulator
}  // namespace jarvis
