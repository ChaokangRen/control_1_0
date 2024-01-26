#pragma once
#include <map>
// #include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace jarvis {
namespace control_lib {
/**
 * @class Interpolation2D
 * @brief linear interpolation from key(double ,double) to one double value
 */
class Interpolation2D {
public:
    typedef std::vector<std::tuple<double, double, double>> DataType;
    typedef std::pair<double, double> KeyType;

    Interpolation2D() = default;

    /**
     * @brief initialize Initerpolation2D internal table
     *
     * @param xyz table data
     * @return true
     * @return false
     */
    bool Init(const DataType &xyz);
    /**
     * @brief linear interpolate from 2D key(double,double to one double value)
     *
     */
    double Interpolate(const KeyType &xy) const;

private:
    double InterpolateYz(const std::map<double, double> &yz_table,
                         double y) const;
    double InterpolateValue(const double value_before, const double dist_before,
                            const double value_after,
                            const double dist_after) const;

    std::map<double, std::map<double, double>> xyz_;
};
}  // namespace control_lib
}  // namespace jarvis
