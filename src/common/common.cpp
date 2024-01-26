#include "common.h"

#include <cmath>
const double semimajor_len = 6378137;
const double semiminor_len = 6356752.31414;
const double earth_radius = 6378137;
const double oblateness = 1 / 298.257222101;
const double first_eccentricity = 0.0818191910428;

const double orgin_longitude = 2.074710701656759;
const double orgin_latitude = 0.5586257075569977;
const double orgin_altitude = 0.22758597622763593;

const double orgin_x = -2614020.578497937;
const double orgin_y = 4740731.728376352;
const double orgin_z = 3361079.9529776173;
namespace jarvis {
namespace control_lib {
void Wgs84ToLocalCoord(const double longitude, const double latitude,
                       const double altitude, double *local_x, double *local_y,
                       double *local_z) {
    double earth_radius_p =
        earth_radius * (1 + oblateness * sin(latitude) * sin(latitude));
    double spre_coord_x =
        (earth_radius_p + altitude) * cos(latitude) * cos(longitude);
    double spre_coord_y =
        (earth_radius_p + altitude) * cos(latitude) * sin(longitude);
    double spre_coord_z =
        ((1 - first_eccentricity * first_eccentricity) * earth_radius_p +
         altitude) *
        sin(latitude);

    *local_x = -sin(orgin_longitude) * (spre_coord_x - orgin_x) +
               cos(orgin_longitude) * (spre_coord_y - orgin_y);
    *local_y =
        -sin(orgin_latitude) * cos(orgin_longitude) * (spre_coord_x - orgin_x) -
        sin(orgin_latitude) * sin(orgin_longitude) * (spre_coord_y - orgin_y) +
        cos(orgin_latitude) * (spre_coord_z - orgin_z);
    *local_z =
        cos(orgin_latitude) * cos(orgin_longitude) * (spre_coord_x - orgin_x) -
        cos(orgin_latitude) * sin(orgin_longitude) * (spre_coord_y - orgin_y) +
        sin(orgin_latitude) * (spre_coord_z - orgin_z);
}
double GoldenSectionSearch(const std::function<double(double)> &func,
                           const double lower_bound, const double upper_bound,
                           const double tol) {
    static constexpr double gr = 1.618033989;

    double a = lower_bound;
    double b = upper_bound;

    double t = (b - a) / gr;
    double c = b - t;
    double d = a + t;

    while (std::abs(c - d) > tol) {
        if (func(c) < func(d)) {
            b = d;
        } else {
            a = c;
        }
        t = (b - a) / gr;
        c = b - t;
        d = a + t;
    }
    return (a + b) * 0.5;
}
// NONE = 0, P = 1, R = 2, N = 3, D = 4
uint8_t GearCommandValue(GEAR gear) {
    switch (gear) {
        case GEAR::NONE:
            return 0;
            break;
        case GEAR::P:
            return 1;
            break;
        case GEAR::R:
            return 2;
            break;
        case GEAR::N:
            return 3;
            break;
        case GEAR::D:
            return 4;
            break;
        default:
            break;
    }
    return 0;
}
}  // namespace control_lib
}  // namespace jarvis