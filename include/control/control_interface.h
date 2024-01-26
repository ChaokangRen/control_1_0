#pragma once

#include <sgtime/sgtime.h>
#include <stdint.h>

#include <cmath>
#include <string>
#include <vector>

namespace jarvis {
namespace control_lib {

struct Header {
    int seq;
    jarvis::Second stamp;
    std::string frame_id;
};

struct ConfigFile {
    std::string vehicle_parameter_path;
    std::string lateral_controller_config_path;
    std::string longitudinal_controller_config_path;
    std::string gear_state_machine_conf_path;
};

struct LocalizationMsg {
    Header header;
    // x,y,z in ENU coordinates
    double x;
    double y;
    double z;
    // x,y,z velocity in ENU coordinates
    double dx;
    double dy;
    double dz;
    double roll_rad;
    double pitch_rad;
    double yaw_rad;
    double dot_yaw_radps;
    bool is_update = false;
};

enum GEAR { NONE = 0, P = 1, R = 2, N = 3, D = 4 };

struct SpeedMsg {
    Header header;
    double velocity;
    bool is_update = false;
};

struct WheelMsg {
    Header header;
    double fl;
    double fr;
    double rl;
    double rr;
    bool is_update = false;
};

struct SteerMsg {
    Header header;
    double steer_angle_rad;
    bool is_update = false;
};

struct GearMsg {
    Header header;
    GEAR gear;
    bool is_update = false;
};

struct AccelerationMsg {
    Header header;
    double acc_x;
    double acc_y;
    bool is_update = false;
};

typedef struct {
    // x,y,z points coordinates  on the reference trajectory
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double theta_rad = 0.0;
    double kappa = 0.0;

    double s = 0.0;

    double dkappa = 0.0;
    double ddkappa = 0.0;

} PathPoint;

typedef struct {
    PathPoint path_point;
    double velocity;
    double acceleration;
    double relative_time;
    double dot_acc;
} TrajectoryPoint;

struct TrajectoryMsg {
    Header header;
    std::vector<TrajectoryPoint> trajectory;
    GEAR gear;
    bool is_update = false;
};

struct ControlCmd {
    double steering_cmd_rad;
    double acc_cmd;
    GEAR gear_cmd;
};

class ControlInterface {
public:
    virtual ~ControlInterface(){};

    static ControlInterface *create_instance();

    virtual std::string get_version() = 0;
    virtual bool init(const ConfigFile &config_file) = 0;

    // The function computes and returns the control command based on setting
    // the local, speed, wheel, steer, gear, acc, traj message. And the message
    // struct's is_update flage must be set Ture. Then set the is_update flage
    // to False after calling the function.
    //
    // Usage:
    //    set_xxx_msg(xxx);     // set is_update = ture
    //    ControlCmd *control_cmd;
    //    std::string debuginfo;
    //    execute(control_cmd, debuginfo);
    //    set_xxx_msg(xxx);     // set is_update = false
    virtual bool execute(ControlCmd *control_cmd, std::string &debuginfo) = 0;
    virtual bool set_localization_msg(
        const LocalizationMsg &localization_msg) = 0;
    virtual bool set_speed_msg(const SpeedMsg &speed_msg) = 0;
    virtual bool set_wheel_msg(const WheelMsg &wheel_msg) = 0;
    virtual bool set_steer_msg(const SteerMsg &steer_msg) = 0;
    virtual bool set_gear_msg(const GearMsg &gear_msg) = 0;
    virtual bool set_acceleration_msg(
        const AccelerationMsg &acceleration_msg) = 0;
    virtual bool set_trajectory_msg(const TrajectoryMsg &trajectory_msg) = 0;
};

}  // namespace control_lib
}  // namespace jarvis