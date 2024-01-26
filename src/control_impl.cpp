#include "control_impl.h"

#include <math.h>
#include <sglog/sglog.h>
#include <sgtime/sgtime.h>

#include <string>

#include "lat_controller_interface.h"

namespace jarvis {
namespace control_lib {

ControlInterface *ControlInterface::create_instance() {
    return new ControlImpl();
}

ControlImpl::ControlImpl() {
    SG_INFO("ControlImpl construct");
}

ControlImpl::~ControlImpl() {
    SG_INFO("ControlImpl destruct");
}

std::string ControlImpl::get_version() {
#ifdef PKG_VERSION
    return PKG_VERSION;
#else
    return "UNKNOWN";
#endif
}

bool ControlImpl::init(const ConfigFile &config_file) {
    SG_INFO("ControlImpl init");
    json_parser_.ParserVehicleConf(config_file.vehicle_parameter_path);
    json_parser_.ParserLatConf(config_file.lateral_controller_config_path);
    json_parser_.ParserLonConf(config_file.longitudinal_controller_config_path);
    json_parser_.ParserGearConf(config_file.gear_state_machine_conf_path);

    control_conf_.vehicle_params = json_parser_.GetVehicleParams();
    control_conf_.lat_conf = json_parser_.GetLatConf();
    control_conf_.lon_conf = json_parser_.GetLonConf();
    control_conf_.gear_conf = json_parser_.GetGearConf();

    SG_INFO(
        "ts:%lf,cf:%lf,cr:%lf,iz:%lf,hor:%d,nc:%d,r:%lf,q0:%lf,q1:%lf,q2:%lf",
        control_conf_.lat_conf.ts, control_conf_.vehicle_params.cf,
        control_conf_.vehicle_params.cr, control_conf_.vehicle_params.iz,
        control_conf_.lat_conf.horizon, control_conf_.lat_conf.controls_step,
        control_conf_.lat_conf.weights_r[0],
        control_conf_.lat_conf.weights_q[0],
        control_conf_.lat_conf.weights_q[1],
        control_conf_.lat_conf.weights_q[2]);

    lat_controller_ptr.reset(
        jarvis::control_lib::LatControllerInterface::CreateInstance());

    lat_controller_ptr->Init(&control_conf_);

    lon_controller_ptr.reset(
        jarvis::control_lib::LonControllerInterface::CreateInstance());

    lon_controller_ptr->Init(&control_conf_);

    state_machine_.Init(&control_conf_);
    return true;
}

bool ControlImpl::execute(ControlCmd *control_cmd, std::string &debuginfo) {
    // SG_INFO("-------------------------");

    // SG_INFO("x=%lf,y=%lf,dx=%lf,dy=%lf,yaw=%lf,is_update=%d",
    //         localization_msg_.x, localization_msg_.y, localization_msg_.dx,
    //         localization_msg_.dy, localization_msg_.yaw_rad,
    //         localization_msg_.is_update);

    // SG_INFO("vel=%lf,is_update=%d", speed_msg_.velocity,
    // speed_msg_.is_update);

    // SG_INFO("fl=%lf,fr=%lf,rl=%lf,rr=%lf,is_update=%d", wheel_msg_.fl,
    //         wheel_msg_.fr, wheel_msg_.rl, wheel_msg_.rr,
    //         wheel_msg_.is_update);

    // SG_INFO("steer_angle_rad=%lf,is_update=%d", steer_msg_.steer_angle_rad,
    //         steer_msg_.is_update);

    // SG_INFO("gear=%d,is_update=%d", gear_msg_.gear, gear_msg_.is_update);

    // SG_INFO("acc_x=%lf,acc_y=%lf,is_update=%d", acceleration_msg_.acc_x,
    //         acceleration_msg_.acc_y, acceleration_msg_.is_update);

    // SG_INFO("traj_size=%d,gear=%d,is_update=%d",
    //         trajectory_msg_.trajectory.size(), trajectory_msg_.gear,
    //         trajectory_msg_.is_update);

    // SG_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^");
    // 1.Update trajectory msg

    // for (int i = 0; i < trajectory_msg_.trajectory.size(); ++i) {
    SG_INFO("0 x = %lf,y = %lf,theta = %lf,s = %lf",
            trajectory_msg_.trajectory[0].path_point.x,
            trajectory_msg_.trajectory[0].path_point.y,
            trajectory_msg_.trajectory[0].path_point.theta_rad,
            trajectory_msg_.trajectory[0].path_point.s);
    // }

    double jerk;
    double acc = acceleration_msg_.acc_x;
    double t = acceleration_msg_.header.stamp;
    double dacc = acc - acc_pre_;
    double dt = t - t_pre_;

    if (acceleration_msg_.is_update == true && dt != 0) {
        jerk = dacc / dt;
    } else {
        jerk = jerk_pre_;
    }

    jerk_pre_ = jerk;
    acc_pre_ = acc;
    t_pre_ = t;

    debuginfo = "steer_real=" + std::to_string(steer_msg_.steer_angle_rad) +
                ";" + "acc_real=" + std::to_string(acceleration_msg_.acc_x) +
                ";" + "vel_real=" + std::to_string(speed_msg_.velocity) + ";" +
                "jerk=" + std::to_string(jerk) + ";" +
                "dot_y=" + std::to_string(localization_msg_.dy) + ";" +
                "yaw=" + std::to_string(localization_msg_.yaw_rad) + ";" +
                "v=" + std::to_string(speed_msg_.velocity) + ";" +
                "y=" + std::to_string(localization_msg_.y) + ";" +
                "x=" + std::to_string(localization_msg_.x) + ";" +
                "dot_yaw=" + std::to_string(localization_msg_.dot_yaw_radps) +
                ";";

    // 2.Call the lateral control algorithm
    SG_INFO("------lat_info------");
    lat_controller_ptr->ComputeControlCommand(localization_msg_, speed_msg_,
                                              steer_msg_, trajectory_msg_,
                                              control_cmd, debuginfo);

    // 3.Call the longitudinal control algorithm
    SG_INFO("------lon_info------");
    lon_controller_ptr->ComputeControlCommand(localization_msg_, speed_msg_,
                                              gear_msg_, trajectory_msg_,
                                              control_cmd, debuginfo);
    SG_INFO("lon end");
    // 4.Call the state machine
    std::string state_machine_info;
    state_machine_.Execute(speed_msg_, gear_msg_, trajectory_msg_, control_cmd,
                           debuginfo);

    count_control_++;

    double end_x = 4203.38;
    double end_y = 2534.15;
    double real_x = localization_msg_.x;
    double real_y = localization_msg_.y;
    // SG_INFO("real_x = %lf,real_y = %lf", real_x, real_y);
    double dis2end = std::hypot(real_x - end_x, real_y - end_y);
    if (dis2end < 10 && speed_msg_.velocity < 1 && count_control_ > 10000) {
        control_cmd->acc_cmd = -1;
        control_cmd->gear_cmd = GEAR::P;
    }

    if (control_cmd->steering_cmd_rad < 0.1 &&
        control_cmd->steering_cmd_rad > -0.1) {
        control_cmd->steering_cmd_rad = 0;
    }

    pre_control_cmd_ = *control_cmd;

    SG_INFO("steering_cmd = %lf,acc_cmd = %lf,gear_cmd = %d",
            control_cmd->steering_cmd_rad, control_cmd->acc_cmd,
            control_cmd->gear_cmd);

    localization_msg_.is_update = false;
    speed_msg_.is_update = false;
    wheel_msg_.is_update = false;
    steer_msg_.is_update = false;
    gear_msg_.is_update = false;
    acceleration_msg_.is_update = false;
    trajectory_msg_.is_update = false;

    return true;
}

bool ControlImpl::set_localization_msg(
    const LocalizationMsg &localization_msg) {
    localization_msg_ = localization_msg;
    return true;
}

bool ControlImpl::set_speed_msg(const SpeedMsg &speed_msg) {
    speed_msg_ = speed_msg;
    return true;
}
bool ControlImpl::set_wheel_msg(const WheelMsg &wheel_msg) {
    wheel_msg_ = wheel_msg;
    return true;
}
bool ControlImpl::set_steer_msg(const SteerMsg &steer_msg) {
    steer_msg_ = steer_msg;
    return true;
}
bool ControlImpl::set_gear_msg(const GearMsg &gear_msg) {
    gear_msg_ = gear_msg;
    return true;
}
bool ControlImpl::set_acceleration_msg(
    const AccelerationMsg &acceleration_msg) {
    acceleration_msg_ = acceleration_msg;
    return true;
}
bool ControlImpl::set_trajectory_msg(const TrajectoryMsg &trajectory_msg) {
    trajectory_msg_ = trajectory_msg;
    return true;
}

}  // namespace control_lib
}  // namespace jarvis
