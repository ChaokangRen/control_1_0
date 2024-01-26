#include <ros/ros.h>
#include <sglog/sglog.h>

#include "control/common_struct.h"
#include "control/control_interface.h"
#include "mpc_control.h"
#include "sg_sensor_msgs/Inspva.h"
#include "sg_vehicle_msgs/BrakeCmd.h"
#include "sg_vehicle_msgs/GearCmd.h"
#include "sg_vehicle_msgs/GearReport.h"
#include "sg_vehicle_msgs/Imu.h"
#include "sg_vehicle_msgs/ModeCmd.h"
#include "sg_vehicle_msgs/Speed.h"
#include "sg_vehicle_msgs/SteeringCmd.h"
#include "sg_vehicle_msgs/SteeringReport.h"
#include "sg_vehicle_msgs/ThrottleCmd.h"
#include "trajectory_sim.h"

using namespace jarvis::control_lib;

typedef std::shared_ptr<jarvis::control_lib::ControlInterface>
    ControlInterfacePtr;

LocalizationMsg localization_msg;
ChassisMsg chassis_msg;
TrajectoryMsg trajectory_msg;
std::string debuginfo;
std::string traj_info;

inline double AngleToRadian(double yaw_degree) {
    return yaw_degree * 3.14159265358979323846 / 180;
}

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

void Wgs84ToLocalCoord(const double longitude, const double latitude,
                       const double altitude, double *local_x, double *local_y,
                       double *local_z) {
    double earth_radius_p =
        earth_radius *
        (1 + oblateness * std::sin(latitude) * std::sin(latitude));
    double spre_coord_x =
        (earth_radius_p + altitude) * std::cos(latitude) * std::cos(longitude);
    double spre_coord_y =
        (earth_radius_p + altitude) * std::cos(latitude) * std::sin(longitude);
    double spre_coord_z =
        ((1 - first_eccentricity * first_eccentricity) * earth_radius_p +
         altitude) *
        std::sin(latitude);

    *local_x = -std::sin(orgin_longitude) * (spre_coord_x - orgin_x) +
               std::cos(orgin_longitude) * (spre_coord_y - orgin_y);
    *local_y = -std::sin(orgin_latitude) * std::cos(orgin_longitude) *
                   (spre_coord_x - orgin_x) -
               std::sin(orgin_latitude) * std::sin(orgin_longitude) *
                   (spre_coord_y - orgin_y) +
               std::cos(orgin_latitude) * (spre_coord_z - orgin_z);
    *local_z = std::cos(orgin_latitude) * std::cos(orgin_longitude) *
                   (spre_coord_x - orgin_x) -
               std::cos(orgin_latitude) * std::sin(orgin_longitude) *
                   (spre_coord_y - orgin_y) +
               std::sin(orgin_latitude) * (spre_coord_z - orgin_z);
}

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

void ChatterCallbackInspva(
    const sg_sensor_msgs::InspvaConstPtr &vehicle_inspva_ptr) {
    static double pre_enu_yaw = 0;
    static auto pre_time = ros::Time::now();
    static double pre_enu_x = 0;
    static double pre_enu_y = 0;

    // 1.Convert latitude and longitude coordinates to ENU coordinates.
    double latitude = AngleToRadian(vehicle_inspva_ptr->latitude);
    double longtitude = AngleToRadian(vehicle_inspva_ptr->longitude);
    double altitude = AngleToRadian(vehicle_inspva_ptr->altitude);
    double enu_x = 0, enu_y = 0, enu_z = 0;

    Wgs84ToLocalCoord(longtitude, latitude, altitude, &enu_x, &enu_y, &enu_z);

    localization_msg.x = enu_x;
    localization_msg.y = enu_y;
    localization_msg.z = enu_z;

    // 2. Calculate the yaw, dot_yaw in ENU coordinates.
    double enu_yaw = AngleToRadian(vehicle_inspva_ptr->yaw) + M_PI / 2;
    auto time_now = ros::Time::now();

    double delta_t = (time_now - pre_time).toSec();
    if (std::abs(delta_t - 0.01) < 0.05) delta_t = 0.01;

    double dot_enu_y = (enu_y - pre_enu_y) / delta_t;
    double dot_enu_yaw = (enu_yaw - pre_enu_yaw) / delta_t;

    localization_msg.yaw = enu_yaw;
    localization_msg.dot_yaw = dot_enu_yaw;

    // 3. Calculate the lateral and longitudinal speed of the vehicle.
    double delta_x = enu_x - pre_enu_x;
    double delta_y = enu_y - pre_enu_y;
    double diff_yaw = enu_yaw - pre_enu_yaw;

    double dx = delta_x / delta_t;
    double dy = delta_y / delta_t;
    double speed = std::sqrt(dx * dx + dy * dy);

    localization_msg.dx = speed * std::cos(diff_yaw);
    localization_msg.dy = speed * std::sin(diff_yaw);

    pre_enu_yaw = enu_yaw;
    pre_enu_y = enu_y;
    pre_time = time_now;
    enu_x = enu_x;
    enu_y = enu_y;

    localization_msg.is_update = true;
}

void ChatterCallbackSpeed(
    const sg_vehicle_msgs::SpeedConstPtr &vehicle_speed_ptr) {
    chassis_msg.velocity = vehicle_speed_ptr->mps;
    chassis_msg.is_update = true;
}

void ChatterCallbackSteer(
    const sg_vehicle_msgs::SteeringReportConstPtr &vehicle_steer_ptr) {
    chassis_msg.steer_angle = AngleToRadian(vehicle_steer_ptr->angle_actual);
}

void ChatterCallbackGear(const sg_vehicle_msgs::GearReport &gear_ptr) {
    if (gear_ptr.actual == sg_vehicle_msgs::GearReport::GEAR_NONE) {
        chassis_msg.gear = GEAR::NONE;
    } else if (gear_ptr.actual == sg_vehicle_msgs::GearReport::GEAR_PARK) {
        chassis_msg.gear = GEAR::P;
    } else if (gear_ptr.actual == sg_vehicle_msgs::GearReport::GEAR_DRIVE) {
        chassis_msg.gear = GEAR::D;
    } else if (gear_ptr.actual == sg_vehicle_msgs::GearReport::GEAR_REVERSE) {
        chassis_msg.gear = GEAR::R;
    } else if (gear_ptr.actual == sg_vehicle_msgs::GearReport::GEAR_NEUTRAL) {
        chassis_msg.gear = GEAR::N;
    }
}

void DBWEnable(const ros::Publisher &automode_pub,
               const ros::Publisher &break_pub,
               const ros::Publisher &throttle_pub,
               const ros::Publisher &steering_pub,
               const ros::Publisher &gear_pub,
               sg_vehicle_msgs::BrakeCmd &break_cmd,
               sg_vehicle_msgs::ModeCmd &mode_cmd,
               sg_vehicle_msgs::ThrottleCmd &throttle_cmd,
               sg_vehicle_msgs::SteeringCmd &steer_cmd,
               sg_vehicle_msgs::GearCmd &gear_cmd) {
    double sleep_time = 10 * 1000;
    int32_t cycle_times = 10;

    break_cmd.command_type = 2;
    break_cmd.command = 40;
    break_pub.publish(break_cmd);

    throttle_cmd.command_type = 2;
    throttle_cmd.command = 0;
    throttle_pub.publish(throttle_cmd);

    steer_cmd.command_type = 1;
    steer_cmd.command = 0;
    steering_pub.publish(steer_cmd);

    for (int i = 0; i < cycle_times; ++i) {
        break_cmd.command = 40;
        break_pub.publish(break_cmd);
        mode_cmd.enable = true;
        automode_pub.publish(mode_cmd);
        usleep(sleep_time);
    }
    for (int i = 0; i < cycle_times; ++i) {
        break_cmd.command = 40;
        break_pub.publish(break_cmd);
        gear_cmd.command = 4;
        gear_pub.publish(gear_cmd);
        usleep(sleep_time);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "control_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    // 1. Initialize the subscribe topic
    ros::Subscriber inspva_sub =
        nh.subscribe("/sensor/ins/fusion", 5, ChatterCallbackInspva);
    ros::Subscriber speed_sub =
        nh.subscribe("/vehicle/chassis/vehicle_speed", 5, ChatterCallbackSpeed);
    ros::Subscriber gear_sub =
        nh.subscribe("/vehicle/gear/report", 5, ChatterCallbackGear);
    ros::Subscriber steer_sub =
        nh.subscribe("/vehicle/steering/report", 5, ChatterCallbackSteer);

    // 2. Initialize the  publisher topic.
    ros::Publisher steering_pub = nh.advertise<sg_vehicle_msgs::SteeringCmd>(
        "/vehicle/steering/command", 1);
    ros::Publisher break_pub =
        nh.advertise<sg_vehicle_msgs::BrakeCmd>("/vehicle/brake/command", 10);
    ros::Publisher automode_pub =
        nh.advertise<sg_vehicle_msgs::ModeCmd>("/vehicle/mode/command", 10);
    ros::Publisher gear_pub =
        nh.advertise<sg_vehicle_msgs::GearCmd>("/vehicle/gear/command", 10);
    ros::Publisher throttle_pub = nh.advertise<sg_vehicle_msgs::ThrottleCmd>(
        "/vehicle/throttle/command", 10);

    sg_vehicle_msgs::BrakeCmd break_cmd;
    sg_vehicle_msgs::ModeCmd mode_cmd;
    sg_vehicle_msgs::ThrottleCmd throttle_cmd;
    sg_vehicle_msgs::SteeringCmd steer_cmd;
    sg_vehicle_msgs::GearCmd gear_cmd;

    ros::Rate loop_rate(100);
    ControlCmd control_cmd;

    // 3. Initialize the control component.
    ControlInterfacePtr control_interface_ptr;
    control_interface_ptr.reset(
        jarvis::control_lib::ControlInterface::CreateInstance());

    SG_INFO("version: %s", control_interface_ptr->GetVersion().c_str());

    // 4. DBW enable
    DBWEnable(automode_pub, break_pub, throttle_pub, steering_pub, gear_pub,
              break_cmd, mode_cmd, throttle_cmd, steer_cmd, gear_cmd);
    TrajecotrySim trajectory_generator;
    while (ros::ok) {
        trajectory_msg = trajectory_generator.UpdateTrajectory(
            localization_msg.x, localization_msg.y);

        // 5. Execute control component
        if (localization_msg.is_update) {
            localization_msg.is_update = false;
            if (!trajectory_msg.trajectory.empty()) {
                control_interface_ptr->Execute(localization_msg, chassis_msg,
                                               trajectory_msg, &control_cmd,
                                               &debuginfo);
            }
        }
        // 6. Publisher control command.
        break_cmd.command = control_cmd.brake_cmd;
        throttle_cmd.command = control_cmd.throttle_cmd;
        steer_cmd.command = control_cmd.steering_cmd;
        gear_cmd.command = GearCommandValue(control_cmd.gear_cmd);

        break_pub.publish(break_cmd);
        throttle_pub.publish(throttle_cmd);
        steering_pub.publish(steer_cmd);
        gear_pub.publish(gear_cmd);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}