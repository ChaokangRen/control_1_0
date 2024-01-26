#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <numeric>
#include <vector>

#include "common.h"
#include "digital_filter.h"
#include "lat_controller_interface.h"
#include "trajectory_gen.h"

namespace jarvis {
namespace control_lib {

class MpcController : public LatControllerInterface {
public:
    MpcController() = default;
    virtual ~MpcController(){};

    bool Init(const ControlConf *control_conf);

    bool ComputeControlCommand(const LocalizationMsg &localization_msg,
                               const SpeedMsg &speed_msg,
                               const SteerMsg &steer_msg,
                               const TrajectoryMsg &trajectory_msg,
                               ControlCmd *control_cmd,
                               std::string &control_info);

    bool Reset() override;

private:
    void UpdateBaseMatrix();

    void UpdatePredictMatrix();

    void ConvertToStandardQuadProgMatrix();

    void UpdateConstriant();

    bool QpOaeseSlover(double *steering_cmd);

    void LimitSteering(double *steering_in);

private:
    // control time int32_terval
    double ts_ = 0.0;
    // corner stiffness: front
    double cf_ = 0.0;
    // corner stiffness: rear
    double cr_ = 0.0;
    // vehicle mass
    double mass_ = 0.0;
    // distance between front and rear wheel center
    double wheelbase_ = 0.0;
    // distance from front wheel center to COM
    double lf_ = 0.0;
    // distance from rear wheel center to COM
    double lr_ = 0.0;
    // rotational inertia
    double iz_ = 0.0;
    // ther ratio betweent the turn of the steering wheel and the turn of the
    // wheels
    double steer_ratio_ = 0.0;

    double compensation_angle_rad_ = 0.0;

    double max_lat_acc_ = 0.0;

    const int32_t state_size_ = 5;

    const int32_t ctrl_size_ = 1;

    const int32_t ref_size_ = 3;

    const int32_t aug_size_ = 6;

    int32_t controls_ = 3;

    int32_t horizon_ = 10;

    // vheicle state matrix
    Eigen::MatrixXd matrix_a_;
    // vheicle state matrix (discerte matrix)
    Eigen::MatrixXd matrix_ad_;
    // vheicle state matrix(Augmented matrix)
    Eigen::MatrixXd matrix_ag_;
    // vheicle predict state matrix
    Eigen::MatrixXd matrix_a_pred_;

    // vheicle control matrix
    Eigen::MatrixXd matrix_b_;
    // vheicle state matrix (discerte matrix)
    Eigen::MatrixXd matrix_bd_;
    // vheicle control matrix(Augmented matrix)
    Eigen::MatrixXd matrix_bg_;
    // vheicle predict state matrix
    Eigen::MatrixXd matrix_b_pred_;

    // vehicle complement matrix
    Eigen::MatrixXd matrix_c_;
    // vheicle state matrix (discerte matrix)
    Eigen::MatrixXd matrix_cd_;
    // vheicle complement matrix(Augmented matrix)
    Eigen::MatrixXd matrix_cg_;
    // vheicle predict state matrix
    Eigen::MatrixXd matrix_c_pred_;
    // trajectoyr matrix
    Eigen::MatrixXd matrix_traj_;

    // control authority weighting matrix
    Eigen::MatrixXd matrix_r_;
    // state weighting matrix
    Eigen::MatrixXd matrix_q_;

    Eigen::MatrixXd matrix_d_;

    Eigen::MatrixXd matrix_r_pred_;
    Eigen::MatrixXd matrix_q_pred_;
    Eigen::MatrixXd matrix_d_pred_;

    Eigen::MatrixXd matrix_dqd_;
    Eigen::MatrixXd matrix_qd_;

    Eigen::MatrixXd matrix_h_;
    Eigen::MatrixXd matrix_g_;

    // max steering value
    double max_steering_;
    // min steering value
    double min_steering_;
    // max delta steering value
    double max_delta_steering_;
    // min delta steering value
    double min_delta_steering_;

    // state vector: diff_y,phi,diff_phi,Y,X
    Eigen::MatrixXd kesi_;
    double dx_;
    double dy_;
    double phi_;
    double dphi_;

    // last output
    double steering_ = 0.0;
    double delta_steering_ = 0.0;

    double eps_;

    // Equality constraints
    Constraint equ_constraint_;
    // Inequality constraint
    Constraint inequ_constraint_;

    DigitalFilter filter_;
    // Ratio of steering wheel to front wheel Angle
    double trajectory_min_dist_ = 3.0;

    vector<double> curv_;
};
}  // namespace control_lib

}  // namespace jarvis