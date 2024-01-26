#include "mpc_controller.h"

#include <iostream>
#include <qpOASES.hpp>

#include "digital_filter_coefficients.h"
#include "runge_kutta.h"
#include "sglog/sglog.h"
#include "trajectory_analyzer.h"
using Eigen::MatrixXd;
namespace jarvis {
namespace control_lib {
USING_NAMESPACE_QPOASES

LatControllerInterface *LatControllerInterface::CreateInstance() {
    return new MpcController();
}

bool MpcController::Init(const ControlConf *control_conf) {
    SG_INFO("MpcController Init!");
    LoadRefline();
    // 1. Initialize MPC controller parameters
    ts_ = control_conf->lat_conf.ts;
    cf_ = control_conf->vehicle_params.cf;
    cr_ = control_conf->vehicle_params.cr;
    iz_ = control_conf->vehicle_params.iz;

    controls_ = control_conf->lat_conf.controls_step;
    horizon_ = control_conf->lat_conf.horizon;
    eps_ = control_conf->lat_conf.eps;
    max_steering_ = control_conf->lat_conf.max_steering;
    min_steering_ = control_conf->lat_conf.min_steering;
    max_delta_steering_ = control_conf->lat_conf.max_delta_steering;
    min_delta_steering_ = control_conf->lat_conf.min_delta_steering;
    steer_ratio_ = control_conf->vehicle_params.steer_ratio;
    compensation_angle_rad_ = control_conf->lat_conf.compensation_angle_rad;

    matrix_q_ = MatrixXd::Identity(ref_size_, ref_size_);
    matrix_r_ = MatrixXd::Identity(ctrl_size_, ctrl_size_);
    if (control_conf->lat_conf.weights_q.size() != ref_size_) {
        SG_ERROR("size of control_conf_mpc_conf_weights_q is wrong");
    }
    if (control_conf->lat_conf.weights_r.size() != ctrl_size_) {
        SG_ERROR("size of control_conf_mpc_conf_weights_r is wrong");
    }
    for (int32_t i = 0; i < ref_size_; ++i) {
        matrix_q_(i, i) = control_conf->lat_conf.weights_q[i];
    }
    for (int32_t i = 0; i < ctrl_size_; ++i) {
        matrix_r_(i, i) = control_conf->lat_conf.weights_r[i];
    }

    wheelbase_ = control_conf->vehicle_params.wheelbase;
    const double mass_fl = control_conf->vehicle_params.mass_fl;
    const double mass_fr = control_conf->vehicle_params.mass_fr;
    const double mass_rl = control_conf->vehicle_params.mass_rl;
    const double mass_rr = control_conf->vehicle_params.mass_rr;

    const double mass_front = mass_fl + mass_fr;
    const double mass_rear = mass_rl + mass_rr;
    mass_ = mass_front + mass_rear;
    lf_ = wheelbase_ * (1.0 - mass_front / mass_);
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

    SG_INFO("lf = %lf,lr = %lf,iz = %lf,ccf = %lf,ccr_ = %lf,mass = %lf", lf_,
            lr_, iz_, cf_, cr_, mass_);
    // 2.Initialize the MPC controller correlation matrix
    matrix_a_ = MatrixXd::Zero(state_size_, state_size_);
    matrix_b_ = MatrixXd::Zero(state_size_, ctrl_size_);
    matrix_c_ = MatrixXd::Zero(state_size_, 1);

    matrix_ag_ = MatrixXd::Zero(aug_size_, aug_size_);
    matrix_bg_ = MatrixXd::Zero(aug_size_, ctrl_size_);
    matrix_cg_ = MatrixXd::Zero(aug_size_, 1);

    matrix_a_pred_ = MatrixXd::Zero(aug_size_ * horizon_, aug_size_);
    matrix_b_pred_ =
        MatrixXd::Zero(aug_size_ * horizon_, ctrl_size_ * controls_);
    matrix_c_pred_ = MatrixXd::Zero(aug_size_ * horizon_, 1);

    matrix_q_pred_ = MatrixXd::Zero(ref_size_ * horizon_, ref_size_ * horizon_);
    matrix_r_pred_ =
        MatrixXd::Zero(ctrl_size_ * controls_, ctrl_size_ * controls_);

    for (int32_t i = 0; i < horizon_; ++i) {
        matrix_q_pred_.block(i * ref_size_, i * ref_size_, ref_size_,
                             ref_size_) = matrix_q_;
    }
    for (int32_t i = 0; i < controls_; ++i) {
        matrix_r_pred_.block(i * ctrl_size_, i * ctrl_size_, ctrl_size_,
                             ctrl_size_) = matrix_r_;
    }

    matrix_d_ = MatrixXd::Zero(ref_size_, aug_size_);
    matrix_d_(0, 3) = 1;
    matrix_d_(1, 4) = 1;
    matrix_d_(2, 1) = 1;

    matrix_d_pred_ = MatrixXd::Zero(ref_size_ * horizon_, aug_size_ * horizon_);
    for (int32_t i = 0; i < horizon_; ++i) {
        matrix_d_pred_.block(i * ref_size_, i * aug_size_, ref_size_,
                             aug_size_) = matrix_d_;
    }
    matrix_qd_ = matrix_q_pred_ * matrix_d_pred_;
    matrix_dqd_ = matrix_d_pred_.transpose() * matrix_qd_;

    matrix_traj_ = MatrixXd::Zero(1, ref_size_ * horizon_);
    kesi_ = MatrixXd::Zero(state_size_, 1);

    for (int32_t i = 0; i < controls_; ++i) {
        equ_constraint_.lower_bound.emplace_back(min_delta_steering_);
        equ_constraint_.upper_bound.emplace_back(max_delta_steering_);
        std::vector<double> cross_vec(controls_, 0);
        cross_vec[i] = 1;
        equ_constraint_.constraint_mat.emplace_back(cross_vec);

        inequ_constraint_.lower_bound.emplace_back(min_steering_ - steering_);
        inequ_constraint_.upper_bound.emplace_back(max_steering_ - steering_);
        for (int32_t j = 0; j < i - 1; ++j) {
            cross_vec[j] = 1;
        }
        inequ_constraint_.constraint_mat.emplace_back(cross_vec);
    }
    // Initialize the digital filter
    std::vector<double> denominators;
    std::vector<double> numerators;
    LpfCoefficients(ts_, control_conf->lat_conf.cutoff_freq, &denominators,
                    &numerators);
    filter_.SetCoefficients(denominators, numerators);
    filter_.SetCoefficients(denominators, numerators);
    return true;
}

void MpcController::UpdateBaseMatrix() {
    double cos_phi = std::cos(phi_);
    double sin_phi = std::sin(phi_);
    // 1.Update matrix a
    matrix_a_(0, 0) = -(2 * cf_ + 2 * cr_) / (mass_ * dx_);
    matrix_a_(0, 1) = -dx_ + 2 * (lr_ * cr_ - lf_ * cf_) / (mass_ * dx_);
    matrix_a_(1, 2) = 1;
    matrix_a_(2, 0) = (2 * lr_ * cr_ - 2 * lf_ * cf_) / (iz_ * dx_);
    matrix_a_(2, 1) =
        -(2 * lf_ * lf_ * cf_ + 2 * lr_ * lr_ * cr_) / (iz_ * dx_);
    matrix_a_(3, 0) = cos_phi;
    matrix_a_(3, 1) = dx_ * cos_phi - dy_ * sin_phi;
    matrix_a_(4, 0) = -sin_phi;
    matrix_a_(4, 1) = -dy_ * sin_phi - dx_ * cos_phi;
    // 2. Update matrix b
    matrix_b_(0, 0) = 2 * cf_ / mass_;
    matrix_b_(2, 0) = 2 * lf_ * cf_ / iz_;

    // 3. Update matrix c
    double temp_formula1 = steering_ - (dy_ + lf_ * dphi_) / dx_;
    double temp_formula2 = (lr_ * dphi_ - dy_) / dx_;
    matrix_c_(0, 0) =
        -dx_ * dphi_ + 2 * (cf_ * temp_formula1 + cr_ * temp_formula2) / mass_;
    matrix_c_(1, 0) = dphi_;
    matrix_c_(2, 0) =
        2 * (lf_ * cf_ * temp_formula1 - lr_ * cr_ * temp_formula2) / iz_;
    matrix_c_(3, 0) = dx_ * sin_phi + dy_ * cos_phi;
    matrix_c_(4, 0) = dx_ * cos_phi - dy_ * sin_phi;

    // matrix_c_ = matrix_c_ - matrix_a_ * kesi_ - matrix_b_ * steering_;

    Eigen::MatrixXd kesi_tmp = kesi_;
    // kesi_tmp(3, 0) = kesi_(4, 0);
    // kesi_tmp(4, 0) = kesi_(3, 0);
    matrix_c_ = matrix_c_ - matrix_a_ * kesi_tmp + matrix_b_ * steering_;

    // 4.Update discrete matrix
    matrix_ad_ = MatrixXd::Identity(state_size_, state_size_) + ts_ * matrix_a_;
    matrix_bd_ = ts_ * matrix_b_;
    matrix_cd_ = ts_ * matrix_c_;

    // 5.Update augmented matrix
    matrix_ag_.block(0, 0, state_size_, state_size_) = matrix_ad_;
    matrix_ag_.block(0, state_size_, state_size_, ctrl_size_) = matrix_bd_;
    matrix_ag_.block(state_size_, state_size_, ctrl_size_, ctrl_size_) =
        MatrixXd::Identity(ctrl_size_, ctrl_size_);

    matrix_bg_.block(0, 0, state_size_, ctrl_size_) = matrix_bd_;
    matrix_bg_.block(state_size_, 0, ctrl_size_, ctrl_size_) =
        MatrixXd::Identity(ctrl_size_, ctrl_size_);
    matrix_cg_.block(0, 0, state_size_, 1) = matrix_cd_;
}
void MpcController::UpdatePredictMatrix() {
    MatrixXd matrix_ag_tmp = matrix_ag_;
    MatrixXd matrix_cg_tmp = matrix_cg_;

    MatrixXd zeros_tmp = MatrixXd::Zero(aug_size_, ctrl_size_);
    // 1. Update prediction matrix A,B,C
    for (int32_t i = 0; i < horizon_; ++i) {
        matrix_a_pred_.block(i * aug_size_, 0, aug_size_, aug_size_) =
            matrix_ag_tmp;
        matrix_ag_tmp *= matrix_ag_;

        for (int32_t j = 0; j < controls_; ++j) {
            if (j == i) {
                matrix_b_pred_.block(i * aug_size_, j * ctrl_size_, aug_size_,
                                     ctrl_size_) = matrix_bg_;
            } else if (j > i) {
                matrix_b_pred_.block(i * aug_size_, j * ctrl_size_, aug_size_,
                                     ctrl_size_) = zeros_tmp;
            } else if (j < i) {
                if (j == 0) {
                    matrix_b_pred_.block(i * aug_size_, j * ctrl_size_,
                                         aug_size_, ctrl_size_) =
                        matrix_ag_ * matrix_b_pred_.block(
                                         (i - 1) * aug_size_, j * ctrl_size_,
                                         aug_size_, ctrl_size_);
                } else if (j != 0) {
                    matrix_b_pred_.block(i * aug_size_, j * ctrl_size_,
                                         aug_size_, ctrl_size_) =
                        matrix_b_pred_.block((i - 1) * aug_size_,
                                             (j - 1) * ctrl_size_, aug_size_,
                                             ctrl_size_);
                }
            }
        }
        matrix_c_pred_.block(i * aug_size_, 0, aug_size_, 1) = matrix_cg_tmp;
        matrix_cg_tmp = matrix_ag_ * matrix_cg_tmp + matrix_cg_;
    }
}
void MpcController::UpdateConstriant() {
    for (int32_t i = 0; i < controls_; ++i) {
        inequ_constraint_.lower_bound[i] = min_steering_ - steering_;
        inequ_constraint_.upper_bound[i] = max_steering_ - steering_;
    }
}

void MpcController::ConvertToStandardQuadProgMatrix() {
    matrix_h_ = 2 * (matrix_b_pred_.transpose() * matrix_dqd_ * matrix_b_pred_ +
                     matrix_r_pred_);
    MatrixXd kesi_aug_tran = MatrixXd::Zero(1, aug_size_);
    for (int32_t i = 0; i < aug_size_; ++i) {
        if (i < state_size_) {
            kesi_aug_tran(0, i) = kesi_(i, 0);
        } else {
            kesi_aug_tran(0, i) = steering_;
        }
    }
    matrix_g_ = 2 *
                ((kesi_aug_tran * matrix_a_pred_.transpose() +
                  matrix_c_pred_.transpose()) *
                     matrix_dqd_ -
                 matrix_traj_ * matrix_qd_) *
                matrix_b_pred_;
}

bool MpcController::ComputeControlCommand(
    const LocalizationMsg &localization_msg, const SpeedMsg &speed_msg,
    const SteerMsg &steer_msg, const TrajectoryMsg &trajectory_msg,
    ControlCmd *control_cmd, std::string &debug_info) {
    // trajectory length too small
    if (trajectory_msg.trajectory.empty() ||
        (trajectory_msg.trajectory.back().path_point.s <= 5)) {
        control_cmd->steering_cmd_rad = steer_msg.steer_angle_rad;
        debug_info =
            debug_info +
            "steer_cmd=" + std::to_string(control_cmd->steering_cmd_rad) + ";" +
            "dsteer_cmd=" + std::to_string(0.0) + ";" +
            "ey=" + std::to_string(0.0) + ";" + "lat0=" + std::to_string(0.0) +
            ";" + "lat1=" + std::to_string(0.0) + ";";
        return true;
    }

    double c_yaw = localization_msg.yaw_rad;
    TrajecotoryAnalyzer trajectory_analyzer(trajectory_msg);
    double error = 0;
    bool has_enough_horizon = true;
    std::vector<std::tuple<double, double, double>> ref_trajectory =
        trajectory_analyzer.MpcReferenceTrajectory(
            horizon_, ts_, localization_msg.x, localization_msg.y,
            speed_msg.velocity, error, has_enough_horizon);

    // for (int i = 0; i < ref_trajectory.size(); ++i) {
    //     SG_INFO("%d,ref_y=%lf,ref_x=%lf,ref_yaw=%lf", i,
    //             std::get<0>(ref_trajectory[i]),
    //             std::get<1>(ref_trajectory[i]),
    //             std::get<2>(ref_trajectory[i]));
    // }
    double ref_yaw = std::get<2>(ref_trajectory[0]);

    if (c_yaw - ref_yaw > M_PI) {
        c_yaw -= 2 * M_PI;
    } else if (c_yaw - ref_yaw < -M_PI) {
        c_yaw += 2 * M_PI;
    }
    kesi_(0, 0) = localization_msg.dy;
    // kesi_(1, 0) = localization_msg.yaw;
    kesi_(1, 0) = c_yaw;
    kesi_(2, 0) = localization_msg.dot_yaw_radps;
    kesi_(3, 0) = localization_msg.y;
    kesi_(4, 0) = localization_msg.x;

    dx_ = localization_msg.dx;
    if (dx_ < 1.0) dx_ = 1.0;

    dy_ = localization_msg.dy;
    phi_ = c_yaw;
    dphi_ = localization_msg.dot_yaw_radps;
    steering_ = steer_msg.steer_angle_rad / steer_ratio_;

    double preview_dist = 0.0;
    preview_dist = std::fmax(2.0, 0.1 * dx_);
    // if (dx_ > 12) {
    //     preview_dist = std::fmax(1.5, 0.05 * dx_);
    // } else if (dx_ < 1.0) {
    //     preview_dist = 0;
    // } else {
    //     preview_dist = std::fmax(3.5, 0.2 * dx_);
    // }
    // if(dx_ > 14){
    //      preview_dist = std::fmax(1.0, 0.1 * dx_);
    // }else if(dx <= 14 && dx_ > 10){
    //     preview_dist = std::fmax(1.5, 0.1 * dx_);
    // }
    // else{
    //     preview_dist = std::fmax(3.5, 0.1 * dx_);
    // }
    // if (dx_ <= 1.001) {
    //     preview_dist = 0.5;
    // }
    // preview_dist = 3.5;

    // std::vector<double> kesi_state(6, 0);
    // kesi_state[0] = dx_;
    // kesi_state[1] = localization_msg.dy;
    // kesi_state[2] = c_yaw;
    // kesi_state[3] = localization_msg.dot_yaw;
    // kesi_state[4] = localization_msg.y;
    // kesi_state[5] = localization_msg.x;
    // double delta_time = preview_dist / dx_;
    // RungeKutta runge_kutta;
    // kesi_state = runge_kutta.Slover(kesi_state, steering_, 0,
    // delta_time); kesi_(3, 0) = kesi_state[4]; kesi_(4, 0) =
    // kesi_state[5];

    double r_curv = dx_ / dphi_;
    if (r_curv > 0) {
        curv_.emplace_back(r_curv);
        if (curv_.size()) {
            double sum = accumulate(begin(curv_), end(curv_), 0.0);
            double mean = sum / curv_.size();
        }
    }
    // update trajectory matrix

    if (ref_trajectory.size() != horizon_) {
        // SG_ERROR("length of trajectory less than horizon!");
        return false;
    }
    const std::vector<TrajectoryPoint> &traj = trajectory_msg.trajectory;

    for (int32_t i = 0; i < horizon_; ++i) {
        matrix_traj_(0, i * ref_size_) = std::get<0>(ref_trajectory[i]);
        matrix_traj_(0, i * ref_size_ + 1) = std::get<1>(ref_trajectory[i]);
        matrix_traj_(0, i * ref_size_ + 2) = std::get<2>(ref_trajectory[i]);
    }
    double x0 = std::get<1>(ref_trajectory[0]);
    double y0 = std::get<0>(ref_trajectory[0]);
    double x1 = std::get<1>(ref_trajectory[1]);
    double y1 = std::get<0>(ref_trajectory[1]);
    double x2 = std::get<1>(ref_trajectory[2]);
    double y2 = std::get<0>(ref_trajectory[2]);
    double curvature =
        (std::atan2((y2 - y1), (x2 - x1)) - std::atan2((y1 - y0), (x1 - x0))) /
        (std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)));
    double theta_error = std::get<2>(ref_trajectory[0]) - c_yaw;
    double dx = std::get<1>(ref_trajectory[5]) - std::get<1>(ref_trajectory[0]);
    double dy = std::get<0>(ref_trajectory[5]) - std::get<0>(ref_trajectory[0]);
    double theta_atan = std::atan2(dy, dx);

    double x_error = kesi_(4, 0) - x0;
    double y_error = kesi_(3, 0) - y0;
    SG_INFO("x_error = %lf,y_error = %lf", x_error, y_error);
    SG_INFO("x = %lf,ref x = %lf,y = %lf,ref y = %lf", kesi_(4, 0), x0,
            kesi_(3, 0), y0);

    //  use new mpc controller
    // SG_INFO("theta error = %lf,ref yaw = %lf,real_yaw = %lf", theta_error,
    //         std::get<2>(ref_trajectory[0]), c_yaw);
    if (has_enough_horizon == true &&
        trajectory_msg.trajectory.back().path_point.s > trajectory_min_dist_) {
        // update mpc matrix
        UpdateBaseMatrix();
        UpdatePredictMatrix();
        ConvertToStandardQuadProgMatrix();
        UpdateConstriant();
        QpOaeseSlover(&steering_);

        control_cmd->steering_cmd_rad =
            (steering_ + compensation_angle_rad_) * steer_ratio_;
        // SG_INFO("steering = %lf,compensation_angle = %lf", steering_,
        //         compensation_angle_rad_);

    } else {
        control_cmd->steering_cmd_rad = steer_msg.steer_angle_rad;
        // SG_WARN("have no enough length");
    }

    SG_INFO("theta_error=%lf,y_error=%lf", theta_error, error);

    debug_info =
        debug_info +
        "steer_cmd=" + std::to_string(control_cmd->steering_cmd_rad) + ";" +
        "dsteer_cmd=" + std::to_string(delta_steering_ * steer_ratio_) + ";" +
        "ey=" + std::to_string(error) + ";" + "lat0=" + std::to_string(0.0) +
        ";" + "lat1=" + std::to_string(0.0) + ";";
    return true;
}
void MpcController::LimitSteering(double *steering_in) {
    if (*steering_in > max_steering_) {
        *steering_in = max_steering_;
    } else if (*steering_in < min_steering_) {
        *steering_in = min_steering_;
    }
}
bool MpcController::QpOaeseSlover(double *steering_cmd) {
    real_t qp_h[controls_ * controls_];
    real_t qp_g[controls_];

    for (int32_t i = 0; i < controls_; ++i) {
        for (int32_t j = 0; j < controls_; ++j) {
            qp_h[i * controls_ + j] = matrix_h_(i, j);
        }
        qp_g[i] = matrix_g_(0, i);
    }

    real_t qp_lb[controls_];
    real_t qp_ub[controls_];
    for (int32_t i = 0; i < controls_; ++i) {
        qp_lb[i] = equ_constraint_.lower_bound[i];
        qp_ub[i] = equ_constraint_.upper_bound[i];
    }

    int32_t inequ_num = inequ_constraint_.lower_bound.size();
    // inequ_num = 0;
    real_t qp_a[inequ_num * controls_];
    real_t qp_lba[inequ_num];
    real_t qp_uba[inequ_num];
    const std::vector<std::vector<double>> &inequ_mat =
        inequ_constraint_.constraint_mat;

    for (int32_t i = 0; i < inequ_num; ++i) {
        for (int32_t j = 0; j < controls_; ++j) {
            qp_a[i * inequ_num + j] = inequ_mat[i][j];
        }
        qp_lba[i] = inequ_constraint_.lower_bound[i];
        qp_uba[i] = inequ_constraint_.upper_bound[i];
    }
    QProblem slover(controls_, inequ_num);
    Options options;
    options.printLevel = PL_NONE;
    slover.setOptions(options);
    int_t nWSR = 1000;
    if (SUCCESSFUL_RETURN !=
        slover.init(qp_h, qp_g, qp_a, qp_lb, qp_ub, qp_lba, qp_uba, nWSR)) {
        // SG_ERROR("MPC_QP_FAILED!!!!!!!!!");
    };
    real_t x_opt[controls_];
    slover.getPrimalSolution(x_opt);
    delta_steering_ = x_opt[0];
    delta_steering_ = filter_.Filter(delta_steering_);

    *steering_cmd = steering_ + delta_steering_;
    SG_INFO("steer = %lf,delta_steer = %lf", steering_, delta_steering_);
    // steering_ = *steering_cmd;
    LimitSteering(steering_cmd);

    return true;
}
bool MpcController::Reset() {
    return true;
}

}  // namespace control_lib
}  // namespace jarvis