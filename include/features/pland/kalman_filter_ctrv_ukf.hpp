#pragma once
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "Eigen/src/Core/Matrix.h"

// 辅助函数：将角度无死循环地归一化到 [-pi, pi]
inline double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// ==========================================
// 结合 AKF 思想的鲁棒无迹卡尔曼滤波器 (AKF-CTRV-UKF)
// ==========================================
class KalmanFilterCTRV_UKF {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KalmanFilterCTRV_UKF() {
        n_x_ = 5;  // State: [x, y, v, yaw, omega]
        n_sig_ = 2 * n_x_ + 1;
        lambda_ = 3.0 - n_x_;

        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;  // 初始不确定度较大

        H_.setZero();
        H_(0, 0) = 1.0;  // 仅观测 x
        H_(1, 1) = 1.0;  // 仅观测 y

        // 初始化 UKF 权重
        weights_.resize(n_sig_);
        weights_(0) = lambda_ / (lambda_ + n_x_);
        for (int i = 1; i < n_sig_; i++) {
            weights_(i) = 0.5 / (n_x_ + lambda_);
        }

        // AKF 历史新息协方差矩阵初始化
        V_.setZero();

        update_count_ = 0;
        base_r_noise_ = 1.0;
    }

    void reset() {
        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;
        V_.setZero();
        update_count_ = 0;
    }

    Eigen::Vector2d get_prob() { return Eigen::Vector2d::Zero(); }

    void force_set_state(double x, double y) {
        x_ << x, y, 0.0, 0.0, 0.0;
        P_.setIdentity();
        P_.block<2, 2>(0, 0) *= 0.1;  // 位置较确定
        P_.block<3, 3>(2, 2) *= 5.0;  // 速度、航向、角速度极不确定
        V_.setZero();
        update_count_ = 0;
    }

    // 核心更新函数 (集成了 AKF 与 鲁棒门控)
    void update(double& epsilon, double meas_x, double meas_y, double dt,
                double current_z, double angular_rate,
                double visual_angle_deg) {
        if (dt <= 1e-4) return;
        update_count_++;

        // =====================================
        // 1. 物理先验自适应过程噪声 Q
        // =====================================
        // 思想：速度越快，越不可能出现极端的角加速度（防飘移）
        double current_v = std::abs(x_(2));
        double max_acc = 2.0;
        double max_yaw_acc = 1.0;

        double adaptive_yaw_acc =
            max_yaw_acc * std::clamp(1.0 / (current_v + 0.5), 0.2, 1.0);

        Eigen::Matrix<double, 5, 5> Q;
        Q.setZero();
        double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;

        double var_a = max_acc * max_acc;
        double var_yaw_a = adaptive_yaw_acc * adaptive_yaw_acc;

        Q(0, 0) = dt4 / 4.0 * var_a;
        Q(1, 1) = dt4 / 4.0 * var_a;
        Q(2, 2) = dt2 * var_a;
        Q(3, 3) = dt4 / 4.0 * var_yaw_a;
        Q(4, 4) = dt2 * var_yaw_a;

        // =====================================
        // 2. 生成 Sigma 点 (LLT 保护)
        // =====================================
        Eigen::MatrixXd P_safe = P_ + Eigen::MatrixXd::Identity(5, 5) * 1e-4;
        Eigen::LLT<Eigen::MatrixXd> lltOfP(P_safe);
        if (lltOfP.info() != Eigen::Success) {
            spdlog::warn("[AKF-UKF] LLT failed, resetting covariance!");
            P_.setIdentity();
            P_.block<2, 2>(0, 0) *= 2.0;
            P_.block<3, 3>(2, 2) *= 5.0;
            P_safe = P_;
            lltOfP.compute(P_safe);
        }

        Eigen::MatrixXd L = lltOfP.matrixL();
        Eigen::MatrixXd Xsig = Eigen::MatrixXd::Zero(n_x_, n_sig_);
        Xsig.col(0) = x_;
        for (int i = 0; i < n_x_; i++) {
            Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * L.col(i);
            Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * L.col(i);
        }

        // =====================================
        // 3. 状态预测 (CTRV 模型无缝退化)
        // =====================================
        Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd::Zero(n_x_, n_sig_);
        for (int i = 0; i < n_sig_; i++) {
            double p_x = Xsig(0, i), p_y = Xsig(1, i);
            double v = Xsig(2, i), yaw = Xsig(3, i), omega = Xsig(4, i);
            double px_p, py_p;

            // 当 omega 趋近 0，自然退化为 CV 模型计算，防止除 0
            if (fabs(omega) > 1e-3) {
                px_p = p_x + (v / omega) * (sin(yaw + omega * dt) - sin(yaw));
                py_p = p_y + (v / omega) * (cos(yaw) - cos(yaw + omega * dt));
            } else {
                px_p = p_x + v * dt * cos(yaw);
                py_p = p_y + v * dt * sin(yaw);
            }

            Xsig_pred(0, i) = px_p;
            Xsig_pred(1, i) = py_p;
            Xsig_pred(2, i) = v;
            Xsig_pred(3, i) = yaw + omega * dt;
            Xsig_pred(4, i) = omega;
        }

        // =====================================
        // 4. 重建预测均值与协方差
        // =====================================
        Eigen::Matrix<double, 5, 1> x_pred =
            Eigen::Matrix<double, 5, 1>::Zero();
        double sum_sin = 0.0, sum_cos = 0.0;
        for (int i = 0; i < n_sig_; i++) {
            x_pred(0) += weights_(i) * Xsig_pred(0, i);
            x_pred(1) += weights_(i) * Xsig_pred(1, i);
            x_pred(2) += weights_(i) * Xsig_pred(2, i);
            x_pred(4) += weights_(i) * Xsig_pred(4, i);
            sum_sin += weights_(i) * sin(Xsig_pred(3, i));
            sum_cos += weights_(i) * cos(Xsig_pred(3, i));
        }
        x_pred(3) = atan2(sum_sin, sum_cos);

        // 【低速航向锁定】：速度太低时，强行抹平角速度防止原地陀螺转
        if (std::abs(x_pred(2)) < 0.3) {
            x_pred(4) *= 0.5;
        }

        Eigen::Matrix<double, 5, 5> P_pred =
            Eigen::Matrix<double, 5, 5>::Zero();
        for (int i = 0; i < n_sig_; i++) {
            Eigen::Matrix<double, 5, 1> x_diff = Xsig_pred.col(i) - x_pred;
            x_diff(3) = normalize_angle(x_diff(3));
            P_pred += weights_(i) * x_diff * x_diff.transpose();
        }
        P_pred += Q;

        // =====================================
        // 5. AKF 核心：自适应测量/过程噪声推断
        // =====================================
        // 基础动态 R (基于感知先验：高度、旋转率、视角)
        double height_factor = std::clamp(current_z / 2.0, 0.8, 5.0);
        double rotation_factor = 1.0 + std::abs(angular_rate) * 3.0;
        double angle_factor = 1.0 + std::pow(visual_angle_deg / 15.0, 2.0);
        Eigen::Matrix2d R_base =
            Eigen::Matrix2d::Identity() *
            (base_r_noise_ * height_factor * rotation_factor * angle_factor);

        Eigen::Vector2d z(meas_x, meas_y);
        Eigen::Vector2d z_pred = H_ * x_pred;
        Eigen::Vector2d y = z - z_pred;  // 创新/残差 (Innovation)

        // 平滑新息协方差 V (AKF 关键记忆矩阵，用于捕获连续误差趋势)
        if (update_count_ == 1) {
            V_ = y * y.transpose();
        } else {
            V_ = 0.8 * V_ + 0.2 * (y * y.transpose());
        }

        Eigen::Matrix2d S_prior = H_ * P_pred * H_.transpose() + R_base;

        // 计算马氏距离平方
        double maha_dist = y.transpose() * S_prior.inverse() * y;
        Eigen::Matrix2d R_final = R_base;

        // 【机制A: 观测跳变抑制 (Adaptive R)】
        // 若突发单点巨大偏差(>9.21，代表置信度外极值)，我们认为是噪声干扰，放大
        // R 压低增益
        if (update_count_ > 10 && maha_dist > 9.21) {
            double scale = std::clamp(maha_dist / 4.0, 1.0, 20.0);
            R_final *= scale;
        }
        // 【机制B: 目标真实机动捕获 (Adaptive P/Q)】
        // 若残差合理但累积误差矩阵 V
        // 大于预期协方差，说明模型滞后于真实运动（开始急转弯或加速）
        else if (update_count_ > 10) {
            double trace_V_minus_R = (V_ - R_base).trace();
            double trace_HP = (H_ * P_pred * H_.transpose()).trace();

            // 强跟踪衰减因子 (Fading factor lambda)
            if (trace_V_minus_R > trace_HP) {
                double akf_lambda = trace_V_minus_R / std::max(trace_HP, 1e-6);
                akf_lambda =
                    std::clamp(akf_lambda, 1.0, 4.0);  // 限制膨胀倍数，防止发散

                // 将预测协方差膨胀，增加对新观测的信任，强行“掰弯”轨迹
                P_pred *= akf_lambda;
            }
        }

        // =====================================
        // 6. 计算最终卡尔曼增益并更新
        // =====================================
        Eigen::Matrix2d S_final = H_ * P_pred * H_.transpose() + R_final;
        Eigen::Matrix<double, 5, 2> K =
            P_pred * H_.transpose() * S_final.inverse();

        x_ = x_pred + K * y;
        x_(3) = normalize_angle(x_(3));

        Eigen::Matrix<double, 5, 5> I = Eigen::Matrix<double, 5, 5>::Identity();
        P_ = (I - K * H_) * P_pred;
        P_ = 0.5 * (P_ + P_.transpose());  // 强制保持对称性
    }

    // 接口获取数据
    Eigen::Vector2d get_vel() const {
        if (update_count_ < 5) return Eigen::Vector2d::Zero();
        return Eigen::Vector2d(x_(2) * cos(x_(3)), x_(2) * sin(x_(3)));
    }
    Eigen::Vector2d get_pos() const { return Eigen::Vector2d(x_(0), x_(1)); }
    double get_speed() const { return x_(2); }
    double get_yaw() const { return x_(3); }
    double get_turn_rate() const { return x_(4); }

   private:
    int n_x_, n_sig_;
    double lambda_;
    Eigen::VectorXd weights_;

    Eigen::Matrix<double, 5, 1> x_;
    Eigen::Matrix<double, 5, 5> P_;
    Eigen::Matrix<double, 2, 5> H_;

    // AKF 新息统计
    Eigen::Matrix2d V_;

    int update_count_;
    double base_r_noise_;
};