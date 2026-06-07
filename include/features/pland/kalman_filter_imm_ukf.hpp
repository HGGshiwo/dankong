#pragma once
#include <spdlog/spdlog.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

// 辅助函数：利用 atan2 和 sin/cos 快速、无死循环地将角度归一化到 [-pi, pi]
inline double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

// ==========================================
// 核心：纯净版无迹卡尔曼滤波器 (UKF) - 移除 AKF
// ==========================================
class UkfModel {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum ModelType { CV_MODEL, CTRV_MODEL };

    ModelType type_;
    double base_r_noise_ = 1.0;
    int update_count_ = 0;

    UkfModel(ModelType type) : type_(type) {
        n_x_ = 5;
        n_sig_ = 2 * n_x_ + 1;
        lambda_ = 3.0 - n_x_;

        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;

        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;

        // 初始化 UKF 权重
        weights_.resize(n_sig_);
        weights_(0) = lambda_ / (lambda_ + n_x_);
        for (int i = 1; i < n_sig_; i++) {
            weights_(i) = 0.5 / (n_x_ + lambda_);
        }
    }

    void reset() {
        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;
        update_count_ = 0;
    }

    void set_state(const Eigen::Matrix<double, 5, 1>& x,
                   const Eigen::Matrix<double, 5, 5>& P) {
        x_ = x;
        P_ = P;
        P_ = 0.5 * (P_ + P_.transpose());  // 强制对称
    }

    Eigen::Matrix<double, 5, 1> get_state() const { return x_; }
    Eigen::Matrix<double, 5, 5> get_covariance() const { return P_; }

    void force_set_state(double x, double y) {
        x_ << x, y, 0.0, 0.0, 0.0;
        P_.setIdentity();
        P_ *= 0.01;
    }

    // UKF 核心 Update，返回该模型的 对数似然 (Log-Likelihood)
    double update(double& epsilon, double meas_x, double meas_y, double dt,
                  double current_z, double angular_rate,
                  double visual_angle_deg, double max_acc = 1.0,
                  double max_yaw_acc = 0.5) {
        update_count_++;
        if (dt <= 1e-4) {
            epsilon = 0.0;
            return 0.0;  // dt过小，不对似然产生贡献
        }

        // --- 1. 生成过程噪声 Q (保持各自模型特性) ---
        Eigen::Matrix<double, 5, 5> Q;
        Q.setZero();

        double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;
        double var_p = max_acc * max_acc;
        double var_v = (max_acc / 2.0) * (max_acc / 2.0);

        Q(0, 0) = dt4 / 4.0 * var_p;
        Q(1, 1) = dt4 / 4.0 * var_p;
        Q(2, 2) = dt2 * var_v;

        if (type_ == CTRV_MODEL) {
            double var_yaw_a = max_yaw_acc * max_yaw_acc;
            Q(3, 3) = dt4 / 4.0 * var_yaw_a;
            Q(4, 4) = dt2 * var_yaw_a;
        }

        // --- 2. 生成 Sigma 点 ---
        Eigen::LLT<Eigen::MatrixXd> lltOfP(
            P_ + Eigen::MatrixXd::Identity(5, 5) * 1e-4);
        if (lltOfP.info() != Eigen::Success) {
            spdlog::error("[UKF] LLT decomposition failed! Resetting model.");
            reset();
            epsilon = 100.0;
            return -1e6;  // 返回极小的对数似然
        }

        Eigen::Matrix<double, 5, 5> L = lltOfP.matrixL();
        Eigen::MatrixXd Xsig = Eigen::MatrixXd::Zero(n_x_, n_sig_);
        Xsig.col(0) = x_;
        for (int i = 0; i < n_x_; i++) {
            Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * L.col(i);
            Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * L.col(i);
        }

        // --- 3. Sigma 点预测 (Process Model) ---
        Eigen::MatrixXd Xsig_pred = Eigen::MatrixXd::Zero(n_x_, n_sig_);
        for (int i = 0; i < n_sig_; i++) {
            double p_x = Xsig(0, i), p_y = Xsig(1, i);
            double v = Xsig(2, i), yaw = Xsig(3, i), omega = Xsig(4, i);
            double px_p, py_p, yaw_p, omega_p;

            if (type_ == CV_MODEL) {
                px_p = p_x + v * cos(yaw) * dt;
                py_p = p_y + v * sin(yaw) * dt;
                yaw_p = yaw;
                omega_p = 0.0;
            } else {
                if (fabs(omega) > 1e-4) {
                    px_p =
                        p_x + (v / omega) * (sin(yaw + omega * dt) - sin(yaw));
                    py_p =
                        p_y + (v / omega) * (cos(yaw) - cos(yaw + omega * dt));
                } else {
                    px_p = p_x + v * dt * cos(yaw);
                    py_p = p_y + v * dt * sin(yaw);
                }
                yaw_p = yaw + omega * dt;
                omega_p = omega;
            }
            Xsig_pred(0, i) = px_p;
            Xsig_pred(1, i) = py_p;
            Xsig_pred(2, i) = v;
            Xsig_pred(3, i) = yaw_p;
            Xsig_pred(4, i) = omega_p;
        }

        // --- 4. 重建预测均值与协方差 ---
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

        Eigen::Matrix<double, 5, 5> P_pred =
            Eigen::Matrix<double, 5, 5>::Zero();
        for (int i = 0; i < n_sig_; i++) {
            Eigen::Matrix<double, 5, 1> x_diff = Xsig_pred.col(i) - x_pred;
            x_diff(3) = normalize_angle(x_diff(3));
            P_pred += weights_(i) * x_diff * x_diff.transpose();
        }
        P_pred += Q;

        // --- 5. 动态观测噪声 R ---
        double height_factor = std::clamp(current_z / 2.0, 0.8, 5.0);
        double rotation_factor = 1.0 + angular_rate * 5.0;
        double angle_factor = 1.0 + std::pow(visual_angle_deg / 10.0, 2.0);
        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() *
                            (base_r_noise_ * height_factor * height_factor *
                             rotation_factor * angle_factor);

        // --- 6. 测量更新 (纯净版，无AKF缩放) ---
        Eigen::Vector2d z(meas_x, meas_y);
        Eigen::Vector2d z_pred = H_ * x_pred;
        Eigen::Vector2d y = z - z_pred;
        Eigen::Matrix2d S = H_ * P_pred * H_.transpose() + R;

        // 计算更新前的新息（马氏距离平方）
        epsilon = y.transpose() * S.inverse() * y;

        // 卡尔曼增益与状态更新
        Eigen::Matrix<double, 5, 2> K = P_pred * H_.transpose() * S.inverse();
        x_ = x_pred + K * y;
        x_(3) = normalize_angle(x_(3));

        Eigen::Matrix<double, 5, 5> I = Eigen::Matrix<double, 5, 5>::Identity();
        P_ = (I - K * H_) * P_pred;
        P_ = 0.5 * (P_ + P_.transpose());

        // --- 7. 计算对数似然 (Log-Likelihood) ---
        // Log(L) = -0.5 * (epsilon + log(det(S)) + 2*log(2*pi))
        double S_det = std::max(S.determinant(), 1e-12);  // 防止log(0)
        double log_likelihood =
            -0.5 * (epsilon + std::log(S_det) + 2.0 * std::log(2.0 * M_PI));

        return log_likelihood;
    }

    Eigen::Vector2d get_vel() const {
        if (update_count_ < 15) return Eigen::Vector2d::Zero();
        return Eigen::Vector2d(x_(2) * cos(x_(3)), x_(2) * sin(x_(3)));
    }
    Eigen::Vector2d get_pos() const { return Eigen::Vector2d(x_(0), x_(1)); }

   private:
    int n_x_, n_sig_;
    double lambda_;
    Eigen::VectorXd weights_;
    Eigen::Matrix<double, 5, 1> x_;
    Eigen::Matrix<double, 5, 5> P_;
    Eigen::Matrix<double, 2, 5> H_;
};

// ==========================================
// 交互式多模型管理器 (IMM-UKF) - 使用 Softmax
// ==========================================
class KalmanFilterIMM {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KalmanFilterIMM() {
        model_cv_ = std::make_shared<UkfModel>(UkfModel::CV_MODEL);
        model_ctrv_ = std::make_shared<UkfModel>(UkfModel::CTRV_MODEL);

        prob_cv_ = 0.8;
        prob_ctrv_ = 0.2;

        p_trans_ << 0.95, 0.05, 0.20, 0.80;

        // 【新增】历史似然平滑变量初始化
        smoothed_log_L_cv_ = 0.0;
        smoothed_log_L_ctrv_ = 0.0;
        is_first_frame_ = true;
    }

    void force_set_state(double x, double y) {
        model_cv_->force_set_state(x, y);
        model_ctrv_->force_set_state(x, y);
        prob_cv_ = 0.8;
        prob_ctrv_ = 0.2;
        // 【新增】重置平滑变量
        is_first_frame_ = true;
    }

    Eigen::Vector2d update(double meas_x, double meas_y, double dt,
                           double current_z, double angular_rate,
                           double visual_angle, double max_acc = 1.0,
                           double max_yaw_acc = 0.5) {
        // =====================================
        // 1. IMM 强混合 (Interaction / Mixing)
        // =====================================
        Eigen::Matrix<double, 5, 1> x_cv = model_cv_->get_state();
        Eigen::Matrix<double, 5, 5> P_cv = model_cv_->get_covariance();
        Eigen::Matrix<double, 5, 1> x_ctrv = model_ctrv_->get_state();
        Eigen::Matrix<double, 5, 5> P_ctrv = model_ctrv_->get_covariance();
        // 【修复BUG】：正确计算先验概率归一化常数 (注意交叉项索引)
        double c_cv = p_trans_(0, 0) * prob_cv_ + p_trans_(1, 0) * prob_ctrv_;
        double c_ctrv = p_trans_(0, 1) * prob_cv_ + p_trans_(1, 1) * prob_ctrv_;
        // 【修复BUG】：正确计算混合概率 mu_i|j (注意交叉项索引)
        double w_cv_to_cv = (p_trans_(0, 0) * prob_cv_) / c_cv;
        double w_ctrv_to_cv = (p_trans_(1, 0) * prob_ctrv_) / c_cv;
        double w_cv_to_ctrv = (p_trans_(0, 1) * prob_cv_) / c_ctrv;
        double w_ctrv_to_ctrv = (p_trans_(1, 1) * prob_ctrv_) / c_ctrv;
        // 混合生成新的初始状态 x
        Eigen::Matrix<double, 5, 1> mixed_x_cv =
            w_cv_to_cv * x_cv + w_ctrv_to_cv * x_ctrv;
        mixed_x_cv(3) =
            atan2(w_cv_to_cv * sin(x_cv(3)) + w_ctrv_to_cv * sin(x_ctrv(3)),
                  w_cv_to_cv * cos(x_cv(3)) + w_ctrv_to_cv * cos(x_ctrv(3)));
        mixed_x_cv(4) = 0.0;
        Eigen::Matrix<double, 5, 1> mixed_x_ctrv =
            w_cv_to_ctrv * x_cv + w_ctrv_to_ctrv * x_ctrv;
        mixed_x_ctrv(3) = atan2(
            w_cv_to_ctrv * sin(x_cv(3)) + w_ctrv_to_ctrv * sin(x_ctrv(3)),
            w_cv_to_ctrv * cos(x_cv(3)) + w_ctrv_to_ctrv * cos(x_ctrv(3)));
        mixed_x_ctrv(4) = w_ctrv_to_ctrv * x_ctrv(4);

        // 混合生成新的初始协方差 P
        auto calc_diff = [](const Eigen::Matrix<double, 5, 1>& x,
                            const Eigen::Matrix<double, 5, 1>& mixed) {
            Eigen::Matrix<double, 5, 1> diff = x - mixed;
            diff(3) = normalize_angle(diff(3));
            return diff;
        };

        Eigen::Matrix<double, 5, 1> diff1 = calc_diff(x_cv, mixed_x_cv);
        Eigen::Matrix<double, 5, 1> diff2 = calc_diff(x_ctrv, mixed_x_cv);
        Eigen::Matrix<double, 5, 5> mixed_P_cv =
            w_cv_to_cv * (P_cv + diff1 * diff1.transpose()) +
            w_ctrv_to_cv * (P_ctrv + diff2 * diff2.transpose());
        mixed_P_cv.row(4).setZero();
        mixed_P_cv.col(4).setZero();
        mixed_P_cv(4, 4) = 1e-6;

        Eigen::Matrix<double, 5, 1> diff3 = calc_diff(x_cv, mixed_x_ctrv);
        Eigen::Matrix<double, 5, 1> diff4 = calc_diff(x_ctrv, mixed_x_ctrv);
        Eigen::Matrix<double, 5, 5> mixed_P_ctrv =
            w_cv_to_ctrv * (P_cv + diff3 * diff3.transpose()) +
            w_ctrv_to_ctrv * (P_ctrv + diff4 * diff4.transpose());

        model_cv_->set_state(mixed_x_cv, mixed_P_cv);
        model_ctrv_->set_state(mixed_x_ctrv, mixed_P_ctrv);

        // =====================================
        // 2. 独立 UKF 滤波获取当前帧的 对数似然
        // =====================================
        double eps_cv = 0, eps_ctrv = 0;
        double log_L_cv =
            model_cv_->update(eps_cv, meas_x, meas_y, dt, current_z,
                              angular_rate, visual_angle, max_acc, 0.0);
        double log_L_ctrv = model_ctrv_->update(
            eps_ctrv, meas_x, meas_y, dt, current_z, angular_rate, visual_angle,
            max_acc, max_yaw_acc);

        // =====================================
        // 【核心改进】：3. 引入历史观测记忆 (Likelihood Momentum)
        // =====================================
        // history_alpha 控制记忆的长度。
        double history_alpha = 0.85;

        if (is_first_frame_) {
            smoothed_log_L_cv_ = log_L_cv;
            smoothed_log_L_ctrv_ = log_L_ctrv;
            is_first_frame_ = false;
        } else {
            // 利用 EMA (指数移动平均) 累积历史对数似然
            smoothed_log_L_cv_ = history_alpha * smoothed_log_L_cv_ + log_L_cv;
            smoothed_log_L_ctrv_ =
                history_alpha * smoothed_log_L_ctrv_ + log_L_ctrv;
        }
        double offset = std::max(smoothed_log_L_cv_, smoothed_log_L_ctrv_);
        smoothed_log_L_cv_ -= offset;
        smoothed_log_L_ctrv_ -= offset;

        // =====================================
        // 【终极鲁棒解法】：3. 状态显著性检验 (Wald Statistic)
        // =====================================
        // 提取 CTRV 模型的角速度及其方差
        double omega = model_ctrv_->get_state()(4);
        double var_omega = model_ctrv_->get_covariance()(4, 4);

        // 计算转弯显著性 (Chi-square 分布，1个自由度)
        double turn_significance = (omega * omega) / (var_omega + 1e-8);

        // 我们将显著性转化为一个对数空间的“奖励分”
        double turn_bonus = 0.0;
        if (turn_significance > 1.0) {
            // 加上一个平滑截断，防止奖励无限大
            turn_bonus = std::min(turn_significance - 1.0, 10.0);
        }

        // =====================================
        // 4. 模型概率更新 (Log-Sum-Exp Softmax)
        // =====================================
        double log_c_cv = std::log(std::max(c_cv, 1e-12));
        double log_c_ctrv = std::log(std::max(c_ctrv, 1e-12));

        // 【核心修改】：把转弯的“自信奖励”加到 CTRV 的 Logit 上！
        double logit_cv = smoothed_log_L_cv_ + log_c_cv;
        double logit_ctrv = smoothed_log_L_ctrv_ + log_c_ctrv + turn_bonus;

        double max_logit = std::max(logit_cv, logit_ctrv);

        double exp_cv = std::exp(logit_cv - max_logit);
        double exp_ctrv = std::exp(logit_ctrv - max_logit);

        double sum_exp = exp_cv + exp_ctrv;

        prob_cv_ = exp_cv / sum_exp;
        prob_ctrv_ = exp_ctrv / sum_exp;

        // 防止出现极限情况引发NaN
        if (std::isnan(prob_cv_) || std::isnan(prob_ctrv_)) {
            spdlog::warn(
                "[IMM-UKF] Softmax yielded NaN. Resetting probabilities.");
            prob_cv_ = 0.8;
            prob_ctrv_ = 0.2;
        }

        return get_vel();
    }

    Eigen::Vector2d get_vel() const {
        return (prob_cv_ * model_cv_->get_vel()) +
               (prob_ctrv_ * model_ctrv_->get_vel());
    }

    Eigen::Vector2d get_pos() const {
        return (prob_cv_ * model_cv_->get_pos()) +
               (prob_ctrv_ * model_ctrv_->get_pos());
    }

    Eigen::Vector2d get_prob() { return {prob_cv_, prob_ctrv_}; }

    void reset() {
        model_cv_->reset();
        model_ctrv_->reset();
        prob_cv_ = 0.8;
        prob_ctrv_ = 0.2;
        // 【新增】重置平滑变量
        is_first_frame_ = true;
    }

   private:
    std::shared_ptr<UkfModel> model_cv_;
    std::shared_ptr<UkfModel> model_ctrv_;
    Eigen::Matrix2d p_trans_;
    double prob_cv_, prob_ctrv_;

    // 【新增变量】
    double smoothed_log_L_cv_;
    double smoothed_log_L_ctrv_;
    bool is_first_frame_;
};