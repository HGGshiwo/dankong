#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

class KalmanFilter2D {
   public:
    // ==========================================
    // 开放调参区 (根据你的物理场景微调)
    // ==========================================
    // 目标的理论最大物理加速度 (m/s^2)。对于静止或平缓移动平台，0.5 足够了。
    double sigma_a_ = 0.5;

    // 基础观测噪声方差。代表理想状态下（低空、平稳）的射线法像素投影误差。
    double base_r_noise_ = 1.0;

    // 马氏距离阈值 (卡方分布，自由度2，5.0大约对应90%置信区间)。
    // 超过此值，滤波器判定为“目标主动机动”，而非“测量抖动”。
    double adaptive_threshold_ = 5.0;

    // Q 矩阵最大放大倍数，防止极端飞点把滤波器炸散。
    double max_q_scale_ = 10.0;

    KalmanFilter2D() {
        // 观测矩阵 H: 我们只测量位置 [pos_x, pos_y]
        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;

        reset();
    }

    void reset() {
        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;  // 初始协方差大一点，让它在刚发现目标时能光速收敛
    }

    void force_set_state(double x, double y) {
        // 将状态向量直接重置
        x_ << x, y, 0.0, 0.0;

        // 【重要】重置协方差矩阵 P
        // 因为状态已经被强制设定为极高置信度了，
        // 我们需要把协方差调小，告诉滤波器：“我现在对这个新位置非常确信”
        P_.setIdentity();
        P_ *= 0.01;  // 赋予一个很小的初始不确定度
    }

    // [重磅修改] 引入当前相对高度 current_z 和无人机自身角速度 angular_rate
    Eigen::Vector2d update(double& epsilon, double meas_x, double meas_y,
                           double dt, double current_z,
                           double angular_rate = 0.0) {
        // 防止除零或 dt 异常
        if (dt <= 1e-4) return Eigen::Vector2d(x_(2), x_(3));

        // --- 1. 动态生成严格的运动学过程噪声 Q ---
        // 利用离散化运动学方程推导的 Q
        // 矩阵，根除盲目给速度赋方差导致的“幽灵速度”
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;

        Eigen::Matrix4d Q;
        Q.setZero();
        Q(0, 0) = dt4 / 4.0;
        Q(0, 2) = dt3 / 2.0;
        Q(1, 1) = dt4 / 4.0;
        Q(1, 3) = dt3 / 2.0;
        Q(2, 0) = dt3 / 2.0;
        Q(2, 2) = dt2;
        Q(3, 1) = dt3 / 2.0;
        Q(3, 3) = dt2;
        Q *= (sigma_a_ * sigma_a_);

        // --- 2. 动态生成观测噪声 R (物理感知) ---
        // 高度越高、无人机自身旋转越快，射线法算出的位置越不可信
        double height_factor = std::max(1.0, current_z / 2.0);  // 2米以内不惩罚
        double rotation_factor =
            1.0 + angular_rate * 5.0;  // 旋转越快，倍率越高
        double dynamic_r =
            base_r_noise_ * height_factor * height_factor * rotation_factor;

        Eigen::Matrix2d R;
        R.setIdentity();
        R *= dynamic_r;

        // --- 3. 基础预测阶段 (Predict) ---
        F_.setIdentity();
        F_(0, 2) = dt;
        F_(1, 3) = dt;

        Eigen::Vector4d x_pred = F_ * x_;
        Eigen::Matrix4d P_pred = F_ * P_ * F_.transpose() + Q;

        // --- 4. 计算残差与马氏距离 ---
        Eigen::Vector2d z(meas_x, meas_y);
        Eigen::Vector2d y = z - H_ * x_pred;                   // 残差
        Eigen::Matrix2d S = H_ * P_pred * H_.transpose() + R;  // 残差协方差

        // 计算马氏距离的平方 (Mahalanobis Distance)
        epsilon = y.transpose() * S.inverse() * y;

        // --- 5. 自适应过程噪声 (AKF 核心：对抗高动态机动) ---
        if (epsilon > adaptive_threshold_) {
            // 马氏距离超标，说明目标发生机动，按残差偏离比例放大 Q
            double scale_factor =
                std::min(max_q_scale_, epsilon / adaptive_threshold_);
            Eigen::Matrix4d adaptive_Q = Q * scale_factor;

            // 使用放大后的 Q 重新计算 P_pred 和 S
            P_pred = F_ * P_ * F_.transpose() + adaptive_Q;
            S = H_ * P_pred * H_.transpose() + R;
        }

        // --- 6. 更新阶段 (Update) ---
        Eigen::Matrix<double, 4, 2> K = P_pred * H_.transpose() * S.inverse();

        x_ = x_pred + K * y;
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        P_ = (I - K * H_) * P_pred;

        // 返回滤波后的速度 (vel_x, vel_y)
        return Eigen::Vector2d(x_(2), x_(3));
    }

    Eigen::Vector2d get_pos() const { return x_.head<2>(); }
    Eigen::Vector2d get_vel() const { return x_.tail<2>(); }

   private:
    Eigen::Vector4d x_;              // 状态量 [x, y, vx, vy]
    Eigen::Matrix4d P_;              // 状态协方差
    Eigen::Matrix4d F_;              // 状态转移矩阵
    Eigen::Matrix<double, 2, 4> H_;  // 观测矩阵
};