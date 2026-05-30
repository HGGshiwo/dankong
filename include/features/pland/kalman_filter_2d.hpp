#pragma once
#include <Eigen/Dense>

class KalmanFilter2D {
   public:
    KalmanFilter2D() {
        // 状态向量 x: [pos_x, pos_y, vel_x, vel_y]^T
        x_.setZero();

        // 协方差矩阵 P
        P_.setIdentity();

        // 状态转移矩阵 F (会随 dt 更新)
        F_.setIdentity();

        // 观测矩阵 H: 我们只测量位置 [pos_x, pos_y]
        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;

        // 过程噪声 Q (调参重点：越大说明目标越容易变速)
        Q_.setIdentity();
        Q_ *= 0.5;

        // 测量噪声 R (调参重点：越大说明视觉抖动越厉害)
        R_.setIdentity();
        R_ *= 1.0;
    }

    void reset() {
        x_.setZero();
        P_.setIdentity();
    }

    // 输入观测到的 x, y 位置和时间步长 dt，返回滤波后的速度估计
    Eigen::Vector2d update(double meas_x, double meas_y, double dt) {
        if (dt <= 0) return Eigen::Vector2d(x_(2), x_(3));

        // --- 1. 预测阶段 (Predict) ---
        F_(0, 2) = dt;
        F_(1, 3) = dt;

        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // --- 2. 更新阶段 (Update) ---
        Eigen::Vector2d z(meas_x, meas_y);  // 测量值
        Eigen::Vector2d y = z - H_ * x_;    // 测量残差 (Innovation)
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;  // 残差协方差
        Eigen::Matrix<double, 4, 2> K =
            P_ * H_.transpose() * S.inverse();  // 卡尔曼增益

        x_ = x_ + K * y;
        Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
        P_ = (I - K * H_) * P_;

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
    Eigen::Matrix4d Q_;              // 过程噪声协方差
    Eigen::Matrix2d R_;              // 观测噪声协方差
};