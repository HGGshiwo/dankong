#pragma once
#include <spdlog/spdlog.h>

#include <cmath>
#include <memory>

#include "kalman_filter_2d.hpp"    // 原有的 CV 模型
#include "kalman_filter_ctrv.hpp"  // 刚才写的 CTRV 模型

class KalmanFilterIMM {
   public:
    KalmanFilterIMM() {
        // 模型 A：CV 模型，极其保守，专门用来过滤抖动
        model_cv_ = std::make_shared<KalmanFilter2D>();
        // 模型 B：CTRV 模型，专门处理转弯和机动
        model_ctrv_ = std::make_shared<KalmanFilterCTRV>();

        prob_cv_ = 0.8;
        prob_ctrv_ = 0.2;

        v_cv_ = Eigen::Vector2d::Zero();
        v_ctrv_ = Eigen::Vector2d::Zero();
    }

    void force_set_state(double x, double y) {
        model_cv_->force_set_state(x, y);
        model_ctrv_->force_set_state(x, y);

        prob_cv_ = 0.5;
        prob_ctrv_ = 0.5;
    }

    // 接口保持不变，完美替换
    Eigen::Vector2d update(double meas_x, double meas_y, double dt,
                           double current_z, double angular_rate,
                           double visual_angle, double max_acc = 1.0) {
        // 1. 并行更新
        double eps_cv = 0;
        double eps_ctrv = 0;
        Eigen::Vector2d v_cv = model_cv_->update(eps_cv, meas_x, meas_y, dt,
                                                 current_z, angular_rate);
        Eigen::Vector2d v_ctrv =
            model_ctrv_->update(eps_ctrv, meas_x, meas_y, dt, current_z,
                                angular_rate, visual_angle, max_acc);

        double L_cv = std::exp(-0.5 * std::min(eps_cv, 20.0));
        double L_ctrv = std::exp(-0.5 * std::min(eps_ctrv, 20.0));

        // 3. 概率切换 (贝叶斯更新)
        double total = (L_cv * prob_cv_) + (L_ctrv * prob_ctrv_);

        if (total < 1e-6) {
            // 此时不要更新概率，保留旧的概率，并输出一个 warning
            spdlog::warn(
                "[IMM] Both models rejected measurement! eps_cv:{:.1f} "
                "eps_ctrv:{:.1f}",
                eps_cv, eps_ctrv);
            return (prob_cv_ * v_cv_) + (prob_ctrv_ * v_ctrv_);
        }

        v_cv_ = v_cv;
        v_ctrv_ = v_ctrv;

        prob_cv_ = (L_cv * prob_cv_) / total;
        prob_ctrv_ = (L_ctrv * prob_ctrv_) / total;

        // 4. 加权输出
        return (prob_cv_ * v_cv) + (prob_ctrv_ * v_ctrv);
    }

    // 同样封装 get_pos() 和 get_vel() ...
    Eigen::Vector2d get_vel() const {
        return (prob_cv_ * model_cv_->get_vel()) +
               (prob_ctrv_ * model_ctrv_->get_vel());
    }

    Eigen::Vector2d get_pos() const {
        return (prob_cv_ * model_cv_->get_pos()) +
               (prob_ctrv_ * model_ctrv_->get_pos());
    }

    void reset() {
        model_cv_->reset();  // 重置时清零
        model_ctrv_->reset();
    }

   private:
    std::shared_ptr<KalmanFilter2D> model_cv_;
    std::shared_ptr<KalmanFilterCTRV> model_ctrv_;
    double prob_cv_, prob_ctrv_;
    Eigen::Vector2d v_cv_, v_ctrv_;
};