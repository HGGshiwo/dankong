#include <Eigen/Dense>
#include <iostream>
#include <numeric>
#include <optional>
#include <vector>

class CameraOffsetEstimator {
   public:
    struct EvaluationResult {
        double reverse_pos_error_norm;  // 反推无人机位置的绝对误差距离 (米)
        Eigen::Vector3d
            reverse_pos_error;  // 反推无人机位置在 ENU 下的各轴误差 (米)
        double offset_std_dev;  // 当前安装估计值的收敛标准差 (米)
    };

    CameraOffsetEstimator(
        const Eigen::Vector3d& tag_enu_pos = Eigen::Vector3d::Zero(),
        size_t window_size = 30)
        : tag_pos_w_(tag_enu_pos), max_window_size_(window_size) {}

    /**
     * @brief 1. 更新无人机机体自身在ENU坐标系下的定位及姿态
     * @param drone_pos_enu 无人机当前ENU坐标 [X, Y, Z]
     * @param R_wb         从机体系(FLU)到世界系(ENU)的旋转矩阵
     */
    void update_drone_state(const Eigen::Vector3d& drone_pos_enu,
                            const Eigen::Matrix3d& R_wb) {
        last_drone_pos_w_ = drone_pos_enu;
        last_R_wb_ = R_wb;
        has_drone_state_ = true;
    }

    /**
     * @brief 2. 更新摄像头PnP解出的二维码位置，并推算平移偏移 t_bc
     * @param t_tag_cam    PnP求得的二维码在相机系下平移向量
     * @param R_cb         从相机坐标系到机体(FLU)坐标系的旋转矩阵
     * @return             当前单帧解算出的安装偏移 (FLU 坐标系)
     */
    std::optional<Eigen::Vector3d> update_detection(
        const Eigen::Vector3d& t_tag_cam, const Eigen::Matrix3d& R_cb) {
        if (!has_drone_state_) {
            std::cerr << "[Warn] 尚未更新无人机状态，无法估算偏移!"
                      << std::endl;
            return std::nullopt;
        }

        // 依据公式: t_bc = (R_wb)^T * (P_tag_w - P_drone_w) - R_cb * t_tag_cam
        Eigen::Vector3d t_bc_single =
            last_R_wb_.transpose() * (tag_pos_w_ - last_drone_pos_w_) -
            R_cb * t_tag_cam;

        // 加入滑动窗口供滤波使用
        offset_history_.push_back(t_bc_single);
        if (offset_history_.size() > max_window_size_) {
            offset_history_.erase(offset_history_.begin());
        }

        return t_bc_single;
    }

    /**
     * @brief 3.
     * 获取平滑估计结果（求当前窗口内的均值偏移，有效消除光流或GPS高频抖动）
     * @return 估算的相机相对于飞机体坐标系(FLU)安装偏置：[X(前), Y(左), Z(上)]
     */
    std::optional<Eigen::Vector3d> get_estimated_offset() const {
        if (offset_history_.empty()) {
            return std::nullopt;
        }

        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const auto& offset : offset_history_) {
            sum += offset;
        }
        return sum / static_cast<double>(offset_history_.size());
    }

    // 清空历史数据（手持挪动到新的静态场景开始新测试时调用）
    void reset(double x, double y, double z) {
        offset_history_.clear();
        has_drone_state_ = false;
        tag_pos_w_ = Eigen::Vector3d{x, y, z};
    }

    int sample_count() { return offset_history_.size(); }

    /**
     * @brief 核心评估方法：计算当前反推误差与收敛标准差
     */
    std::optional<EvaluationResult> evaluate_error(
        const Eigen::Vector3d& t_tag_cam, const Eigen::Matrix3d& R_cb) const {
        if (!has_drone_state_ || offset_history_.empty()) {
            return std::nullopt;
        }

        EvaluationResult res;

        // 1. 获取当前平滑估算出的安装偏移
        Eigen::Vector3d t_bc_est = get_estimated_offset().value();

        // 2. 反向推算无人机在 ENU 坐标系下的位置
        // 公式: P_drone_calc = P_tag - R_wb * (t_bc + R_cb * t_tag_cam)
        Eigen::Vector3d P_drone_calc_w =
            tag_pos_w_ - last_R_wb_ * (t_bc_est + R_cb * t_tag_cam);

        // 3. 计算反推位置与飞控实际位置的误差
        res.reverse_pos_error = last_drone_pos_w_ - P_drone_calc_w;
        res.reverse_pos_error_norm = res.reverse_pos_error.norm();

        // 4. 计算收敛标准差（判断数据是否还在剧烈波动）
        double variance = 0.0;
        for (const auto& offset : offset_history_) {
            // 计算历史中每个样本点到平均值的距离平方
            variance += (offset - t_bc_est).squaredNorm();
        }
        variance /= offset_history_.size();
        res.offset_std_dev = std::sqrt(variance);

        return res;
    }

   private:
    Eigen::Vector3d tag_pos_w_;  // 二维码在ENU下的位置 (通常设置为 0,0,0)
    Eigen::Vector3d last_drone_pos_w_;  // 最新一次无机ENU坐标
    Eigen::Matrix3d last_R_wb_;  // 最新一次机体系到ENU的旋转矩阵
    bool has_drone_state_ = false;

    size_t max_window_size_;
    std::vector<Eigen::Vector3d> offset_history_;
};