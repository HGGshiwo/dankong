#pragma once
#include <Eigen/Dense>

#include "mavlink/imavlink.hpp"

enum CmdFrame { BODY, ENU };

template <typename MavlinkType>
class IRobot : public MavlinkType {
    static_assert(std::is_base_of<IMavlink, MavlinkType>::value,
                  "Template parameter MavlinkType must inherit from IMavlink!");

   public:
    virtual bool check_hover(bool arm, double rel_alt) = 0;

    // 计算目标距离（车和机有高度上区别）
    virtual double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) = 0;
    virtual bool is_prearm_enable() = 0;
    virtual bool is_alt_enable() = 0;
    virtual bool send_cmd(std::optional<Eigen::Vector3d> pos, std::optional<Eigen::Vector3d> vel,
                          std::optional<Eigen::Vector3d> acc, std::optional<double> yaw, std::optional<double> yaw_rate,
                          CmdFrame frame) = 0;
};