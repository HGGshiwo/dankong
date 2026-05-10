#pragma once
#include <Eigen/Dense>

#include "dk/future.hpp"
#include "mavlink/imavlink.hpp"

template <typename MavlinkType, typename Context>
class IRobot : public MavlinkType {
    static_assert(std::is_base_of<IMavlink, MavlinkType>::value,
                  "Template parameter MavlinkType must inherit from IMavlink!");

   public:
    virtual bool check_hover(bool arm, double rel_alt) = 0;

    // 计算目标距离（车和机有高度上区别）
    virtual double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) = 0;
    virtual dk::Future<bool> land(Context& ctx) = 0;

    virtual bool is_prearm_enable() = 0;
    virtual bool is_alt_enable() = 0;
};