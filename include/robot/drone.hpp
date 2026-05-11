#pragma once
#include "./robot_base.hpp"
#include "dk/future.hpp"
#include "mavlink/mavros.hpp"
#include "robot_context.hpp"
#include "states/state_utils.hpp"
#include "utils.hpp"

template <typename MavlinkType>
class Drone : public IRobot<MavlinkType, RobotContext> {
    bool check_hover(bool arm, double rel_alt) override;
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) override;
    bool is_prearm_enable() override;
    bool is_alt_enable() override;

    dk::Future<bool> land(RobotContext& ctx) override;
};

inline double HOVER_THRESH = 1.0;

template <typename MavlinkType>
double Drone<MavlinkType>::get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) {
    return (pos - goal).norm();
}

template <typename MavlinkType>
bool Drone<MavlinkType>::is_prearm_enable() {
    return true;
}

template <typename MavlinkType>
bool Drone<MavlinkType>::is_alt_enable() {
    return true;
}

template <typename MavlinkType>
bool Drone<MavlinkType>::check_hover(bool arm, double rel_alt) {
    return arm && rel_alt > HOVER_THRESH;
}

template <typename MavlinkType>
dk::Future<bool> Drone<MavlinkType>::land(RobotContext& ctx) {
    using Promise = dk::Promise<bool>;
    FixedString64 mode("LAND");
    if (ctx.mode.get() == mode) return Promise::resolve(ctx.engine, true);
    ctx.robot->set_mode(mode);

    return ctx.engine->wait_for(1000, [mode](const FlightModeEvent& e) -> bool {
        if (mode != e.cur) return false;
        return true;
    });
}