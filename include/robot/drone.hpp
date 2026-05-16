#pragma once
#include <Eigen/Dense>
#include <memory>

#include "../robot_context.hpp"
#include "./robot_base.hpp"
#include "dk/future.hpp"
#include "mavlink/mavros.hpp"
#include "robot_context.hpp"
#include "states/state_utils.hpp"
#include "utils.hpp"
inline double THROTTLE_THRESH = 0.1;

class Drone : public IRobot {
   public:
    Drone(std::shared_ptr<IMavlink> mavlink) : IRobot(mavlink) {};

    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) {
        return (pos - goal).norm();
    }

    bool is_prearm_enable() { return true; }

    bool is_alt_enable() { return true; }

    bool check_hover(IContext& ctx) {
        return ctx.arm.get() && ctx.throttle.load() > THROTTLE_THRESH;
    }

    bool land(IContext& ctx) {
        FixedString64 mode("LAND");
        return mavlink_->set_mode(mode);
    }

    bool loiter() { return mavlink_->set_mode("LOITER"); }

    bool takeoff(double alt) { return mavlink_->takeoff(alt); }
};
