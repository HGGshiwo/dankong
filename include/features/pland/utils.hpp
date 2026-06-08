#pragma once
#include "robot_context.hpp"

inline double get_current_z(RobotContext& ctx_) {
    double min_alt = 3.0;
    double max_alt = 10.0;

    double pos_z = ctx_.pos_enu.load().z();
    double rangefinder_z = std::abs(ctx_.rangefinder_alt.load());

    if (pos_z < min_alt) {
        return rangefinder_z;
    } else if (pos_z > max_alt) {
        return pos_z;
    } else {
        double ratio = (pos_z - min_alt) / (max_alt - min_alt);  // 3→0, 10→1
        return (1.0 - ratio) * rangefinder_z + ratio * pos_z;
    }
}
