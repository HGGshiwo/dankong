#pragma once
#include "../robot_context.hpp"
#include "./utils.hpp"
#include "dk/logger.hpp"

namespace state_utils {
dk::Future<bool> takeoff_vehicle(RobotContext& ctx, double alt);
std::shared_ptr<dk::IState<RobotEvent, RobotContext>> check_state(const RobotContext& ctx);

bool check_alt(RobotContext& ctx, double target);

dk::Future<bool> arm_vehicle(RobotContext& ctx);
Eigen::Vector3d gps_to_enu_body(double lat, double lon, double alt);
Eigen::Vector3d gps_to_enu(RobotContext& ctx, double lat, double lon, double alt);
dk::Future<bool> prearm_check(RobotContext& ctx);

}  // namespace state_utils