#include "core/engine.hpp"
#include "dk/adapters/mavsdk.hpp"
#include "robot_context.hpp"

template class dk::MavsdkAdapter<RobotContext, Engine>;
