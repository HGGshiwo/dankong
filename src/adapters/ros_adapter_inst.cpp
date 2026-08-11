#ifdef USE_ROS
#include "core/engine.hpp"
#include "dk/adapters/ros.hpp"
#include "robot_context.hpp"

template class dk::RosAdapter<RobotContext, Engine>;
#endif
