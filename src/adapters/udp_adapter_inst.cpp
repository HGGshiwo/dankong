#include "core/engine.hpp"
#include "dk/adapters/udp/udp.hpp"
#include "features/dog/command.hpp"
#include "robot_context.hpp"

template class dk::UdpAdapter<RobotContext, Engine, CommandType>;
