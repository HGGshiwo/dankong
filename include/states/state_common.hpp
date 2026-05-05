#pragma once
#include "dk/core.hpp"
#include "robot_context.hpp"
#include "robot_event.hpp"
using StatePtr = std::shared_ptr<dk::IState<RobotEvent, RobotContext>>;