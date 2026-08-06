#pragma once

#include "dk/engine.hpp"
#include "dk/state.hpp"
#include "features/algo/context.hpp"
#include "features/algo/events.hpp"
#include "features/control/context.hpp"
#include "features/control/events.hpp"
#include "robot_context.hpp"

using StateAction = dk::StateAction<RobotContext>;

StateFlags get_state_flags(RobotContext& ctx);