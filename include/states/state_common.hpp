#pragma once

#include "dk/engine.hpp"
#include "dk/state.hpp"
#include "features/algo/context.hpp"
#include "features/algo/events.hpp"
#include "features/control/context.hpp"
#include "features/control/events.hpp"
#include "robot_context.hpp"
#include "utils/logger/spd_logger.hpp"

using StateAction = dk::StateAction<RobotContext>;