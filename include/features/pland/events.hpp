#pragma once

#include "core/event_result.hpp"
#include "dk/engine.hpp"

// @JSON_ENABLE
struct StartPlandDetectEvent : dk::AsyncEvent<EventResult> {};