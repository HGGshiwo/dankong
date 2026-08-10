#pragma once
#include <memory>

#include "core/base_tracker.hpp"
#include "utils/state_registry.hpp"

struct TrackerContext {
    std::shared_ptr<ITracker> tracker;

   public:
    explicit TrackerContext(StateRegistry& r) {};
};