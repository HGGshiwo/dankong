#pragma once
#include "utils/state_registry.hpp"

class ThreadedTracker;
struct TrackerContext {
    std::shared_ptr<ThreadedTracker> tracker;

   public:
    explicit TrackerContext(StateRegistry& r) {};
};