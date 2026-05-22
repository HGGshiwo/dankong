#pragma once
#include "dk/report.hpp"

class ThreadedTracker;
struct TrackerContext {
    dk::StateRegistry& reg;

    std::shared_ptr<ThreadedTracker> tracker;

   public:
    explicit TrackerContext(dk::StateRegistry& r) : reg(r) {};
};