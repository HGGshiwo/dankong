#pragma once
#include "dk/report.hpp"
#include "utils/fixed_string64.hpp"

struct AlgoContext {
    dk::StateRegistry& reg;
    dk::TrackedVar<bool> planner_enable{reg, "planner", 2.0, false};
    dk::TrackedVar<bool> pland_enable{reg, "pland", 2.0, false};
    dk::TrackedVar<FixedString64> version{reg, "version", 0.5,
                                          FixedString64("未知")};
    dk::TrackedVar<FixedString64> detect_type{reg, "detect_type", 1.0,
                                              "Disabled"};
    dk::TrackedVar<bool> recording{reg, "recording", 1.0, false};

   public:
    explicit AlgoContext(dk::StateRegistry& r) : reg(r) {};
};