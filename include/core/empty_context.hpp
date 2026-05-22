#pragma once
#include "dk/report.hpp"
// 一个什么都不做的假 Context

struct EmptyContext {
    explicit EmptyContext(dk::StateRegistry&) {}
};