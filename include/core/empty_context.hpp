#pragma once
#include "utils/state_registry.hpp"
// 一个什么都不做的假 Cont

struct EmptyContext {
    explicit EmptyContext(StateRegistry&) {}
};