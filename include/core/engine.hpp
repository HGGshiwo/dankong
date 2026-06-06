#pragma once
#include "dk/engine.hpp"
#include "robot_context.hpp"

class Engine : public dk::BaseEngine<RobotContext, Engine> {
    using BaseEngine = dk::BaseEngine<RobotContext, Engine>;
    using BaseEngine::BaseEngine;
};
