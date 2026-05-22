#pragma once
#include "context_config.hpp"
#include "dk/engine.hpp"

class Engine : public dk::BaseEngine<RobotContext, Engine> {};
