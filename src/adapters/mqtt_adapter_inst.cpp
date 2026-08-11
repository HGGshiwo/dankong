#include "core/engine.hpp"
#include "dk/adapters/mqtt.hpp"
#include "robot_context.hpp"

template class dk::MqttClientAdapter<RobotContext, Engine>;
