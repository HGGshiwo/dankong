#pragma once
#include <memory>

#include "context_config.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/report.hpp"
#include "mavlink/mavros.hpp"

struct DroneContext {
    std::shared_ptr<IRobot> robot;
    dk::StateRegistry& reg;

    std::atomic<double> throttle = -1.0;
    std::atomic<double> rangefinder_alt = -1.0;

   public:
    explicit DroneContext(dk::StateRegistry& r) : reg(r) {};
    static constexpr std::string_view ROBOT_TYPE = "DRONE";
};