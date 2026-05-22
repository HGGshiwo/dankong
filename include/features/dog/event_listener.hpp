#include <memory>

#include "./command.hpp"
#include "context_config.hpp"
#include "dk/adapters/udp/udp_client.hpp"
#include "dk/engine.hpp"
#include "dk/event_listener.hpp"
#include "dk/report.hpp"

class DogListener : public dk::BaseEventListener<RobotContext, DogListener> {
   public:
    using AllowedEvents = std::tuple<dk::TickEvent>;

    DogListener(RobotContext& ctx)
        : udp_client_(ctx.udp_client), rate_(dk::RateLimiter(3)) {}

    void on_event(const dk::TickEvent& event, RobotContext& ctx) {
        if (rate_.check_and_update()) {
            auto data = pack_cmd(CommandType::MANUAL_HEARTBEAT);
            udp_client_->send(data);
        }
    }

   private:
    std::shared_ptr<UdpClient> udp_client_;
    dk::RateLimiter rate_;
};