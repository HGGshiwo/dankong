#pragma once
#include <memory>

#include "./ntrip_client.hpp"
#include "utils/dirty_var.hpp"
#include "utils/state_registry.hpp"

struct NtripContext {
    std::shared_ptr<NtripClient> ntrip_client;
    DirtyVar<bool> ntrip_running{false};

   public:
    explicit NtripContext(StateRegistry& reg) {
        reg.bind("ntrip_running", ntrip_running, 10.0);
    }
};