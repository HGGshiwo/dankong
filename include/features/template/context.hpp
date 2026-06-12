#include "utils/state_registry.hpp"

struct TemplateContext {
    StateRegistry& reg;

   public:
    explicit TemplateContext(StateRegistry& r) : reg(r) {};
};