#include "dk/report.hpp"

struct TemplateContext {
    dk::StateRegistry& reg;

   public:
    explicit TemplateContext(dk::StateRegistry& r) : reg(r) {};
};