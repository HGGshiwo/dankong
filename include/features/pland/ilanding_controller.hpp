#pragma once
#include "ilanding_detector.hpp"

class ILandingController {
   public:
    virtual ~ILandingController() = default;
    virtual void update_observation(const DetectorResult& result) = 0;
    virtual void start(int hz) = 0;
    virtual void stop() = 0;
};
