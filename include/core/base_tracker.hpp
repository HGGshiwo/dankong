#pragma once
#include <Eigen/Dense>

enum class CmdFrame {
    BODY,  // 机体坐标系 (x向前, y向左, z向上)
    ENU    // 东北天全局坐标系 (x向东, y向北, z向上)
};

class ITrackerRuntime {
   public:
    virtual bool cmd_vel(Eigen::Vector4d) = 0;
};