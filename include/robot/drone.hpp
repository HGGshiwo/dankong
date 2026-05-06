#include "./robot_base.hpp"
#include "mavlink/mavros.hpp"

template <typename MavlinkType>
class Drone : public IRobot<MavlinkType> {
    bool check_hover(bool arm, double rel_alt) override;
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal);
    bool is_prearm_enable();
    bool is_alt_enable();
    bool send_cmd(std::optional<Eigen::Vector3d> pos, std::optional<Eigen::Vector3d> vel,
                  std::optional<Eigen::Vector3d> acc, std::optional<double> yaw, std::optional<double> yaw_rate,
                  CmdFrame frame);
};

inline double HOVER_THRESH = 1.0;

template <typename MavlinkType>
double Drone<MavlinkType>::get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) {
    return (pos - goal).norm();
}

template <typename MavlinkType>
bool Drone<MavlinkType>::is_prearm_enable() {
    return true;
}

template <typename MavlinkType>
bool Drone<MavlinkType>::is_alt_enable() {
    return true;
}

template <typename MavlinkType>
bool Drone<MavlinkType>::send_cmd(std::optional<Eigen::Vector3d> pos, std::optional<Eigen::Vector3d> vel,
                                  std::optional<Eigen::Vector3d> acc, std::optional<double> yaw,
                                  std::optional<double> yaw_rate, CmdFrame frame) {
    return true;
}

template <typename MavlinkType>
bool Drone<MavlinkType>::check_hover(bool arm, double rel_alt) {
    return arm && rel_alt > HOVER_THRESH;
}