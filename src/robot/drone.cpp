#include "robot/drone.hpp"
double HOVER_THRESH = 1.0;

double Drone::get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal) {
    return (pos - goal).norm();
}

bool Drone::is_prearm_enable() {
    return true;
}

bool Drone::is_alt_enable() {
    return true;
}
void Drone::takeoff(double alt) {
    return;
}
void Drone::land() {
    return;
}
void Drone::send_cmd(std::optional<Eigen::Vector3d> pos, std::optional<Eigen::Vector3d> vel,
                     std::optional<Eigen::Vector3d> acc, std::optional<double> yaw, std::optional<double> yaw_rate,
                     CmdFrame frame) {
    return;
}

bool Drone::check_hover(bool arm, double rel_alt) {
    return arm && rel_alt > HOVER_THRESH;
}