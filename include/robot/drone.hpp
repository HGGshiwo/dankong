#include "./robot_base.hpp"

class Drone : public IRobot {
    bool check_hover(bool arm, double rel_alt);
    double get_distance(Eigen::Vector3d pos, Eigen::Vector3d goal);
    bool is_prearm_enable();
    bool is_alt_enable();
    void takeoff(double alt);
    void land();
    void send_cmd(std::optional<Eigen::Vector3d> pos, std::optional<Eigen::Vector3d> vel,
                  std::optional<Eigen::Vector3d> acc, std::optional<double> yaw, std::optional<double> yaw_rate,
                  CmdFrame frame);
};