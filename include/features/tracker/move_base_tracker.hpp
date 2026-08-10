#pragma once
#include <memory>
#include <string>

#include "features/tracker/tracker.hpp"

#ifdef USE_ROS1
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#elif defined(USE_ROS2)
#include <action_msgs/msg/goal_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

class MoveBaseTracker : public ThreadedTracker {
   private:
    std::string frame_id_ = "map";

#ifdef USE_ROS1
    ros::NodeHandle nh_;
    ros::Publisher goal_pub_;
    ros::Publisher cancel_pub_;
#elif defined(USE_ROS2)
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<action_msgs::msg::GoalInfo>::SharedPtr cancel_pub_;
#endif

   public:
#ifdef USE_ROS1
    MoveBaseTracker(ros::NodeHandle nh, const TrackerConfig& config,
                    ITrackerRuntime* runtime,
                    DirtyVar<Eigen::Vector3d>& pos_enu,
                    std::atomic<double>& yaw_enu,
                    std::shared_ptr<dk::ITimeProvider> time_provider,
                    const std::string& goal_topic = "move_base_simple/goal",
                    const std::string& cancel_topic = "move_base/cancel",
                    const std::string& frame_id = "map", bool use_thread = true)
        : ThreadedTracker(config, runtime, pos_enu, yaw_enu, time_provider,
                          use_thread),
          frame_id_(frame_id),
          nh_(nh) {
        goal_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>(goal_topic, 1, true);
        cancel_pub_ =
            nh_.advertise<actionlib_msgs::GoalID>(cancel_topic, 1, true);
    }
#elif defined(USE_ROS2)
    MoveBaseTracker(std::shared_ptr<rclcpp::Node> node,
                    const TrackerConfig& config, ITrackerRuntime* runtime,
                    DirtyVar<Eigen::Vector3d>& pos_enu,
                    std::atomic<double>& yaw_enu,
                    std::shared_ptr<dk::ITimeProvider> time_provider,
                    const std::string& goal_topic = "goal_pose",
                    const std::string& cancel_topic =
                        "navigate_to_pose/_action/cancel_goal",
                    const std::string& frame_id = "map", bool use_thread = true)
        : ThreadedTracker(config, runtime, pos_enu, yaw_enu, time_provider,
                          use_thread),
          frame_id_(frame_id),
          node_(node) {
        goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            goal_topic, 10);
        cancel_pub_ = node_->create_publisher<action_msgs::msg::GoalInfo>(
            cancel_topic, 10);
    }
#else
    MoveBaseTracker(const TrackerConfig& config, ITrackerRuntime* runtime,
                    DirtyVar<Eigen::Vector3d>& pos_enu,
                    std::atomic<double>& yaw_enu,
                    std::shared_ptr<dk::ITimeProvider> time_provider,
                    const std::string& frame_id = "map", bool use_thread = true)
        : ThreadedTracker(config, runtime, pos_enu, yaw_enu, time_provider,
                          use_thread),
          frame_id_(frame_id) {}
#endif

    ~MoveBaseTracker() override { stop(); }

   protected:
    void on_waypoint_activated(
        const TrackerWaypoint& wp,
        const Eigen::Vector4d& target_pose_enu) override {
        double target_yaw = target_pose_enu.w();
        double half_yaw = target_yaw * 0.5;
        double qz = std::sin(half_yaw);
        double qw = std::cos(half_yaw);

#ifdef USE_ROS1
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = frame_id_;
        goal.pose.position.x = target_pose_enu.x();
        goal.pose.position.y = target_pose_enu.y();
        goal.pose.position.z = target_pose_enu.z();
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = qz;
        goal.pose.orientation.w = qw;
        goal_pub_.publish(goal);
#elif defined(USE_ROS2)
        if (goal_pub_) {
            geometry_msgs::msg::PoseStamped goal;
            if (node_) {
                goal.header.stamp = node_->now();
            }
            goal.header.frame_id = frame_id_;
            goal.pose.position.x = target_pose_enu.x();
            goal.pose.position.y = target_pose_enu.y();
            goal.pose.position.z = target_pose_enu.z();
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = qz;
            goal.pose.orientation.w = qw;
            goal_pub_->publish(goal);
        }
#endif
    }

    void on_cancel_navigation() override {
#ifdef USE_ROS1
        actionlib_msgs::GoalID cancel_msg;
        cancel_pub_.publish(cancel_msg);
#elif defined(USE_ROS2)
        if (cancel_pub_) {
            action_msgs::msg::GoalInfo cancel_msg;
            cancel_pub_->publish(cancel_msg);
        }
#endif
    }

    void on_pos_step(double dt, const Eigen::Vector4d& current_pose,
                     const Eigen::Vector4d& target_pose,
                     const Eigen::Vector4d& body_target_vel) override {
        // 在 move_base 导航模式下，由 ROS move_base / Nav2 local planner
        // 发布速度控制指令 ThreadedTracker 基类在 on_step
        // 中继续负责航点到达判定和多航点自动切点
    }
};
