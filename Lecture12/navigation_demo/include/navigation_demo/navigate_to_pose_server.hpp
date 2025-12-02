#pragma once

#include <memory>
#include <string>
#include <thread>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_custom_interfaces/action/navigate_to_pose.hpp"

namespace navigation_demo {

class NavigateToPoseServer : public rclcpp::Node {
public:
    // Type aliases
    using NavigateToPose = robot_custom_interfaces::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    /**
     * @brief Construct the NavigateToPose action server
     * @param node_name Name of the node
     */
    explicit NavigateToPoseServer(
        const std::string& node_name = "navigate_to_pose_server");

    /**
     * @brief Destructor
     */
    ~NavigateToPoseServer() = default;

private:
    /**
     * @brief Handle goal request
     * @param uuid Goal unique identifier
     * @param goal The goal request
     * @return Response indicating accept or reject
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal);

    /**
     * @brief Handle cancel request
     * @param goal_handle Handle to the goal being canceled
     * @return Response indicating if cancel is accepted
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Handle accepted goal
     * @param goal_handle Handle to the accepted goal
     */
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Execute the navigation (runs in separate thread)
     * @param goal_handle Handle to the goal being executed
     */
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Compute distance between two points
     * @param x1 First point X
     * @param y1 First point Y
     * @param x2 Second point X
     * @param y2 Second point Y
     * @return Euclidean distance
     */
    double compute_distance(double x1, double y1, double x2, double y2) const;

    // Action server
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

    // Simulated robot position
    double current_x_{0.0};
    double current_y_{0.0};
    double current_theta_{0.0};

    // Navigation parameters
    static constexpr double kGoalTolerance = 0.1;       // meters
    static constexpr double kMaxLinearVelocity = 0.5;   // m/s
    static constexpr double kControlRate = 10.0;        // Hz
};

}  // namespace navigation_demo