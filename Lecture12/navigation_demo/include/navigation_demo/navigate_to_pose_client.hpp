#pragma once

#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_custom_interfaces/action/navigate_to_pose.hpp"

namespace navigation_demo {

class NavigateToPoseClient : public rclcpp::Node {
public:
    // Type aliases
    using NavigateToPose = robot_custom_interfaces::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Construct the NavigateToPose action client
     * @param node_name Name of the node
     */
    explicit NavigateToPoseClient(
        const std::string& node_name = "navigate_to_pose_client");

    /**
     * @brief Destructor
     */
    ~NavigateToPoseClient() = default;

    /**
     * @brief Send a navigation goal asynchronously
     * @param x Target X coordinate
     * @param y Target Y coordinate
     * @param theta Target orientation
     * @return true if goal was sent, false if server unavailable
     */
    bool send_goal(double x, double y, double theta);

    /**
     * @brief Cancel the current goal
     */
    void cancel_goal();

    /**
     * @brief Check if action server is available
     * @param timeout Timeout duration
     * @return true if server is available
     */
    bool wait_for_server(std::chrono::seconds timeout = std::chrono::seconds(10));

private:
    /**
     * @brief Callback for goal response (accepted/rejected)
     * @param goal_handle Handle to the goal
     */
    void goal_response_callback(const GoalHandle::SharedPtr& goal_handle);

    /**
     * @brief Callback for feedback during execution
     * @param goal_handle Handle to the goal
     * @param feedback The feedback message
     */
    void feedback_callback(
        GoalHandle::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);

    /**
     * @brief Callback for final result
     * @param result The wrapped result
     */
    void result_callback(const GoalHandle::WrappedResult& result);

    // Action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    // Current goal handle (for cancellation)
    GoalHandle::SharedPtr current_goal_handle_;
};

}  // namespace navigation_demo