#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "robot_custom_interfaces/srv/set_goal.hpp"

namespace navigation_demo {

class SetGoalServer : public rclcpp::Node {
public:
    /**
     * @brief Construct the SetGoal service server
     * @param node_name Name of the node
     */
    explicit SetGoalServer(const std::string& node_name = "set_goal_server");

    /**
     * @brief Destructor
     */
    ~SetGoalServer() = default;

private:
    /**
     * @brief Handle incoming SetGoal service requests
     * @param request The service request containing x, y, theta
     * @param response The service response with success status and message
     */
    void handle_set_goal(
        const std::shared_ptr<robot_custom_interfaces::srv::SetGoal::Request> request,
        std::shared_ptr<robot_custom_interfaces::srv::SetGoal::Response> response);

    /**
     * @brief Validate the goal coordinates
     * @param x X coordinate
     * @param y Y coordinate
     * @param theta Orientation angle
     * @return true if goal is valid, false otherwise
     */
    bool validate_goal(double x, double y, double theta) const;

    // Service server
    rclcpp::Service<robot_custom_interfaces::srv::SetGoal>::SharedPtr service_;

    // Goal boundaries (example constraints)
    static constexpr double kMinX = -10.0;
    static constexpr double kMaxX = 10.0;
    static constexpr double kMinY = -10.0;
    static constexpr double kMaxY = 10.0;
};

}  // namespace navigation_demo