#pragma once

#include <memory>
#include <string>
#include <functional>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include "robot_custom_interfaces/srv/set_goal.hpp"

namespace navigation_demo {

class SetGoalClient : public rclcpp::Node {
public:
    // Type aliases for convenience
    using SetGoal = robot_custom_interfaces::srv::SetGoal;
    using ServiceResponseFuture = rclcpp::Client<SetGoal>::SharedFuture;

    /**
     * @brief Construct the SetGoal service client
     * @param node_name Name of the node
     */
    explicit SetGoalClient(const std::string& node_name = "set_goal_client");

    /**
     * @brief Destructor
     */
    ~SetGoalClient() = default;

    /**
     * @brief Send a goal request asynchronously
     * @param x Target X coordinate
     * @param y Target Y coordinate
     * @param theta Target orientation
     * @return true if request was sent, false if service unavailable
     */
    bool send_goal_async(double x, double y, double theta);

    /**
     * @brief Check if the service is available
     * @param timeout Timeout duration
     * @return true if service is available
     */
    bool wait_for_service(std::chrono::seconds timeout = std::chrono::seconds(5));

private:
    /**
     * @brief Callback invoked when service response is received
     * @param future The future containing the service response
     */
    void response_callback(ServiceResponseFuture future);

    // Service client
    rclcpp::Client<SetGoal>::SharedPtr client_;
};

}  // namespace navigation_demo