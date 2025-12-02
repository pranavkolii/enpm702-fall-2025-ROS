#include "navigation_demo/set_goal_server.hpp"

#include <cmath>
#include <functional>

namespace navigation_demo {

SetGoalServer::SetGoalServer(const std::string& node_name)
    : Node(node_name)
{
    // Create the service server
    service_ = this->create_service<robot_custom_interfaces::srv::SetGoal>(
        "set_goal",
        std::bind(&SetGoalServer::handle_set_goal, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), 
                "SetGoal service server started on '/set_goal'");
}

void SetGoalServer::handle_set_goal(
    const std::shared_ptr<robot_custom_interfaces::srv::SetGoal::Request> request,
    std::shared_ptr<robot_custom_interfaces::srv::SetGoal::Response> response)
{
    RCLCPP_INFO(this->get_logger(),
                "Received goal request: x=%.2f, y=%.2f, theta=%.2f",
                request->x, request->y, request->theta);

    // Validate the goal
    if (!validate_goal(request->x, request->y, request->theta)) {
        response->success = false;
        response->message = "Goal coordinates out of bounds";
        RCLCPP_WARN(this->get_logger(), "Goal rejected: %s", 
                    response->message.c_str());
        return;
    }

    // Goal is valid - in a real application, this is where you would
    // trigger the action client to navigate to the pose
    response->success = true;
    response->message = "Goal accepted: navigating to (" +
                        std::to_string(request->x) + ", " +
                        std::to_string(request->y) + ")";

    RCLCPP_INFO(this->get_logger(), "Goal accepted and processed");
}

bool SetGoalServer::validate_goal(double x, double y, double theta) const
{
    // Check coordinate boundaries
    if (x < kMinX || x > kMaxX || y < kMinY || y > kMaxY) {
        return false;
    }

    // Normalize theta check (optional)
    if (theta < -M_PI || theta > M_PI) {
        return false;
    }

    return true;
}

}  // namespace navigation_demo

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<navigation_demo::SetGoalServer>();

    RCLCPP_INFO(node->get_logger(), "Spinning SetGoal server node...");

    rclcpp::spin(node);

    rclcpp::shutdown();
}