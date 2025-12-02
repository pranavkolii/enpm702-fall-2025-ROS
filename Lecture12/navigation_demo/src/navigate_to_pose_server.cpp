#include "navigation_demo/navigate_to_pose_server.hpp"

#include <functional>

namespace navigation_demo {

NavigateToPoseServer::NavigateToPoseServer(const std::string& node_name)
    : Node(node_name)
{
    using namespace std::placeholders;

    // Create the action server
    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        "navigate_to_pose",
        std::bind(&NavigateToPoseServer::handle_goal, this, _1, _2),
        std::bind(&NavigateToPoseServer::handle_cancel, this, _1),
        std::bind(&NavigateToPoseServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "NavigateToPose action server started on '/navigate_to_pose'");
}

rclcpp_action::GoalResponse NavigateToPoseServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal)
{
    (void)uuid;  // Unused

    RCLCPP_INFO(this->get_logger(),
                "Received goal: x=%.2f, y=%.2f, theta=%.2f",
                goal->x, goal->y, goal->theta);

    // Validate goal (example: reject if too far)
    double distance = compute_distance(current_x_, current_y_, 
                                       goal->x, goal->y);
    if (distance > 20.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Goal rejected: distance %.2f exceeds maximum", distance);
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigateToPoseServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    (void)goal_handle;  // Unused

    RCLCPP_INFO(this->get_logger(), "Received cancel request");

    // Always accept cancellation in this example
    return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigateToPoseServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal accepted, starting execution");

    // Execute in a separate thread to avoid blocking
    std::thread{
        std::bind(&NavigateToPoseServer::execute, this, goal_handle)
    }.detach();
}

void NavigateToPoseServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing navigation...");

    // Get goal
    const auto goal = goal_handle->get_goal();

    // Prepare feedback and result
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    // Control loop timing
    rclcpp::Rate rate(kControlRate);
    auto start_time = this->now();

    // Simulated navigation loop
    while (rclcpp::ok()) {
        // Check for cancellation
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->total_time = (this->now() - start_time).seconds();
            result->message = "Navigation canceled";

            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        // Compute distance to goal
        double distance = compute_distance(current_x_, current_y_,
                                           goal->x, goal->y);

        // Check if goal reached
        if (distance < kGoalTolerance) {
            break;
        }

        // Simulate movement toward goal
        double direction_x = (goal->x - current_x_) / distance;
        double direction_y = (goal->y - current_y_) / distance;

        double step = kMaxLinearVelocity / kControlRate;
        current_x_ += direction_x * step;
        current_y_ += direction_y * step;

        // Publish feedback
        feedback->current_x = current_x_;
        feedback->current_y = current_y_;
        feedback->distance_remaining = distance;

        goal_handle->publish_feedback(feedback);

        RCLCPP_DEBUG(this->get_logger(),
                     "Feedback: pos=(%.2f, %.2f), dist=%.2f",
                     current_x_, current_y_, distance);

        rate.sleep();
    }

    // Goal reached
    if (rclcpp::ok()) {
        current_theta_ = goal->theta;

        result->success = true;
        result->total_time = (this->now() - start_time).seconds();
        result->message = "Goal reached successfully";

        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(),
                    "Goal succeeded in %.2f seconds", result->total_time);
    }
}

// this should be static
double NavigateToPoseServer::compute_distance(
    double x1, double y1, double x2, double y2) const
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

}  // namespace navigation_demo

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<navigation_demo::NavigateToPoseServer>();

    RCLCPP_INFO(node->get_logger(), "Spinning NavigateToPose server node...");

    rclcpp::spin(node);

    rclcpp::shutdown();
}