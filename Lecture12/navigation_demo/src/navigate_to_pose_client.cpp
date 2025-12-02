#include "navigation_demo/navigate_to_pose_client.hpp"

namespace navigation_demo {

NavigateToPoseClient::NavigateToPoseClient(const std::string& node_name)
    : Node(node_name)
{
    // Create the action client
    action_client_ = rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "NavigateToPose action client created");
}

bool NavigateToPoseClient::wait_for_server(std::chrono::seconds timeout)
{
    RCLCPP_INFO(this->get_logger(), 
                "Waiting for action server '/navigate_to_pose'...");

    if (!action_client_->wait_for_action_server(timeout)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Action server not available after waiting");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Action server is available");
    return true;
}

bool NavigateToPoseClient::send_goal(double x, double y, double theta)
{
    using namespace std::placeholders;

    // Check if server is available
    if (!action_client_->action_server_is_ready()) {
        RCLCPP_WARN(this->get_logger(), 
                    "Action server not ready");
        return false;
    }

    // Create goal
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.x = x;
    goal_msg.y = y;
    goal_msg.theta = theta;

    RCLCPP_INFO(this->get_logger(),
                "Sending goal: x=%.2f, y=%.2f, theta=%.2f",
                x, y, theta);

    // Set up callbacks
    auto send_goal_options = 
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&NavigateToPoseClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&NavigateToPoseClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&NavigateToPoseClient::result_callback, this, _1);

    // Send goal asynchronously
    action_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(this->get_logger(), "Goal sent (non-blocking)");

    return true;
}

void NavigateToPoseClient::cancel_goal()
{
    if (current_goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "Requesting goal cancellation...");
        action_client_->async_cancel_goal(current_goal_handle_);
    } else {
        RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
    }
}

void NavigateToPoseClient::goal_response_callback(
    const GoalHandle::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        return;
    }

    current_goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
}

void NavigateToPoseClient::feedback_callback(
    GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    (void)goal_handle;  // Unused

    RCLCPP_INFO(this->get_logger(),
                "Feedback - Position: (%.2f, %.2f), Distance remaining: %.2f m",
                feedback->current_x,
                feedback->current_y,
                feedback->distance_remaining);
}

void NavigateToPoseClient::result_callback(
    const GoalHandle::WrappedResult& result)
{
    current_goal_handle_ = nullptr;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
                        "Goal succeeded! Time: %.2f s, Message: %s",
                        result.result->total_time,
                        result.result->message.c_str());
            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),
                         "Goal aborted: %s",
                         result.result->message.c_str());
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),
                        "Goal canceled: %s",
                        result.result->message.c_str());
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
}

}  // namespace navigation_demo

// Main function with example usage
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<navigation_demo::NavigateToPoseClient>();

    // Wait for server
    if (!node->wait_for_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "Exiting: server unavailable");
        rclcpp::shutdown();
        return 1;
    }

    // Send a goal
    node->send_goal(5.0, 3.0, 1.57);

    // Spin to process callbacks (feedback, result)
    RCLCPP_INFO(node->get_logger(), "Spinning to receive feedback and result...");
    rclcpp::spin(node);

    rclcpp::shutdown();
}