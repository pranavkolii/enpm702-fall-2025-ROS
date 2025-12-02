#include "navigation_demo/set_goal_client.hpp"

namespace navigation_demo {

SetGoalClient::SetGoalClient(const std::string& node_name)
    : Node(node_name)
{
    // Create the service client
    client_ = this->create_client<SetGoal>("set_goal");

    RCLCPP_INFO(this->get_logger(), "SetGoal service client created");
}

bool SetGoalClient::wait_for_service(std::chrono::seconds timeout)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for service '/set_goal'...");

    if (!client_->wait_for_service(timeout)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Service '/set_goal' not available after waiting");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Service '/set_goal' is available");
    return true;
}

bool SetGoalClient::send_goal_async(double x, double y, double theta)
{
    // Check if service is available (non-blocking check)
    if (!client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), 
                    "Service not ready, attempting to wait...");
        if (!wait_for_service(std::chrono::seconds(2))) {
            return false;
        }
    }

    // Create request
    auto request = std::make_shared<SetGoal::Request>();
    request->x = x;
    request->y = y;
    request->theta = theta;

    RCLCPP_INFO(this->get_logger(),
                "Sending goal request: x=%.2f, y=%.2f, theta=%.2f",
                x, y, theta);

    // Send request asynchronously with callback
    client_->async_send_request(
        request,
        std::bind(&SetGoalClient::response_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), 
                "Request sent, continuing execution (non-blocking)");

    return true;
}

void SetGoalClient::response_callback(ServiceResponseFuture future)
{
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(),
                    "Goal succeeded: %s", response->message.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "Goal failed: %s", response->message.c_str());
    }
}

}  // namespace navigation_demo

// Main function with example usage
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<navigation_demo::SetGoalClient>();

    // Wait for service to be available
    if (!node->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node->get_logger(), "Exiting: service unavailable");
        rclcpp::shutdown();
        return 1;
    }

    // Send a goal asynchronously
    node->send_goal_async(5.0, 3.0, 1.57);

    // Continue doing other work or spin to process callbacks
    RCLCPP_INFO(node->get_logger(), 
                "Main thread continues while waiting for response...");

    // Spin to allow callback to be processed
    rclcpp::spin(node);

    rclcpp::shutdown();
}