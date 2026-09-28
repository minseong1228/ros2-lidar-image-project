#include <iostream>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "embedded_project/srv/set_roi.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("roi_selector");

  int32_t x1, y1, x2, y2;
  std::cout << "Enter ROI (x1 y1 x2 y2): ";
  std::cin >> x1 >> y1 >> x2 >> y2;

auto client = node->create_client<embedded_project::srv::SetROI>("SetROI");

if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available.");
    rclcpp::shutdown();
    return 1;
}
  auto req = std::make_shared<embedded_project::srv::SetROI::Request>();
  req->x1 = x1;
  req->y1 = y1;
  req->x2 = x2;
  req->y2 = y2;

  RCLCPP_INFO(node->get_logger(), "ROI 요청: (%d, %d) ~ (%d, %d)", x1, y1, x2, y2);

auto result = client->async_send_request(req);

if (rclcpp::spin_until_future_complete(node, result,5s) ==
    rclcpp::FutureReturnCode::SUCCESS) {

    auto response = result.get();

    if (response->success) {
        RCLCPP_INFO(node->get_logger(), "Success: %s", response->message.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "Fail: %s", response->message.c_str());
    }

} else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed.");
}
  rclcpp::shutdown();
  return 0;
}
