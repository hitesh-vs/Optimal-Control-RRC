#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("effort_test_node");

  auto publisher_slider = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/slider/commands", 10);
  auto publisher_slider_y = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_controller/slider_y/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands_slider;
  std_msgs::msg::Float64MultiArray commands_slider_y;

  using namespace std::chrono_literals;
  double values[] = {3, 4, 3, 4};

  commands_slider.data.push_back(0);
  publisher_slider->publish(commands_slider);
  std::this_thread::sleep_for(0.5s);

  for (int i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
    commands_slider.data[0] = values[i+1];
    publisher_slider->publish(commands_slider);
    std::this_thread::sleep_for(0.5s);
  }

  commands_slider_y.data.push_back(0);
  publisher_slider_y->publish(commands_slider_y);
  std::this_thread::sleep_for(0.5s);

  for (int i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
    commands_slider_y.data[0] = values[i+1];
    publisher_slider_y->publish(commands_slider_y);
    std::this_thread::sleep_for(0.5s);
  }

  rclcpp::shutdown();

  return 0;
}