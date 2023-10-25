#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("effort_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/effort_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;

  commands.data.push_back(0);
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 3.0898;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 3.3942;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.6687;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);
  
  commands.data[0] = -0.2826;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0700;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0025;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0013;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0025;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0013;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0025;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0012;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0024;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0012;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0011;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -1.6079;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0011;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0012;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0024;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0012;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0025;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0013;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0025;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.0013;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0025;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.0700;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0.2826;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -0.6687;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -3.3942;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = -3.0898;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.2786s);

  commands.data[0] = 0;
  publisher->publish(commands);



  return 0;
}