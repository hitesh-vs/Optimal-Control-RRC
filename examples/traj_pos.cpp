#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

std::shared_ptr<rclcpp::Node> node;
bool common_goal_accepted = false;
rclcpp_action::ResultCode common_resultcode = rclcpp_action::ResultCode::UNKNOWN;
int common_action_result_code = control_msgs::action::FollowJointTrajectory_Result::SUCCESSFUL;

void common_goal_response(
  rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::SharedPtr goal_handle)
{
  RCLCPP_DEBUG(
    node->get_logger(), "common_goal_response time: %f",
    rclcpp::Clock().now().seconds());
  if (!goal_handle) {
    common_goal_accepted = false;
    printf("Goal rejected\n");
  } else {
    common_goal_accepted = true;
    printf("Goal accepted\n");
  }
}

void common_result_response(
  const rclcpp_action::ClientGoalHandle
  <control_msgs::action::FollowJointTrajectory>::WrappedResult & result)
{
  printf("common_result_response time: %f\n", rclcpp::Clock().now().seconds());
  common_resultcode = result.code;
  common_action_result_code = result.result->error_code;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      printf("SUCCEEDED result code\n");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      printf("Goal was aborted\n");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      printf("Goal was canceled\n");
      return;
    default:
      printf("Unknown result code\n");
      return;
  }
}

void common_feedback(
  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr,
  const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
{
  std::cout << "feedback->desired.positions :";
  for (auto & x : feedback->desired.positions) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
  std::cout << "feedback->desired.velocities :";
  for (auto & x : feedback->desired.velocities) {
    std::cout << x << "\t";
  }
  std::cout << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("trajectory_test_node");

  std::cout << "node created" << std::endl;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;
  action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node->get_node_base_interface(),
    node->get_node_graph_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "/joint_trajectory_controller/follow_joint_trajectory");

  bool response =
    action_client->wait_for_action_server(std::chrono::seconds(1));
  if (!response) 
    throw std::runtime_error("could not get action server");
  
  std::cout << "Created action server" << std::endl;

  std::vector<std::string> joint_names = {"slider"};

  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(0.0);  // start asap
  point.positions.resize(joint_names.size());

  point.positions[0] = 0.0;

  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.time_from_start = rclcpp::Duration::from_seconds(0.2786);
  point2.positions.resize(joint_names.size());
  point2.positions[0] = 0.0580;

  trajectory_msgs::msg::JointTrajectoryPoint point3;
  point3.time_from_start = rclcpp::Duration::from_seconds(0.5571);
  point3.positions.resize(joint_names.size());
  point3.positions[0] = 0.3197;

  trajectory_msgs::msg::JointTrajectoryPoint point4;
  point4.time_from_start = rclcpp::Duration::from_seconds(0.8357);
  point4.positions.resize(joint_names.size());
  point4.positions[0] = 0.8121;

  trajectory_msgs::msg::JointTrajectoryPoint point5;
  point5.time_from_start = rclcpp::Duration::from_seconds(1.1142);
  point5.positions.resize(joint_names.size());
  point5.positions[0] = 1.3793;

  trajectory_msgs::msg::JointTrajectoryPoint point6;
  point6.time_from_start = rclcpp::Duration::from_seconds(1.3928);
  point6.positions.resize(joint_names.size());
  point6.positions[0] = 1.9327;

  trajectory_msgs::msg::JointTrajectoryPoint point7;
  point7.time_from_start = rclcpp::Duration::from_seconds(1.6713);
  point7.positions.resize(joint_names.size());
  point7.positions[0] = 2.4861;

  trajectory_msgs::msg::JointTrajectoryPoint point8;
  point8.time_from_start = rclcpp::Duration::from_seconds(1.9499);
  point8.positions.resize(joint_names.size());
  point8.positions[0] = 3.0432;

  trajectory_msgs::msg::JointTrajectoryPoint point9;
  point9.time_from_start = rclcpp::Duration::from_seconds(2.2284);
  point9.positions.resize(joint_names.size());
  point9.positions[0] = 3.6003;

  trajectory_msgs::msg::JointTrajectoryPoint point10;
  point10.time_from_start = rclcpp::Duration::from_seconds(2.5070);
  point10.positions.resize(joint_names.size());
  point10.positions[0] = 4.1574;

  trajectory_msgs::msg::JointTrajectoryPoint point11;
  point11.time_from_start = rclcpp::Duration::from_seconds(2.7855);
  point11.positions.resize(joint_names.size());
  point11.positions[0] = 4.7145; 

  trajectory_msgs::msg::JointTrajectoryPoint point12;
  point12.time_from_start = rclcpp::Duration::from_seconds(3.0641);
  point12.positions.resize(joint_names.size());
  point12.positions[0] = 5.2716; 

  trajectory_msgs::msg::JointTrajectoryPoint point13;
  point13.time_from_start = rclcpp::Duration::from_seconds(3.3426);
  point13.positions.resize(joint_names.size());
  point13.positions[0] = 5.8287; 

  trajectory_msgs::msg::JointTrajectoryPoint point14;
  point14.time_from_start = rclcpp::Duration::from_seconds(3.6212);
  point14.positions.resize(joint_names.size());
  point14.positions[0] = 6.3858; 

  trajectory_msgs::msg::JointTrajectoryPoint point15;
  point15.time_from_start = rclcpp::Duration::from_seconds(3.8997);
  point15.positions.resize(joint_names.size());
  point15.positions[0] = 6.9429;

  trajectory_msgs::msg::JointTrajectoryPoint point16;
  point16.time_from_start = rclcpp::Duration::from_seconds(4.1783);
  point16.positions.resize(joint_names.size());
  point16.positions[0] = 7.500;

  trajectory_msgs::msg::JointTrajectoryPoint point17;
  point17.time_from_start = rclcpp::Duration::from_seconds(4.4568);
  point17.positions.resize(joint_names.size());
  point17.positions[0] = 8.0571;

  trajectory_msgs::msg::JointTrajectoryPoint point18;
  point18.time_from_start = rclcpp::Duration::from_seconds(4.7354);
  point18.positions.resize(joint_names.size());
  point18.positions[0] = 8.6142;

  trajectory_msgs::msg::JointTrajectoryPoint point19;
  point19.time_from_start = rclcpp::Duration::from_seconds(5.0139);
  point19.positions.resize(joint_names.size());
  point19.positions[0] = 9.1713;

  trajectory_msgs::msg::JointTrajectoryPoint point20;
  point20.time_from_start = rclcpp::Duration::from_seconds(5.2925);
  point20.positions.resize(joint_names.size());
  point20.positions[0] = 9.7824;

  trajectory_msgs::msg::JointTrajectoryPoint point21;
  point21.time_from_start = rclcpp::Duration::from_seconds(5.5710);
  point21.positions.resize(joint_names.size());
  point21.positions[0] = 10.2855;

  trajectory_msgs::msg::JointTrajectoryPoint point22;
  point22.time_from_start = rclcpp::Duration::from_seconds(5.8496);
  point22.positions.resize(joint_names.size());
  point22.positions[0] = 10.8426;

  trajectory_msgs::msg::JointTrajectoryPoint point23;
  point23.time_from_start = rclcpp::Duration::from_seconds(6.1281);
  point23.positions.resize(joint_names.size());
  point23.positions[0] = 11.3997;

  trajectory_msgs::msg::JointTrajectoryPoint point24;
  point24.time_from_start = rclcpp::Duration::from_seconds(6.4067);
  point24.positions.resize(joint_names.size());
  point24.positions[0] = 11.9568;

  trajectory_msgs::msg::JointTrajectoryPoint point25;
  point25.time_from_start = rclcpp::Duration::from_seconds(6.6852);
  point25.positions.resize(joint_names.size());
  point25.positions[0] = 12.5139;  

  trajectory_msgs::msg::JointTrajectoryPoint point26;
  point26.time_from_start = rclcpp::Duration::from_seconds(6.9638);
  point26.positions.resize(joint_names.size());
  point26.positions[0] = 13.0673;

  trajectory_msgs::msg::JointTrajectoryPoint point27;
  point27.time_from_start = rclcpp::Duration::from_seconds(7.2423);
  point27.positions.resize(joint_names.size());
  point27.positions[0] = 13.6207;

  trajectory_msgs::msg::JointTrajectoryPoint point28;
  point28.time_from_start = rclcpp::Duration::from_seconds(7.5209);
  point28.positions.resize(joint_names.size());
  point28.positions[0] = 14.1879;

  trajectory_msgs::msg::JointTrajectoryPoint point29;
  point29.time_from_start = rclcpp::Duration::from_seconds(7.7994);
  point29.positions.resize(joint_names.size());
  point29.positions[0] = 14.6803;

  trajectory_msgs::msg::JointTrajectoryPoint point30;
  point30.time_from_start = rclcpp::Duration::from_seconds(8.0780);
  point30.positions.resize(joint_names.size());
  point30.positions[0] = 14.9420;

  trajectory_msgs::msg::JointTrajectoryPoint point31;
  point31.time_from_start = rclcpp::Duration::from_seconds(8.3565);
  point31.positions.resize(joint_names.size());
  point31.positions[0] = 15;

  points.push_back(point);
  points.push_back(point2);
  points.push_back(point3);
  points.push_back(point4);
  points.push_back(point5);
  points.push_back(point6);
  points.push_back(point7);
  points.push_back(point8);
  points.push_back(point9);
  points.push_back(point10);
  points.push_back(point11);
  points.push_back(point12);
  points.push_back(point13);
  points.push_back(point14);
  points.push_back(point15);
  points.push_back(point16);
  points.push_back(point17);
  points.push_back(point18);
  points.push_back(point19);
  points.push_back(point20);
  points.push_back(point21);
  points.push_back(point22);
  points.push_back(point23);
  points.push_back(point24);
  points.push_back(point25);
  points.push_back(point26);
  points.push_back(point27);
  points.push_back(point28);
  points.push_back(point29);
  points.push_back(point30);
  points.push_back(point31);
 
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
  opt.goal_response_callback = std::bind(common_goal_response, std::placeholders::_1);
  opt.result_callback = std::bind(common_result_response, std::placeholders::_1);
  opt.feedback_callback = std::bind(common_feedback, std::placeholders::_1, std::placeholders::_2);

  control_msgs::action::FollowJointTrajectory_Goal goal_msg;
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(1.0);
  goal_msg.trajectory.joint_names = joint_names;
  goal_msg.trajectory.points = points;

  auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "send goal call failed :(");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "send goal call ok :)");

  rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr
    goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    action_client.reset();
    node.reset();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Goal was accepted by server");

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);
  RCLCPP_INFO(node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "get result call failed :(");
    return 1;
  }

  action_client.reset();
  node.reset();
  rclcpp::shutdown();

  return 0;
}
