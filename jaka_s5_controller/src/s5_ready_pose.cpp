#include "jaka_s5_controller/s5_joint_utils.hpp"

#include "rclcpp/rclcpp.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr char kPlanningGroup[] = "jaka_s5";
constexpr double kCurrentStateTimeoutSeconds = 10.0;

const std::vector<double> kDefaultJointDegrees(
  jaka_s5_controller::s5::taskReadyPoseDegrees().begin(),
  jaka_s5_controller::s5::taskReadyPoseDegrees().end());

bool isFiniteInRange(double value, double minimum, double maximum)
{
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

int run(const rclcpp::Node::SharedPtr & node)
{
  const bool execute = node->declare_parameter<bool>("execute", false);
  const auto joint_degrees =
    node->declare_parameter<std::vector<double>>("joint_degrees", kDefaultJointDegrees);
  const double velocity_scaling = node->declare_parameter<double>("velocity_scaling", 0.1);
  const double acceleration_scaling =
    node->declare_parameter<double>("acceleration_scaling", 0.1);
  const double planning_time = node->declare_parameter<double>("planning_time", 10.0);

  if (joint_degrees.size() != jaka_s5_controller::s5::jointNames().size()) {
    RCLCPP_ERROR(
      node->get_logger(), "Parameter 'joint_degrees' must contain exactly %zu values; got %zu.",
      jaka_s5_controller::s5::jointNames().size(), joint_degrees.size());
    return 2;
  }
  if (!isFiniteInRange(velocity_scaling, 0.0, 1.0) || velocity_scaling == 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'velocity_scaling' must be in (0.0, 1.0].");
    return 2;
  }
  if (!isFiniteInRange(acceleration_scaling, 0.0, 1.0) || acceleration_scaling == 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'acceleration_scaling' must be in (0.0, 1.0].");
    return 2;
  }
  if (!std::isfinite(planning_time) || planning_time <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'planning_time' must be greater than zero.");
    return 2;
  }
  if (!std::all_of(joint_degrees.begin(), joint_degrees.end(), [](double value) {
      return std::isfinite(value);
    }))
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joint_degrees' contains a non-finite value.");
    return 2;
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);

  const auto robot_model = move_group.getRobotModel();
  const auto * joint_model_group = robot_model->getJointModelGroup(kPlanningGroup);
  if (joint_model_group == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt planning group '%s' was not found.", kPlanningGroup);
    return 3;
  }

  const auto & group_variable_names = joint_model_group->getVariableNames();
  for (const auto & joint_name : jaka_s5_controller::s5::jointNames()) {
    if (std::find(group_variable_names.begin(), group_variable_names.end(), joint_name) ==
      group_variable_names.end())
    {
      RCLCPP_ERROR(
        node->get_logger(), "Planning group '%s' does not contain expected joint '%s'.",
        kPlanningGroup, joint_name.c_str());
      return 3;
    }
  }

  const auto current_state = move_group.getCurrentState(kCurrentStateTimeoutSeconds);
  if (!current_state) {
    RCLCPP_ERROR(
      node->get_logger(), "Unable to get the current robot state within %.1f seconds.",
      kCurrentStateTimeoutSeconds);
    return 4;
  }

  std::map<std::string, double> joint_targets;
  moveit::core::RobotState target_state(*current_state);
  for (std::size_t index = 0; index < jaka_s5_controller::s5::jointNames().size(); ++index) {
    const double radians = jaka_s5_controller::s5::degreesToRadians(joint_degrees[index]);
    joint_targets.emplace(jaka_s5_controller::s5::jointNames()[index], radians);
    target_state.setVariablePosition(jaka_s5_controller::s5::jointNames()[index], radians);
    RCLCPP_INFO(
      node->get_logger(), "%s target: %.3f deg (%.6f rad)",
      jaka_s5_controller::s5::jointNames()[index].c_str(), joint_degrees[index], radians);
  }
  target_state.update();

  if (!target_state.satisfiesBounds(joint_model_group)) {
    RCLCPP_ERROR(node->get_logger(), "The requested ready pose exceeds the S5 joint limits.");
    return 5;
  }

  move_group.setStartState(*current_state);
  if (!move_group.setJointValueTarget(joint_targets)) {
    RCLCPP_ERROR(node->get_logger(), "MoveIt rejected the requested joint target.");
    return 5;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto planning_result = move_group.plan(plan);
  if (planning_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan a collision-free path to the ready pose.");
    return 6;
  }

  if (!execute) {
    RCLCPP_INFO(
      node->get_logger(),
      "Planning succeeded. No motion was executed; launch again with execute:=true to move the robot.");
    return 0;
  }

  RCLCPP_WARN(node->get_logger(), "Executing the planned trajectory to the S5 ready pose.");
  const auto execution_result = move_group.execute(plan);
  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed.");
    return 7;
  }

  RCLCPP_INFO(node->get_logger(), "The S5 reached the requested ready pose.");
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("s5_ready_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner_thread([&executor]() {executor.spin();});

  int exit_code = 1;
  try {
    exit_code = run(node);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(node->get_logger(), "Ready-pose node failed: %s", exception.what());
  }

  executor.cancel();
  if (spinner_thread.joinable()) {
    spinner_thread.join();
  }
  rclcpp::shutdown();
  return exit_code;
}
