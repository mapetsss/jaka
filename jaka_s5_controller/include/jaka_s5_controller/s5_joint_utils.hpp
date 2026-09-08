#ifndef JAKA_PLANNER__S5_JOINT_UTILS_HPP_
#define JAKA_PLANNER__S5_JOINT_UTILS_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace jaka_s5_controller
{
namespace s5
{

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kJointCount = 6;
using JointPositions = std::array<double, kJointCount>;

inline const JointPositions & initialPoseDegrees()
{
  static const JointPositions pose = {
    89.816, 109.950, -132.277, 201.880, 94.806, -74.132};
  return pose;
}

inline const JointPositions & taskReadyPoseDegrees()
{
  static const JointPositions pose = {
    -179.753, 90.057, -90.199, 90.196, 91.724, -64.680};
  return pose;
}

inline const std::array<std::string, kJointCount> & jointNames()
{
  static const std::array<std::string, kJointCount> names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  return names;
}

inline double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

inline double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}

inline JointPositions poseDegreesToRadians(const JointPositions & degrees)
{
  JointPositions radians{};
  for (std::size_t index = 0; index < degrees.size(); ++index) {
    radians[index] = degreesToRadians(degrees[index]);
  }
  return radians;
}

inline bool positionsNear(
  const JointPositions & first,
  const JointPositions & second,
  double tolerance_radians)
{
  if (!std::isfinite(tolerance_radians) || tolerance_radians < 0.0) {
    return false;
  }
  for (std::size_t index = 0; index < first.size(); ++index) {
    if (!std::isfinite(first[index]) || !std::isfinite(second[index]) ||
      std::abs(first[index] - second[index]) > tolerance_radians)
    {
      return false;
    }
  }
  return true;
}

inline bool shouldStartAutomaticInitialPose(
  bool enabled, bool state_connected, bool controls_idle, bool & attempted)
{
  if (!enabled || attempted || !state_connected || !controls_idle) {
    return false;
  }
  attempted = true;
  return true;
}

inline bool reorderJointPositions(
  const std::vector<std::string> & names,
  const std::vector<double> & positions,
  std::array<double, kJointCount> & ordered)
{
  if (names.size() != positions.size()) {
    return false;
  }

  std::unordered_map<std::string, std::size_t> expected;
  for (std::size_t index = 0; index < jointNames().size(); ++index) {
    expected.emplace(jointNames()[index], index);
  }

  std::array<bool, kJointCount> found{};
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto expected_joint = expected.find(names[index]);
    if (expected_joint == expected.end()) {
      continue;
    }
    const auto output_index = expected_joint->second;
    if (found[output_index] || !std::isfinite(positions[index])) {
      return false;
    }
    ordered[output_index] = positions[index];
    found[output_index] = true;
  }

  for (const bool joint_found : found) {
    if (!joint_found) {
      return false;
    }
  }
  return true;
}

inline bool isStateFresh(
  const std::chrono::steady_clock::time_point & received_at,
  const std::chrono::steady_clock::time_point & now,
  std::chrono::milliseconds timeout)
{
  return received_at != std::chrono::steady_clock::time_point{} &&
         now >= received_at && now - received_at <= timeout;
}

inline std::array<double, kJointCount> singleJointTarget(
  const std::array<double, kJointCount> & current,
  std::size_t selected_joint,
  double target_radians)
{
  auto target = current;
  if (selected_joint < target.size()) {
    target[selected_joint] = target_radians;
  }
  return target;
}

inline std::array<double, kJointCount> incrementalJointTarget(
  const std::array<double, kJointCount> & current,
  std::size_t joint_index,
  int direction,
  double step_radians)
{
  auto target = current;
  if (joint_index < target.size() && (direction == -1 || direction == 1) &&
    std::isfinite(step_radians) && step_radians > 0.0)
  {
    target[joint_index] += direction * step_radians;
  }
  return target;
}

}  // namespace s5
}  // namespace jaka_s5_controller

#endif  // JAKA_PLANNER__S5_JOINT_UTILS_HPP_
