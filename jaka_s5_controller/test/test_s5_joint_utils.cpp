#include "jaka_s5_controller/s5_joint_utils.hpp"
#include "jaka_s5_controller/robot_shutdown_utils.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <limits>
#include <string>
#include <vector>

TEST(S5JointUtils, ConvertsAnglesInBothDirections)
{
  EXPECT_NEAR(jaka_s5_controller::s5::degreesToRadians(180.0), jaka_s5_controller::s5::kPi, 1e-12);
  EXPECT_NEAR(jaka_s5_controller::s5::radiansToDegrees(jaka_s5_controller::s5::kPi / 2.0), 90.0, 1e-12);
}

TEST(S5JointUtils, ProvidesConfiguredInitialAndTaskReadyPoses)
{
  const auto & initial = jaka_s5_controller::s5::initialPoseDegrees();
  const auto & ready = jaka_s5_controller::s5::taskReadyPoseDegrees();
  EXPECT_EQ(initial, (jaka_s5_controller::s5::JointPositions{
    89.816, 109.950, -132.277, 201.880, 94.806, -74.132}));
  EXPECT_EQ(ready, (jaka_s5_controller::s5::JointPositions{
    -179.753, 90.057, -90.199, 90.196, 91.724, -64.680}));

  const auto initial_radians = jaka_s5_controller::s5::poseDegreesToRadians(initial);
  for (std::size_t index = 0; index < initial.size(); ++index) {
    EXPECT_NEAR(
      jaka_s5_controller::s5::radiansToDegrees(initial_radians[index]), initial[index], 1e-12);
  }
}

TEST(S5JointUtils, AutomaticInitialPoseTriggersOnlyOnce)
{
  bool attempted = false;
  EXPECT_FALSE(jaka_s5_controller::s5::shouldStartAutomaticInitialPose(
    true, false, true, attempted));
  EXPECT_FALSE(attempted);
  EXPECT_TRUE(jaka_s5_controller::s5::shouldStartAutomaticInitialPose(
    true, true, true, attempted));
  EXPECT_TRUE(attempted);
  EXPECT_FALSE(jaka_s5_controller::s5::shouldStartAutomaticInitialPose(
    true, true, true, attempted));

  bool disabled_attempted = false;
  EXPECT_FALSE(jaka_s5_controller::s5::shouldStartAutomaticInitialPose(
    false, true, true, disabled_attempted));
  EXPECT_FALSE(disabled_attempted);
}

TEST(S5JointUtils, DetectsWhenNamedPoseIsAlreadyReached)
{
  const auto radians = jaka_s5_controller::s5::poseDegreesToRadians(
    jaka_s5_controller::s5::taskReadyPoseDegrees());
  auto nearby = radians;
  nearby[2] += jaka_s5_controller::s5::degreesToRadians(0.1);
  EXPECT_TRUE(jaka_s5_controller::s5::positionsNear(
    radians, nearby, jaka_s5_controller::s5::degreesToRadians(0.2)));
  nearby[2] += jaka_s5_controller::s5::degreesToRadians(0.2);
  EXPECT_FALSE(jaka_s5_controller::s5::positionsNear(
    radians, nearby, jaka_s5_controller::s5::degreesToRadians(0.2)));
}

TEST(S5JointUtils, ReordersPositionsByJointName)
{
  const std::vector<std::string> names = {
    "joint_6", "joint_4", "joint_2", "joint_5", "joint_1", "joint_3"};
  const std::vector<double> positions = {6.0, 4.0, 2.0, 5.0, 1.0, 3.0};
  std::array<double, jaka_s5_controller::s5::kJointCount> ordered{};
  ASSERT_TRUE(jaka_s5_controller::s5::reorderJointPositions(names, positions, ordered));
  for (std::size_t index = 0; index < ordered.size(); ++index) {
    EXPECT_DOUBLE_EQ(ordered[index], static_cast<double>(index + 1));
  }
}

TEST(S5JointUtils, RejectsMissingDuplicateAndInvalidPositions)
{
  std::array<double, jaka_s5_controller::s5::kJointCount> ordered{};
  EXPECT_FALSE(jaka_s5_controller::s5::reorderJointPositions(
    {"joint_1", "joint_2"}, {0.0, 0.0}, ordered));
  EXPECT_FALSE(jaka_s5_controller::s5::reorderJointPositions(
    {"joint_1", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, ordered));
  EXPECT_FALSE(jaka_s5_controller::s5::reorderJointPositions(
    {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"},
    {0.0, 0.0, std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0}, ordered));
}

TEST(S5JointUtils, DetectsFreshAndStaleState)
{
  const auto received = std::chrono::steady_clock::now();
  EXPECT_TRUE(jaka_s5_controller::s5::isStateFresh(
    received, received + std::chrono::milliseconds(999), std::chrono::seconds(1)));
  EXPECT_FALSE(jaka_s5_controller::s5::isStateFresh(
    received, received + std::chrono::milliseconds(1001), std::chrono::seconds(1)));
}

TEST(S5JointUtils, ChangesOnlySelectedJoint)
{
  const std::array<double, jaka_s5_controller::s5::kJointCount> current = {1, 2, 3, 4, 5, 6};
  const auto target = jaka_s5_controller::s5::singleJointTarget(current, 2, 30.0);
  EXPECT_EQ(target, (std::array<double, jaka_s5_controller::s5::kJointCount>{1, 2, 30, 4, 5, 6}));
}

TEST(S5JointUtils, MapsIncrementButtonsToEveryJointAndDirection)
{
  const std::array<double, jaka_s5_controller::s5::kJointCount> current{};
  const double step = jaka_s5_controller::s5::degreesToRadians(1.0);
  for (std::size_t joint = 0; joint < current.size(); ++joint) {
    for (const int direction : {-1, 1}) {
      const auto target = jaka_s5_controller::s5::incrementalJointTarget(
        current, joint, direction, step);
      for (std::size_t index = 0; index < target.size(); ++index) {
        EXPECT_DOUBLE_EQ(target[index], index == joint ? direction * step : 0.0);
      }
    }
  }
}

TEST(RobotShutdownUtils, RunsCompleteShutdownInSafetyOrder)
{
  std::vector<std::string> calls;
  jaka_s5_controller::RobotShutdownOperations operations{
    [&calls]() {calls.push_back("stop_motion"); return 0;},
    [&calls]() {calls.push_back("disable_servo"); return 0;},
    [&calls]() {calls.push_back("disable_robot"); return 0;},
    [&calls]() {calls.push_back("power_off"); return 0;},
    [&calls]() {calls.push_back("logout"); return 0;}};

  jaka_s5_controller::RobotShutdownState state;
  const auto result = jaka_s5_controller::runRobotShutdownSequence(true, true, operations, state);

  EXPECT_TRUE(result.success());
  EXPECT_EQ(calls, (std::vector<std::string>{
    "stop_motion", "disable_servo", "disable_robot", "power_off", "logout"}));
}

TEST(RobotShutdownUtils, SkipsDisableAndPowerWhenAutomaticPowerOffIsDisabled)
{
  std::vector<std::string> calls;
  jaka_s5_controller::RobotShutdownOperations operations{
    [&calls]() {calls.push_back("stop_motion"); return 0;},
    [&calls]() {calls.push_back("disable_servo"); return 0;},
    [&calls]() {calls.push_back("disable_robot"); return 0;},
    [&calls]() {calls.push_back("power_off"); return 0;},
    [&calls]() {calls.push_back("logout"); return 0;}};

  jaka_s5_controller::RobotShutdownState state;
  const auto result = jaka_s5_controller::runRobotShutdownSequence(false, true, operations, state);

  EXPECT_TRUE(result.success());
  EXPECT_EQ(calls, (std::vector<std::string>{"stop_motion", "disable_servo", "logout"}));
}

TEST(RobotShutdownUtils, ContinuesCleanupAfterAnSdkFailure)
{
  std::vector<std::string> calls;
  jaka_s5_controller::RobotShutdownOperations operations{
    [&calls]() {calls.push_back("stop_motion"); return 0;},
    [&calls]() {calls.push_back("disable_servo"); return -8;},
    [&calls]() {calls.push_back("disable_robot"); return 0;},
    [&calls]() {calls.push_back("power_off"); return 0;},
    [&calls]() {calls.push_back("logout"); return 0;}};

  jaka_s5_controller::RobotShutdownState state;
  const auto result = jaka_s5_controller::runRobotShutdownSequence(true, true, operations, state);

  EXPECT_FALSE(result.success());
  EXPECT_NE(result.summary().find("disable_servo=-8"), std::string::npos);
  EXPECT_EQ(calls.back(), "logout");
}

TEST(RobotShutdownUtils, DoesNotRepeatCompletedSdkOperations)
{
  std::vector<std::string> calls;
  jaka_s5_controller::RobotShutdownOperations operations{
    [&calls]() {calls.push_back("stop_motion"); return 0;},
    [&calls]() {calls.push_back("disable_servo"); return 0;},
    [&calls]() {calls.push_back("disable_robot"); return 0;},
    [&calls]() {calls.push_back("power_off"); return 0;},
    [&calls]() {calls.push_back("logout"); return 0;}};
  jaka_s5_controller::RobotShutdownState state;

  EXPECT_TRUE(jaka_s5_controller::runRobotShutdownSequence(
    true, true, operations, state).success());
  EXPECT_TRUE(jaka_s5_controller::runRobotShutdownSequence(
    true, true, operations, state).success());

  EXPECT_EQ(calls, (std::vector<std::string>{
    "stop_motion", "disable_servo", "disable_robot", "power_off", "logout"}));
}
