#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <thread>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

geometry_msgs::msg::Pose to_pose(const Eigen::Isometry3d& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond q(transform.rotation());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

int run_sequence(const rclcpp::Node::SharedPtr& node)
{
  const bool execute = node->get_parameter("execute").as_bool();
  const bool clockwise = node->get_parameter("clockwise").as_bool();
  const double direction = clockwise ? -1.0 : 1.0;
  const double dwell = node->get_parameter("dwell_seconds").as_double();
  if (!std::isfinite(dwell) || dwell < 0.0)
    throw std::runtime_error("dwell_seconds must be finite and nonnegative.");
  MoveGroup group(node, "cr5_group");
  group.setEndEffectorLink("probe_tcp");
  group.setPoseReferenceFrame(group.getRobotModel()->getModelFrame());
  auto state = group.getCurrentState(10.0);
  if (!state)
    throw std::runtime_error("No current robot state.");
  state->update();
  const Eigen::Isometry3d initial = state->getGlobalLinkTransform("probe_tcp");
  RCLCPP_INFO(node->get_logger(), "Scan origin in %s [m]: %.6f, %.6f, %.6f",
              group.getPoseReferenceFrame().c_str(), initial.translation().x(),
              initial.translation().y(), initial.translation().z());
  constexpr double radius = 0.05;
  constexpr double pi = 3.14159265358979323846;
  constexpr int station_count = 12;
  RCLCPP_INFO(node->get_logger(), "Scan direction viewed from +Z: %s",
              clockwise ? "clockwise" : "counterclockwise");
  // Contact-frame convention: TCP +X is the contact-face long edge,
  // TCP +Z is the outward contact normal, and TCP +Y is transverse.
  // Set the long edge radially outward and the contact direction downward.
  // The TCP is located at the nominal mesh contact face, not hardware-calibrated.
  const auto station_rotation = [](double theta) -> Eigen::Matrix3d {
    Eigen::Matrix3d rotation;
    const Eigen::Vector3d radial(-std::sin(theta), std::cos(theta), 0.0);
    const Eigen::Vector3d down(0.0, 0.0, -1.0);
    rotation.col(0) = radial;
    rotation.col(1) = down.cross(radial);
    rotation.col(2) = down;
    return rotation;
  };
  Eigen::Isometry3d first_station = initial;
  first_station.translation().y() += radius;
  first_station.linear() = station_rotation(0.0);
  struct Stage
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    std::string label;
    bool dwell_after;
  };
  std::vector<Stage> stages;
  // The initial setup may use joint-space planning to select a different
  // elbow/wrist branch. The scan itself remains Cartesian throughout.
  Eigen::Isometry3d aligned_origin = initial;
  aligned_origin.linear() = first_station.rotation();
  stages.push_back({{to_pose(aligned_origin)},
                    "Joint-space setup: align downward on selected IK branch", false});
  stages.push_back({{to_pose(first_station)}, "Move +Y 5 cm to first station", true});
  for (int station = 0; station < station_count; ++station)
  {
    // Counterclockwise in model XY, starting on the origin's +Y side.
    const double theta = direction * station * pi / 6.0;
    Eigen::Isometry3d top = initial;
    top.translation().x() -= radius * std::sin(theta);
    top.translation().y() += radius * std::cos(theta);
    top.linear() = station_rotation(theta);
    Eigen::Isometry3d bottom = top;
    bottom.translation().z() -= 0.05;
    const std::string prefix = "Station " + std::to_string(station + 1) + "/12: ";
    stages.push_back({{to_pose(bottom)}, prefix + "lower 5 cm", true});
    double previous = 0.0;
    for (double degrees : {10.0, -10.0, 20.0, -20.0, 0.0})
    {
      const double angle = degrees * pi / 180.0;
      const int steps = static_cast<int>(std::ceil(std::abs(angle - previous) / (pi / 180.0)));
      std::vector<geometry_msgs::msg::Pose> waypoints;
      for (int i = 1; i <= steps; ++i)
      {
        const double a = previous + (angle - previous) * i / steps;
        Eigen::Isometry3d pose = bottom;
        // Right multiplication rotates around this station's local transverse Y.
        // The swing axis therefore follows the probe around the hemisphere.
        pose.linear() = bottom.rotation() *
                        Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix();
        if (degrees == 0.0 && i == steps)
          pose = bottom;
        waypoints.push_back(to_pose(pose));
      }
      stages.push_back({waypoints, prefix + "local transverse Y swing " + std::to_string(degrees) + " deg", false});
      previous = angle;
    }
    stages.push_back({{to_pose(top)}, prefix + "raise 5 cm", false});
    // Follow an actual sampled circular arc, not a straight chord.
    // The last arc closes 360 degrees without repeating station one's task.
    std::vector<geometry_msgs::msg::Pose> arc;
    for (int i = 1; i <= 30; ++i)
    {
      const double a = theta + direction * i * pi / 180.0;
      Eigen::Isometry3d pose = initial;
      pose.translation().x() -= radius * std::sin(a);
      pose.translation().y() += radius * std::cos(a);
      pose.linear() = station_rotation(a);
      if (station == station_count - 1 && i == 30)
        pose = first_station;
      arc.push_back(to_pose(pose));
    }
    stages.push_back({arc, prefix + "advance 30 deg with radial orientation", false});
  }

  const int attempts = static_cast<int>(node->get_parameter("ik_attempts").as_int());
  if (attempts < 1 || attempts > 100)
    throw std::runtime_error("ik_attempts must be in [1, 100].");
  const moveit::core::RobotState original_state(*state);
  const auto* joint_group = group.getRobotModel()->getJointModelGroup("cr5_group");
  std::vector<MoveGroup::Plan> plans;
  bool complete = false;
  group.setPlanningTime(5.0);
  group.setMaxVelocityScalingFactor(0.1);
  group.setMaxAccelerationScalingFactor(0.1);
  for (int attempt = 0; attempt < attempts && rclcpp::ok(); ++attempt)
  {
    RCLCPP_INFO(node->get_logger(), "Trying initial IK branch %d/%d", attempt + 1, attempts);
    moveit::core::RobotState candidate(original_state);
    if (attempt > 0)
      candidate.setToRandomPositions(joint_group);
    if (!candidate.setFromIK(joint_group, aligned_origin, "probe_tcp", 0.2))
    {
      RCLCPP_WARN(node->get_logger(), "No alignment IK solution for this seed.");
      continue;
    }
    // Equivalent revolutions are chosen close to the real startup state.
    // Do this only before planning, never splice a 2*pi jump into a trajectory.
    for (const auto& name : joint_group->getVariableNames())
    {
      const auto& bounds = group.getRobotModel()->getVariableBounds(name);
      const double q = candidate.getVariablePosition(name);
      const double reference = original_state.getVariablePosition(name);
      double best = q;
      for (int turn = -2; turn <= 2; ++turn)
      {
        const double equivalent = q + turn * 2.0 * pi;
        if ((!bounds.position_bounded_ ||
             (equivalent >= bounds.min_position_ && equivalent <= bounds.max_position_)) &&
            std::abs(equivalent - reference) < std::abs(best - reference))
          best = equivalent;
      }
      candidate.setVariablePosition(name, best);
    }
    if (!candidate.satisfiesBounds(joint_group))
      continue;
    candidate.update();
    *state = candidate;
    plans.clear();
    plans.emplace_back();  // Filled with the collision-checked setup plan below.
    bool candidate_complete = true;
    for (std::size_t i = 1; i < stages.size(); ++i)
    {
      group.setStartState(*state);
      moveit_msgs::msg::RobotTrajectory message;
      const double fraction = group.computeCartesianPath(stages[i].waypoints, 0.001, 0.0, message, true);
      RCLCPP_INFO(node->get_logger(), "Plan %zu/%zu [%s]: %.2f%%", i + 1,
                  stages.size(), stages[i].label.c_str(), fraction * 100.0);
      if (!std::isfinite(fraction) || fraction < 0.999999 ||
          message.joint_trajectory.points.empty())
      {
        // Report the last valid IK state; a full wrist revolution may consume
        // the available range even when the Cartesian circle itself is small.
        moveit::core::RobotState last_valid(*state);
        if (!message.joint_trajectory.points.empty())
        {
          const auto& point = message.joint_trajectory.points.back();
          for (std::size_t j = 0; j < message.joint_trajectory.joint_names.size(); ++j)
            last_valid.setVariablePosition(message.joint_trajectory.joint_names[j],
                                           point.positions.at(j));
        }
        for (const auto& name : group.getRobotModel()->getJointModelGroup("cr5_group")->getVariableNames())
        {
          const auto& bounds = group.getRobotModel()->getVariableBounds(name);
          const double value = last_valid.getVariablePosition(name);
          if (bounds.position_bounded_)
            RCLCPP_ERROR(node->get_logger(),
                         "%s last valid: %.2f deg, limits [%.2f, %.2f], nearest margin %.2f deg",
                         name.c_str(), value * 180.0 / pi,
                         bounds.min_position_ * 180.0 / pi, bounds.max_position_ * 180.0 / pi,
                         std::min(value - bounds.min_position_, bounds.max_position_ - value) * 180.0 / pi);
        }
        if (attempt + 1 == attempts)
        {
          // Diagnostic only: never execute this path with collision checking off.
          moveit_msgs::msg::RobotTrajectory unchecked;
          const double unchecked_fraction = group.computeCartesianPath(
              stages[i].waypoints, 0.001, 0.0, unchecked, false);
          RCLCPP_ERROR(node->get_logger(),
                       "Diagnostic only, same segment without collision checks: %.2f%%. "
                       "A higher fraction implicates collision checking; similar failure suggests IK/limits.",
                       unchecked_fraction * 100.0);
        }
        candidate_complete = false;
        RCLCPP_WARN(node->get_logger(), "IK candidate %d rejected at %s; trying another initial branch.",
                    attempt + 1, stages[i].label.c_str());
        break;
      }
      robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), "cr5_group");
      trajectory.setRobotTrajectoryMsg(*state, message);
      trajectory_processing::IterativeParabolicTimeParameterization timing;
      if (!timing.computeTimeStamps(trajectory, 0.1, 0.1) ||
          !std::isfinite(trajectory.getDuration()) || trajectory.getDuration() <= 0.0)
        throw std::runtime_error("Invalid trajectory timing.");
      MoveGroup::Plan plan;
      trajectory.getRobotTrajectoryMsg(plan.trajectory_);
      plans.push_back(plan);
      *state = trajectory.getLastWayPoint();
    }
    if (!candidate_complete)
      continue;
    // Only plan the setup once this branch can complete the entire scan.
    // This setup path may translate the probe before reaching aligned_origin.
    group.setStartState(original_state);
    std::vector<double> joint_values;
    candidate.copyJointGroupPositions(joint_group, joint_values);
    if (!group.setJointValueTarget(joint_values) ||
        group.plan(plans.front()) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(node->get_logger(), "Setup motion to this branch could not be planned.");
      continue;
    }
    complete = true;
    RCLCPP_INFO(node->get_logger(), "Selected complete scan on IK candidate %d.", attempt + 1);
    break;
  }
  if (!complete)
    throw std::runtime_error(
        "No complete scan found across the tested IK branches. No motion executed. "
        "The requested path may require a different scan center or initial configuration.");
  RCLCPP_INFO(node->get_logger(),
              "All 12 stations planned in %s: radius 5 cm, depth 5 cm, dwell %.1f s, radial long edge, local Y swings.",
              group.getPoseReferenceFrame().c_str(), dwell);
  if (!execute)
  {
    RCLCPP_INFO(node->get_logger(), "Calculation only. Use execute:=true to move.");
    return 0;
  }
  for (std::size_t i = 0; i < plans.size(); ++i)
  {
    if (!rclcpp::ok())
      return 1;
    RCLCPP_INFO(node->get_logger(), "Executing %zu/%zu: %s", i + 1, plans.size(), stages[i].label.c_str());
    if (group.execute(plans[i]) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Execution failed; remaining stages cancelled.");
    if (stages[i].dwell_after)
    {
      RCLCPP_INFO(node->get_logger(), "Dwelling for %.1f seconds", dwell);
      const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(dwell);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < until)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  auto final_state = group.getCurrentState(10.0);
  if (!final_state)
    throw std::runtime_error("Motion finished but final state is unavailable.");
  final_state->update();
  const auto& final = final_state->getGlobalLinkTransform("probe_tcp");
  const double position_error = (final.translation() - first_station.translation()).norm();
  const double angle_error = Eigen::AngleAxisd(first_station.rotation().transpose() * final.rotation()).angle();
  RCLCPP_INFO(node->get_logger(), "Return to first circle station error: %.6f m, %.6f rad", position_error, angle_error);
  if (position_error > 0.001 || angle_error > 0.01)
    throw std::runtime_error("Return pose exceeds 1 mm / 0.01 rad tolerance.");
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "cr5_circular_scan",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!node->has_parameter("execute")) node->declare_parameter<bool>("execute", false);
  if (!node->has_parameter("ik_attempts")) node->declare_parameter<int>("ik_attempts", 12);
  if (!node->has_parameter("clockwise")) node->declare_parameter<bool>("clockwise", false);
  if (!node->has_parameter("dwell_seconds")) node->declare_parameter<double>("dwell_seconds", 2.0);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int result = 0;
  try { result = run_sequence(node); }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
    result = 1;
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
