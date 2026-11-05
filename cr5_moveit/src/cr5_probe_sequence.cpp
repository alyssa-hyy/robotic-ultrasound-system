#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <thread>
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
  Eigen::Isometry3d lowered = initial;
  lowered.translation().z() -= 0.05;

  // Pre-plan the entire sequence from consecutive state snapshots before motion.
  // Angles are absolute offsets from the initial orientation around model X.
  std::vector<std::vector<geometry_msgs::msg::Pose>> stages;
  stages.push_back({to_pose(lowered)});
  constexpr double pi = 3.14159265358979323846;
  double previous_angle = 0.0;
  for (const double degrees : {10.0, -10.0, 20.0, -20.0, 0.0})
  {
    const double angle = degrees * pi / 180.0;
    const int steps = static_cast<int>(std::ceil(std::abs(angle - previous_angle) /
                                               (pi / 180.0)));
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (int i = 1; i <= steps; ++i)
    {
      const double a = previous_angle + (angle - previous_angle) * i / steps;
      Eigen::Isometry3d target = lowered;
      // Left multiplication rotates around the reference frame X axis.
      target.linear() = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix() *
                        initial.rotation();
      waypoints.push_back(to_pose(target));
    }
    stages.push_back(waypoints);
    previous_angle = angle;
  }
  stages.push_back({to_pose(initial)});

  std::vector<MoveGroup::Plan> plans;
  for (std::size_t i = 0; i < stages.size(); ++i)
  {
    group.setStartState(*state);
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = group.computeCartesianPath(stages[i], 0.001, 0.0, message, true);
    RCLCPP_INFO(node->get_logger(), "Stage %zu/%zu: %.2f%%", i + 1,
                stages.size(), fraction * 100.0);
    if (!std::isfinite(fraction) || fraction < 0.999999 ||
        message.joint_trajectory.points.empty())
      throw std::runtime_error("Incomplete path. Sequence will not execute.");
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
  RCLCPP_INFO(node->get_logger(),
              "All stages planned in %s: down 5 cm, dwell %.1f s, X swings, restore, up 5 cm.",
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
    RCLCPP_INFO(node->get_logger(), "Executing stage %zu/%zu", i + 1, plans.size());
    if (group.execute(plans[i]) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Execution failed; remaining stages cancelled.");
    if (i == 0)
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
  const double position_error = (final.translation() - initial.translation()).norm();
  const double angle_error = Eigen::AngleAxisd(initial.rotation().transpose() * final.rotation()).angle();
  RCLCPP_INFO(node->get_logger(), "Return error: %.6f m, %.6f rad", position_error, angle_error);
  if (position_error > 0.001 || angle_error > 0.01)
    throw std::runtime_error("Return pose exceeds 1 mm / 0.01 rad tolerance.");
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "cr5_probe_sequence",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!node->has_parameter("execute")) node->declare_parameter<bool>("execute", false);
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
