#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <moveit/robot_trajectory/robot_trajectory.h>

// Sample the returned joint-space path for geometric validation, independently
// of how densely MoveIt stores the trajectory. Do not alter the execution plan.
// This checks linear joint interpolation, not controller-specific splines.
template <typename Visitor>
inline void visit_trajectory_states(
    const robot_trajectory::RobotTrajectory& trajectory, Visitor visitor,
    double maximum_joint_step = 0.005)
{
  if (!std::isfinite(maximum_joint_step) || maximum_joint_step <= 0.0)
    throw std::runtime_error("Invalid FK sampling step.");
  if (trajectory.getWayPointCount() == 0)
    return;
  moveit::core::RobotState previous(trajectory.getWayPoint(0));
  previous.update();
  visitor(previous);
  moveit::core::RobotState interpolated(previous);
  for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i)
  {
    const auto& next = trajectory.getWayPoint(i);
    double largest_delta = 0.0;
    for (std::size_t j = 0; j < previous.getRobotModel()->getVariableCount(); ++j)
    {
      const double delta = std::abs(next.getVariablePositions()[j] - previous.getVariablePositions()[j]);
      if (!std::isfinite(delta))
        throw std::runtime_error("Non-finite joint position in trajectory.");
      largest_delta = std::max(largest_delta, delta);
    }
    const double required_steps = std::ceil(largest_delta / maximum_joint_step);
    if (required_steps > 10000.0)
      throw std::runtime_error("Joint discontinuity too large for FK validation.");
    const int steps = std::max(1, static_cast<int>(required_steps));
    for (int sample = 1; sample <= steps; ++sample)
    {
      previous.interpolate(next, static_cast<double>(sample) / steps, interpolated);
      interpolated.update();
      visitor(interpolated);
    }
    previous = next;
  }
}

inline std::vector<Eigen::Isometry3d> sample_trajectory_fk(
    const robot_trajectory::RobotTrajectory& trajectory, const std::string& link,
    double maximum_joint_step = 0.005)
{
  std::vector<Eigen::Isometry3d> samples;
  visit_trajectory_states(trajectory, [&](const moveit::core::RobotState& state) {
    samples.push_back(state.getGlobalLinkTransform(link));
  }, maximum_joint_step);
  return samples;
}
