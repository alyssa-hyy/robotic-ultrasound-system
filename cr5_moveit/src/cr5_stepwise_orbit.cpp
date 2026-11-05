#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "trajectory_fk_sampling.hpp"

using MoveGroup = moveit::planning_interface::MoveGroupInterface;
constexpr double pi = 3.14159265358979323846;

geometry_msgs::msg::Pose pose_message(const Eigen::Isometry3d& target)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = target.translation().x();
  pose.position.y = target.translation().y();
  pose.position.z = target.translation().z();
  const Eigen::Quaterniond q(target.rotation());
  pose.orientation.x = q.x(); pose.orientation.y = q.y();
  pose.orientation.z = q.z(); pose.orientation.w = q.w();
  return pose;
}

Eigen::Matrix3d radial_rotation(double angle)
{
  // TCP X is the contact long edge; TCP Z is its outward normal.
  const double yaw = angle;
  Eigen::Matrix3d rotation;
  const Eigen::Vector3d x(-std::sin(yaw), std::cos(yaw), 0.0);
  const Eigen::Vector3d z(0.0, 0.0, -1.0);
  rotation.col(0) = x;
  rotation.col(1) = z.cross(x);
  rotation.col(2) = z;
  return rotation;
}

int run(const rclcpp::Node::SharedPtr& node)
{
  const auto logger = node->get_logger();
  const double radius = node->get_parameter("radius").as_double();
  const double speed = node->get_parameter("speed_scale").as_double();
  const bool clockwise = node->get_parameter("clockwise").as_bool();
  const bool allow_flip = node->get_parameter("allow_axis_flip").as_bool();
  const bool return_center = node->get_parameter("return_center").as_bool();
  const double dwell_seconds = node->get_parameter("dwell_seconds").as_double();
  if (!std::isfinite(dwell_seconds) || dwell_seconds < 0.0)
    throw std::runtime_error("dwell_seconds must be finite and nonnegative.");
  const double stage_limit = node->get_parameter("max_stage_joint_travel_deg").as_double() * pi / 180.0;
  const double arm_limit = node->get_parameter("max_arm_excursion_deg").as_double() * pi / 180.0;
  const double wrist_limit = node->get_parameter("max_wrist_stage_travel_deg").as_double() * pi / 180.0;
  const double floor_z = node->get_parameter("floor_z").as_double();
  if (!std::isfinite(radius) || radius <= 0.0 || !std::isfinite(speed) || speed <= 0.0 || speed > 1.0)
    throw std::runtime_error("radius must be positive; speed_scale must be in (0, 1].");
  if (!std::isfinite(stage_limit) || stage_limit <= 0.0 ||
      !std::isfinite(arm_limit) || arm_limit <= 0.0 ||
      !std::isfinite(wrist_limit) || wrist_limit <= 0.0 || !std::isfinite(floor_z))
    throw std::runtime_error("Joint movement limits must be positive and floor_z finite.");
  MoveGroup group(node, "cr5_group");
  group.setEndEffectorLink("probe_tcp");
  group.setPoseReferenceFrame(group.getRobotModel()->getModelFrame());
  group.setMaxVelocityScalingFactor(speed);
  group.setMaxAccelerationScalingFactor(speed);
  group.setPlanningTime(5.0);
  group.setNumPlanningAttempts(5);
  auto state = group.getCurrentState(10.0);
  if (!state) throw std::runtime_error("No current robot state.");
  state->update();
  const Eigen::Isometry3d initial = state->getGlobalLinkTransform("probe_tcp");
  const moveit::core::RobotState original_state(*state);
  // RViz's visual Grid is not a collision object. Add an actual ground slab.
  moveit::planning_interface::PlanningSceneInterface scene_interface;
  moveit_msgs::msg::CollisionObject floor;
  floor.header.frame_id = group.getPlanningFrame();
  floor.id = "cr5_scan_ground";
  floor.operation = moveit_msgs::msg::CollisionObject::ADD;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {6.0, 6.0, 0.2};
  geometry_msgs::msg::Pose floor_pose;
  floor_pose.orientation.w = 1.0;
  floor_pose.position.z = floor_z - 0.1;
  floor.primitives.push_back(box);
  floor.primitive_poses.push_back(floor_pose);
  if (!scene_interface.applyCollisionObject(floor))
    throw std::runtime_error("Could not add ground collision object.");
  auto scene_client = node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  if (!scene_client->wait_for_service(std::chrono::seconds(10)))
    throw std::runtime_error("Planning scene service unavailable.");
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  using Components = moveit_msgs::msg::PlanningSceneComponents;
  request->components.components = Components::SCENE_SETTINGS | Components::ROBOT_STATE |
      Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_GEOMETRY |
      Components::OCTOMAP | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX |
      Components::LINK_PADDING_AND_SCALING;
  auto future = scene_client->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready)
    throw std::runtime_error("Timed out reading planning scene.");
  planning_scene::PlanningScene collision_scene(group.getRobotModel());
  if (!collision_scene.setPlanningSceneMsg(future.get()->scene) ||
      !collision_scene.getWorld()->getObject(floor.id))
    throw std::runtime_error("Ground collision scene could not be verified.");
  if (collision_scene.isStateColliding(*state))
    throw std::runtime_error("Startup state is in collision, including the modeled ground.");
  RCLCPP_INFO(logger, "Ground top Z=%.3f m; stage joint travel limit=%.1f deg; arm excursion limit=%.1f deg",
              floor_z, stage_limit * 180.0 / pi, arm_limit * 180.0 / pi);
  const double direction = clockwise ? -1.0 : 1.0;
  const auto point_at = [&](double angle) -> Eigen::Vector3d {
    Eigen::Vector3d p = initial.translation();
    p.x() -= radius * std::sin(angle);
    p.y() += radius * std::cos(angle);
    return p;
  };
  struct PreparedPlan
  {
    MoveGroup::Plan plan;
    std::string label;
    int dwell_station = 0;
  };
  std::vector<PreparedPlan> plans;

  // Accept a stage only after its endpoint is verified against the target.
  const auto save_plan = [&](MoveGroup::Plan plan, const Eigen::Isometry3d& target,
                              const std::string& label) {
    if (plan.trajectory_.joint_trajectory.points.empty())
      throw std::runtime_error("Empty trajectory: " + label);
    robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), "cr5_group");
    trajectory.setRobotTrajectoryMsg(*state, plan.trajectory_);
    // Imported joint values invalidate FK caches; const waypoint access does not update them.
    for (std::size_t waypoint = 0; waypoint < trajectory.getWayPointCount(); ++waypoint)
      trajectory.getWayPointPtr(waypoint)->update();
    // TCP-only checks miss elbow flips and full-turn equivalent joint jumps.
    // Check the entire arm, including interpolated states, before saving a plan.
    const auto& names = group.getRobotModel()->getJointModelGroup("cr5_group")->getVariableNames();
    std::vector<double> previous, travel(names.size(), 0.0);
    for (const auto& name : names) previous.push_back(state->getVariablePosition(name));
    visit_trajectory_states(trajectory, [&](const moveit::core::RobotState& sample) {
      if (!sample.satisfiesBounds())
        throw std::runtime_error("Joint bounds violated: " + label);
      for (std::size_t j = 0; j < names.size(); ++j)
      {
        const double value = sample.getVariablePosition(names[j]);
        travel[j] += std::abs(value - previous[j]);
        previous[j] = value;
        if (travel[j] > (names[j] == "joint6" ? wrist_limit : stage_limit))
          throw std::runtime_error(names[j] + " stage travel exceeds configured limit: " + label);
        if (names[j] != "joint6" &&
            std::abs(value - original_state.getVariablePosition(names[j])) > arm_limit)
          throw std::runtime_error(names[j] + " departs too far from startup arm posture: " + label);
      }
      if (collision_scene.isStateColliding(sample))
        throw std::runtime_error("Interpolated robot state collides with robot/environment/ground: " + label);
    });
    for (std::size_t j = 0; j < names.size(); ++j)
      RCLCPP_INFO(logger, "%s: %s total joint travel %.2f deg", label.c_str(), names[j].c_str(),
                  travel[j] * 180.0 / pi);
    const auto& end = trajectory.getLastWayPoint().getGlobalLinkTransform("probe_tcp");
    if ((end.translation() - target.translation()).norm() > 0.001 ||
        Eigen::Quaterniond(end.rotation()).angularDistance(Eigen::Quaterniond(target.rotation())) > 0.01)
      throw std::runtime_error("Endpoint mismatch: " + label);
    *state = trajectory.getLastWayPoint();
    state->update();
    plans.push_back({plan, label});
    RCLCPP_INFO(logger, "Prepared stage %zu: %s", plans.size(), label.c_str());
  };
  const auto cartesian = [&](const std::vector<geometry_msgs::msg::Pose>& waypoints,
                              const Eigen::Isometry3d& target, const std::string& label,
                              double expected_sweep, bool rotation_only) {
    group.clearPoseTargets();
    group.setStartState(*state);
    moveit_msgs::msg::RobotTrajectory message;
    const double fraction = group.computeCartesianPath(waypoints, 0.001, 0.0, message, true);
    RCLCPP_INFO(logger, "%s Cartesian completion: %.2f%%", label.c_str(), fraction * 100.0);
    if (!std::isfinite(fraction) || fraction < 0.999999 || message.joint_trajectory.points.empty())
      throw std::runtime_error("Incomplete translation path: " + label);
    // Choose equivalent revolute positions nearest the preceding state. This
    // removes +/-2*pi representation jumps without changing waypoint FK.
    std::vector<double> preceding;
    for (const auto& name : message.joint_trajectory.joint_names)
      preceding.push_back(state->getVariablePosition(name));
    std::size_t corrected = 0;
    for (auto& point : message.joint_trajectory.points)
    {
      if (point.positions.size() != preceding.size())
        throw std::runtime_error("Invalid trajectory joint dimensions.");
      for (std::size_t j = 0; j < preceding.size(); ++j)
      {
        const auto& name = message.joint_trajectory.joint_names[j];
        const double raw = point.positions[j];
        if (!std::isfinite(raw)) throw std::runtime_error("Non-finite trajectory joint value.");
        const auto* joint = group.getRobotModel()->getJointModel(name);
        double closest = raw;
        if (joint && joint->getType() == moveit::core::JointModel::REVOLUTE)
        {
          const auto& bounds = group.getRobotModel()->getVariableBounds(name);
          const double turns = std::round((preceding[j] - raw) / (2.0 * pi));
          for (int offset = -1; offset <= 1; ++offset)
          {
            const double equivalent = raw + (turns + offset) * 2.0 * pi;
            if ((!bounds.position_bounded_ ||
                 (equivalent >= bounds.min_position_ && equivalent <= bounds.max_position_)) &&
                std::abs(equivalent - preceding[j]) < std::abs(closest - preceding[j]))
              closest = equivalent;
          }
        }
        if (std::abs(closest - raw) > 1e-8) ++corrected;
        point.positions[j] = closest;
        preceding[j] = closest;
      }
    }
    if (corrected)
      RCLCPP_WARN(logger, "%s: corrected %zu equivalent full-turn joint representations before retiming",
                  label.c_str(), corrected);
    robot_trajectory::RobotTrajectory trajectory(group.getRobotModel(), "cr5_group");
    trajectory.setRobotTrajectoryMsg(*state, message);
    // Imported joint values invalidate FK caches; const waypoint access does not update them.
    for (std::size_t waypoint = 0; waypoint < trajectory.getWayPointCount(); ++waypoint)
      trajectory.getWayPointPtr(waypoint)->update();
    if (rotation_only)
    {
      const Eigen::Isometry3d rotation_start = state->getGlobalLinkTransform("probe_tcp");
      const bool yaw_only = rotation_start.rotation().col(2).dot(target.rotation().col(2)) > 1.0 - 1e-6;
      visit_trajectory_states(trajectory, [&](const moveit::core::RobotState& sample) {
        if ((sample.getGlobalLinkTransform("probe_tcp").translation() - target.translation()).norm() > 0.001)
          throw std::runtime_error("Contact center moves during rotation: " + label);
        if (yaw_only)
        {
          for (int joint = 1; joint <= 5; ++joint)
          {
            const auto name = "joint" + std::to_string(joint);
            if (std::abs(sample.getVariablePosition(name) - state->getVariablePosition(name)) > 2.0 * pi / 180.0)
              throw std::runtime_error("Pure TCP yaw changed proximal arm joint by over 2 degrees: " + name + ": " + label);
          }
        }
      });
    }
    if (expected_sweep != 0.0)
    {
      const auto& start = state->getGlobalLinkTransform("probe_tcp");
      double previous_angle = std::atan2(start.translation().y() - initial.translation().y(),
                                         start.translation().x() - initial.translation().x());
      double sweep = 0.0;
      double maximum_increment = 0.0;
      const auto fk_samples = sample_trajectory_fk(trajectory, "probe_tcp");
      for (std::size_t i = 0; i < fk_samples.size(); ++i)
      {
        const auto& fk = fk_samples[i];
        const double dx = fk.translation().x() - initial.translation().x();
        const double dy = fk.translation().y() - initial.translation().y();
        if (std::abs(std::hypot(dx, dy) - radius) > 0.001 ||
            std::abs(fk.translation().z() - initial.translation().z()) > 0.001 ||
            Eigen::Quaterniond(fk.rotation()).angularDistance(Eigen::Quaterniond(start.rotation())) > 0.01)
          throw std::runtime_error("Transfer geometry/orientation mismatch: " + label);
        const double angle = std::atan2(dy, dx);
        const double increment = std::atan2(std::sin(angle - previous_angle),
                                            std::cos(angle - previous_angle));
        maximum_increment = std::max(maximum_increment, std::abs(increment));
        if (std::abs(increment) > 0.1)
        {
          RCLCPP_ERROR(logger,
                       "%s: point %zu/%zu, angular gap %.3f deg, previous angle %.3f deg, angle %.3f deg",
                       label.c_str(), i + 1, fk_samples.size(),
                       increment * 180.0 / pi, previous_angle * 180.0 / pi, angle * 180.0 / pi);
          throw std::runtime_error("Transfer sampling too sparse to verify circle: " + label);
        }
        sweep += increment;
        previous_angle = angle;
      }
      RCLCPP_INFO(logger, "%s verified sweep: %.2f deg, trajectory points=%zu, FK samples=%zu, max angular gap=%.3f deg",
                  label.c_str(), sweep * 180.0 / pi, trajectory.getWayPointCount(),
                  fk_samples.size(), maximum_increment * 180.0 / pi);
      if (std::abs(sweep - expected_sweep) > 0.02)
        throw std::runtime_error("Transfer arc coverage mismatch: " + label);
    }
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, speed, speed) || trajectory.getDuration() <= 0.0 ||
        !std::isfinite(trajectory.getDuration()))
      throw std::runtime_error("Invalid timing: " + label);
    MoveGroup::Plan plan;
    trajectory.getRobotTrajectoryMsg(plan.trajectory_);
    save_plan(plan, target, label);
  };
  const auto reorient = [&](Eigen::Isometry3d target, const std::string& label,
                              bool radial_goal) {
    const Eigen::Isometry3d actual = state->getGlobalLinkTransform("probe_tcp");
    // For an undirected long edge, choose the closest equivalent pose rather
    // than forcing a 180-degree reversal at an arbitrary +/-90 yaw boundary.
    if (radial_goal && allow_flip)
    {
      Eigen::Isometry3d flipped = target;
      flipped.linear() = target.rotation() *
          Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      const Eigen::Quaterniond current_q(actual.rotation());
      if (current_q.angularDistance(Eigen::Quaterniond(flipped.rotation())) <
          current_q.angularDistance(Eigen::Quaterniond(target.rotation())))
        target = flipped;
    }
    if ((actual.translation() - target.translation()).norm() < 1e-6 &&
        Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(target.rotation())) < 1e-6)
      return;

    const bool pure_yaw = actual.rotation().col(2).dot(target.rotation().col(2)) > 1.0 - 1e-6;
    if (pure_yaw)
    {
      // In this CR5 mounting, the contact center lies on the joint6 axis and
      // TCP Z is parallel to it. Do not ask full-arm IK to perform wrist yaw.
      // Verify the equivalence with FK before accepting this model assumption.
      const double q6_start = state->getVariablePosition("joint6");
      const auto& bounds = group.getRobotModel()->getVariableBounds("joint6");
      double best_delta = 0.0;
      bool found = false;
      Eigen::Isometry3d selected = target;
      for (int flip = 0; flip < ((radial_goal && allow_flip) ? 2 : 1); ++flip)
      {
        Eigen::Isometry3d candidate = target;
        if (flip)
          candidate.linear() = target.rotation() *
              Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const Eigen::Matrix3d relative = actual.rotation().transpose() * candidate.rotation();
        const double principal = std::atan2(relative(1, 0), relative(0, 0));
        for (int turn = -1; turn <= 1; ++turn)
        {
          const double delta = principal + turn * 2.0 * pi;
          const double q6_goal = q6_start + delta;
          if (std::abs(delta) > wrist_limit ||
              (bounds.position_bounded_ &&
               (q6_goal < bounds.min_position_ || q6_goal > bounds.max_position_)))
            continue;
          moveit::core::RobotState check(*state);
          check.setVariablePosition("joint6", q6_goal);
          check.update();
          const auto& fk = check.getGlobalLinkTransform("probe_tcp");
          if ((fk.translation() - candidate.translation()).norm() > 0.001 ||
              Eigen::Quaterniond(fk.rotation()).angularDistance(Eigen::Quaterniond(candidate.rotation())) > 0.01)
            continue;
          if (!found || std::abs(delta) < std::abs(best_delta))
          {
            found = true; best_delta = delta; selected = candidate;
          }
        }
      }
      if (!found)
        throw std::runtime_error("No bounded joint6-only yaw reaches the requested contact pose: " + label);
      target = selected;
      if (std::abs(best_delta) < 1e-8)
        return;
      RCLCPP_INFO(logger, "%s: joint6-only yaw %.2f deg, %.2f -> %.2f deg; joints1-5 fixed",
                  label.c_str(), best_delta * 180.0 / pi, q6_start * 180.0 / pi,
                  (q6_start + best_delta) * 180.0 / pi);
      robot_trajectory::RobotTrajectory wrist_trajectory(group.getRobotModel(), "cr5_group");
      const int count = std::max(1, static_cast<int>(std::ceil(std::abs(best_delta) / (pi / 180.0))));
      for (int i = 0; i <= count; ++i)
      {
        moveit::core::RobotState sample(*state);
        sample.setVariablePosition("joint6", q6_start + best_delta * i / count);
        sample.update();
        if ((sample.getGlobalLinkTransform("probe_tcp").translation() - target.translation()).norm() > 0.001)
          throw std::runtime_error("Joint6 rotation does not preserve contact center: " + label);
        wrist_trajectory.addSuffixWayPoint(sample, 0.0);
      }
      trajectory_processing::IterativeParabolicTimeParameterization timing;
      if (!timing.computeTimeStamps(wrist_trajectory, speed, speed) ||
          !std::isfinite(wrist_trajectory.getDuration()) || wrist_trajectory.getDuration() <= 0.0)
        throw std::runtime_error("Invalid wrist-only trajectory timing.");
      MoveGroup::Plan plan;
      wrist_trajectory.getRobotTrajectoryMsg(plan.trajectory_);
      // The shared guard verifies interpolated full-robot collisions and bounds.
      save_plan(plan, target, label + " (joint6-only TCP yaw)");
    }
    else
    {
      // Initial downward alignment/restoration may require changing the tilt.
      // Only these non-yaw stages still use Cartesian IK.
      // After downward alignment, this is rotation about TCP Z (flange Z),
      // not rotation about the mesh/handle long axis.
      const Eigen::Quaterniond from(actual.rotation()), to(target.rotation());
      const int steps = std::max(1, static_cast<int>(std::ceil(from.angularDistance(to) / (pi / 180.0))));
      std::vector<geometry_msgs::msg::Pose> rotations;
      for (int i = 1; i <= steps; ++i)
      {
        Eigen::Isometry3d pose = target;
        pose.linear() = from.slerp(static_cast<double>(i) / steps, to).toRotationMatrix();
        rotations.push_back(pose_message(pose));
      }
      // A failed local rotation must stop the scan. Never replace it with a
      // free arm reconfiguration that could swing the shoulder through the floor.
      cartesian(rotations, target, label + " (contact-center rotation)", 0.0, true);
    }
    if (radial_goal)
    {
      const auto& achieved = state->getGlobalLinkTransform("probe_tcp");
      Eigen::Vector3d radial = target.translation() - initial.translation();
      radial.z() = 0.0;
      radial.normalize();
      double alignment = achieved.rotation().col(0).dot(radial);
      if (allow_flip) alignment = std::abs(alignment);
      const double radial_error = std::acos(std::max(-1.0, std::min(1.0, alignment)));
      const double normal_error = std::acos(std::max(-1.0, std::min(1.0,
          achieved.rotation().col(2).dot(Eigen::Vector3d(0.0, 0.0, -1.0)))));
      RCLCPP_INFO(logger, "%s: contact long-edge radial error %.3f deg, downward-normal error %.3f deg",
                  label.c_str(), radial_error * 180.0 / pi, normal_error * 180.0 / pi);
      if (radial_error > 0.01 || normal_error > 0.01)
        throw std::runtime_error("Contact-frame radial alignment check failed: " + label);
    }
  };

  RCLCPP_INFO(logger, "Stepwise orbit in %s: center=(%.6f, %.6f, %.6f), radius=%.3f, axis_flip=%s",
              group.getPoseReferenceFrame().c_str(), initial.translation().x(),
              initial.translation().y(), initial.translation().z(), radius, allow_flip ? "true" : "false");
  Eigen::Isometry3d first = initial;
  first.translation() = point_at(0.0);
  cartesian({pose_message(first)}, first, "Approach first circle point with current orientation", 0.0, false);
  first.linear() = radial_rotation(0.0);
  reorient(first, "Station 1/12: adjust radial orientation", true);
  plans.back().dwell_station = 1;
  first.linear() = state->getGlobalLinkTransform("probe_tcp").rotation();
  for (int station = 0; station < 12; ++station)
  {
    const double start_angle = direction * station * pi / 6.0;
    // Freeze the actually planned orientation throughout this transfer arc.
    const Eigen::Matrix3d held_rotation = state->getGlobalLinkTransform("probe_tcp").rotation();
    std::vector<geometry_msgs::msg::Pose> arc;
    Eigen::Isometry3d target = initial;
    target.linear() = held_rotation;
    for (int sample = 1; sample <= 30; ++sample)
    {
      const double angle = start_angle + direction * sample * pi / 180.0;
      target.translation() = point_at(angle);
      if (station == 11 && sample == 30) target.translation() = first.translation();
      arc.push_back(pose_message(target));
    }
    cartesian(arc, target, "Transfer arc " + std::to_string(station + 1) + "/12, fixed orientation",
              direction * pi / 6.0, false);
    target.linear() = radial_rotation(direction * (station + 1) * pi / 6.0);
    if (station == 11) target = first;
    reorient(target, station == 11 ? "Close circle: restore first radial orientation" :
             "Station " + std::to_string(station + 2) + "/12: adjust radial orientation", station != 11);
    if (station < 11)
      plans.back().dwell_station = station + 2;
  }
  if (return_center)
  {
    Eigen::Isometry3d restore = initial;
    restore.translation() = first.translation();
    reorient(restore, "Restore startup orientation before returning to center", false);
    cartesian({pose_message(initial)}, initial, "Return to startup center", 0.0, false);
  }
  RCLCPP_INFO(logger, "All %zu stages prepared. 12 circle arcs total 360 degrees.", plans.size());
  if (!node->get_parameter("execute").as_bool())
  {
    RCLCPP_INFO(logger, "Planning only. Use execute:=true to execute.");
    return 0;
  }
  for (std::size_t i = 0; i < plans.size(); ++i)
  {
    if (!rclcpp::ok()) return 1;
    RCLCPP_INFO(logger, "Executing %zu/%zu: %s", i + 1, plans.size(), plans[i].label.c_str());
    if (group.execute(plans[i].plan) != moveit::core::MoveItErrorCode::SUCCESS)
      throw std::runtime_error("Execution failed; remaining stages cancelled.");
    if (plans[i].dwell_station != 0 && dwell_seconds > 0.0)
    {
      RCLCPP_INFO(logger, "Station %d/12: holding radial pose for %.2f seconds",
                  plans[i].dwell_station, dwell_seconds);
      const auto until = std::chrono::steady_clock::now() +
                         std::chrono::duration<double>(dwell_seconds);
      while (rclcpp::ok() && std::chrono::steady_clock::now() < until)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  auto final_state = group.getCurrentState(10.0);
  if (!final_state) throw std::runtime_error("Final state unavailable.");
  final_state->update();
  const auto& actual = final_state->getGlobalLinkTransform("probe_tcp");
  const Eigen::Isometry3d expected = return_center ? initial : first;
  const double position_error = (actual.translation() - expected.translation()).norm();
  const double angle_error = Eigen::Quaterniond(actual.rotation()).angularDistance(Eigen::Quaterniond(expected.rotation()));
  RCLCPP_INFO(logger, "Final pose error: %.6f m, %.6f rad", position_error, angle_error);
  if (position_error > 0.001 || angle_error > 0.01) throw std::runtime_error("Final pose tolerance exceeded.");
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cr5_stepwise_orbit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!node->has_parameter("execute")) node->declare_parameter<bool>("execute", false);
  if (!node->has_parameter("radius")) node->declare_parameter<double>("radius", 0.05);
  if (!node->has_parameter("speed_scale")) node->declare_parameter<double>("speed_scale", 0.1);
  if (!node->has_parameter("clockwise")) node->declare_parameter<bool>("clockwise", false);
  if (!node->has_parameter("allow_axis_flip")) node->declare_parameter<bool>("allow_axis_flip", true);
  if (!node->has_parameter("return_center")) node->declare_parameter<bool>("return_center", true);
  if (!node->has_parameter("dwell_seconds")) node->declare_parameter<double>("dwell_seconds", 2.0);
  if (!node->has_parameter("max_stage_joint_travel_deg"))
    node->declare_parameter<double>("max_stage_joint_travel_deg", 90.0);
  if (!node->has_parameter("max_arm_excursion_deg"))
    node->declare_parameter<double>("max_arm_excursion_deg", 60.0);
  if (!node->has_parameter("max_wrist_stage_travel_deg"))
    node->declare_parameter<double>("max_wrist_stage_travel_deg", 180.0);
  if (!node->has_parameter("floor_z")) node->declare_parameter<double>("floor_z", -0.01);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int result = 0;
  try { result = run(node); }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", exception.what()); result = 1;
  }
  executor.cancel(); spinner.join(); rclcpp::shutdown();
  return result;
}
