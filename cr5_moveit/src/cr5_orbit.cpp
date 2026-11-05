#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <memory>
#include <thread>
#include <vector>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "trajectory_fk_sampling.hpp"

// 完成轨迹计算，并根据参数决定是否执行。
// 返回 0 表示成功，非 0 表示失败。
int run_orbit(const rclcpp::Node::SharedPtr& node)
{
  const auto logger = node->get_logger();

  // ---------- 1. 连接 MoveIt ----------
  moveit::planning_interface::MoveGroupInterface move_group(
      node, "cr5_group");

  // 使用之前已经确认的末端链接。
  move_group.setEndEffectorLink("probe_tcp");

  // 明确指定路径点的参考坐标系。
  move_group.setPoseReferenceFrame(move_group.getRobotModel()->getModelFrame());

  RCLCPP_INFO(
      logger, "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      logger, "Pose reference frame: %s",
      move_group.getPoseReferenceFrame().c_str());

  RCLCPP_INFO(
      logger, "End effector link: %s",
      move_group.getEndEffectorLink().c_str());

  // ---------- 2. 获取当前状态 ----------
  auto current_state = move_group.getCurrentState(10.0);

  if (!current_state)
  {
    RCLCPP_ERROR(logger, "Failed to obtain current robot state.");
    return 1;
  }

  // Use the CR5 robot model frame (dummy_link); distance is in meters.
  if (move_group.getPlanningFrame() != move_group.getRobotModel()->getModelFrame())
  {
    RCLCPP_ERROR(
        logger,
        "Planning and robot model frames must match.");
    return 1;
  }

  // 从同一份状态快照计算末端位姿，
  // 确保路径起点和起始关节状态一致。
  current_state->update();

  const auto& transform =
      current_state->getGlobalLinkTransform("probe_tcp");

  const Eigen::Quaterniond quaternion(transform.rotation());

  geometry_msgs::msg::Pose start_pose;

  start_pose.position.x = transform.translation().x();
  start_pose.position.y = transform.translation().y();
  start_pose.position.z = transform.translation().z();

  start_pose.orientation.x = quaternion.x();
  start_pose.orientation.y = quaternion.y();
  start_pose.orientation.z = quaternion.z();
  start_pose.orientation.w = quaternion.w();

  RCLCPP_INFO(
      logger,
      "Start [m]: x=%.4f, y=%.4f, z=%.4f",
      start_pose.position.x,
      start_pose.position.y,
      start_pose.position.z);

  // ---------- 3. 生成目标位姿 ----------
  const double radius = node->get_parameter("radius").as_double();
  const bool clockwise = node->get_parameter("clockwise").as_bool();
  const bool follow_radial = node->get_parameter("follow_radial").as_bool();
  const bool return_center = node->get_parameter("return_center").as_bool();
  const double speed = node->get_parameter("speed_scale").as_double();
  if (!std::isfinite(radius) || radius <= 0.0)
    throw std::runtime_error("radius must be finite and greater than zero.");
  if (!std::isfinite(speed) || speed <= 0.0 || speed > 1.0)
    throw std::runtime_error("speed_scale must be in (0, 1].");
  constexpr double pi = 3.14159265358979323846;
  constexpr int segments = 360;
  const double direction = clockwise ? -1.0 : 1.0;
  const auto radial_quaternion = [](double angle) -> Eigen::Quaterniond {
    Eigen::Matrix3d rotation;
    rotation.col(0) = Eigen::Vector3d(-std::sin(angle), std::cos(angle), 0.0);
    rotation.col(2) = Eigen::Vector3d(0.0, 0.0, -1.0);
    rotation.col(1) = rotation.col(2).cross(rotation.col(0));
    return Eigen::Quaterniond(rotation);
  };
  const auto set_orientation = [](geometry_msgs::msg::Pose& pose, const Eigen::Quaterniond& q) {
    pose.orientation.x = q.x(); pose.orientation.y = q.y();
    pose.orientation.z = q.z(); pose.orientation.w = q.w();
  };
  const Eigen::Quaterniond initial_q(transform.rotation());
  const Eigen::Quaterniond first_q = follow_radial ? radial_quaternion(0.0) : initial_q;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  // Explicitly sample angular changes as well as translation.
  const auto append_rotation = [&](const geometry_msgs::msg::Pose& position,
                                    const Eigen::Quaterniond& from, const Eigen::Quaterniond& to) {
    const double distance = from.angularDistance(to);
    if (distance < 1e-9) return;
    const int count = std::max(1, static_cast<int>(std::ceil(distance / (pi / 180.0))));
    for (int i = 1; i <= count; ++i)
    {
      auto pose = position;
      set_orientation(pose, from.slerp(static_cast<double>(i) / count, to));
      waypoints.push_back(pose);
    }
  };
  append_rotation(start_pose, initial_q, first_q);
  auto first_pose = start_pose;
  first_pose.position.y += radius;
  set_orientation(first_pose, first_q);
  waypoints.push_back(first_pose);
  for (int i = 1; i <= segments; ++i)
  {
    const double angle = direction * 2.0 * pi * i / segments;
    auto pose = start_pose;
    pose.position.x -= radius * std::sin(angle);
    pose.position.y += radius * std::cos(angle);
    set_orientation(pose, follow_radial ? radial_quaternion(angle) : initial_q);
    if (i == segments) pose = first_pose;
    waypoints.push_back(pose);
  }
  if (return_center)
  {
    append_rotation(first_pose, first_q, initial_q);
    waypoints.push_back(start_pose);  // Exact original position and orientation.
  }
  RCLCPP_INFO(logger,
              "Orbit in %s: center=(%.6f, %.6f, %.6f), radius=%.3f m, %s, "
              "radial=%s, return_center=%s, requested waypoints=%zu",
              move_group.getPoseReferenceFrame().c_str(), start_pose.position.x,
              start_pose.position.y, start_pose.position.z, radius,
              clockwise ? "clockwise" : "counterclockwise",
              follow_radial ? "true" : "false", return_center ? "true" : "false", waypoints.size());

  move_group.setStartState(*current_state);

  // ---------- 4. 计算笛卡尔路径 ----------
  moveit_msgs::msg::RobotTrajectory trajectory_msg;

  // 最大平移插值步长：1 mm。
  const double eef_step = 0.001;

  // Humble 此接口中，0.0 关闭相对关节跳变检查。
  // 本例仅用于假硬件入门练习。
  const double jump_threshold = 0.0;

  // true 表示启用碰撞检查。
  const double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory_msg,
      true);

  RCLCPP_INFO(
      logger,
      "Cartesian path completion: %.2f%%",
      fraction * 100.0);

  // 不执行部分完成的路径。
  if (!std::isfinite(fraction) ||
      fraction < 0.999999 ||
      trajectory_msg.joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(
        logger,
        "Cartesian path is incomplete. Will not execute. "
        "Use follow_radial:=false to compare with position-only orbit.");
    return 1;
  }

  // ---------- 5. 时间参数化 ----------
  robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), "cr5_group");

  trajectory.setRobotTrajectoryMsg(
      *current_state, trajectory_msg);
  // Imported joint values invalidate FK caches; const waypoint access does not update them.
  for (std::size_t waypoint = 0; waypoint < trajectory.getWayPointCount(); ++waypoint)
    trajectory.getWayPointPtr(waypoint)->update();

  // Independently verify the returned FK path actually winds once around
  // the circle. A success fraction alone does not measure geometric coverage.
  double swept_angle = 0.0;
  double previous_angle = 0.0;
  bool have_previous = false;
  std::size_t circle_samples = 0;
  const auto fk_samples = sample_trajectory_fk(trajectory, "probe_tcp");
  for (std::size_t i = 0; i < fk_samples.size(); ++i)
  {
    const auto& fk = fk_samples[i];
    const double dx = fk.translation().x() - start_pose.position.x;
    const double dy = fk.translation().y() - start_pose.position.y;
    const double r = std::hypot(dx, dy);
    if (std::abs(r - radius) > 0.001 ||
        std::abs(fk.translation().z() - start_pose.position.z) > 0.001)
      continue;
    const double angle = std::atan2(dy, dx);
    if (have_previous)
      swept_angle += std::atan2(std::sin(angle - previous_angle), std::cos(angle - previous_angle));
    previous_angle = angle;
    have_previous = true;
    ++circle_samples;
  }
  RCLCPP_INFO(logger, "FK circle coverage: %.2f deg, samples=%zu",
              swept_angle * 180.0 / pi, circle_samples);
  if (circle_samples < 12 || std::abs(swept_angle - direction * 2.0 * pi) > 0.05)
    throw std::runtime_error("Returned trajectory does not cover a full circle. Will not execute.");
  const auto& expected_pose = return_center ? start_pose : first_pose;
  const auto& end_fk = trajectory.getLastWayPoint().getGlobalLinkTransform("probe_tcp");
  const Eigen::Vector3d expected_position(expected_pose.position.x, expected_pose.position.y,
                                           expected_pose.position.z);
  const Eigen::Quaterniond expected_q(expected_pose.orientation.w, expected_pose.orientation.x,
                                     expected_pose.orientation.y, expected_pose.orientation.z);
  if ((end_fk.translation() - expected_position).norm() > 0.001 ||
      expected_q.angularDistance(Eigen::Quaterniond(end_fk.rotation())) > 0.01)
    throw std::runtime_error("Planned endpoint does not match the requested return pose.");

  trajectory_processing::IterativeParabolicTimeParameterization
      time_parameterization;

  const bool timing_ok =
      time_parameterization.computeTimeStamps(
          trajectory,
          speed,   // 关节速度上限缩放
          speed);  // 关节加速度上限缩放

  if (!timing_ok)
  {
    RCLCPP_ERROR(
        logger,
        "Time parameterization failed. Will not execute.");
    return 1;
  }

  trajectory.getRobotTrajectoryMsg(trajectory_msg);

  const auto& points = trajectory_msg.joint_trajectory.points;
  const auto& duration = points.back().time_from_start;

  const double duration_seconds =
      static_cast<double>(duration.sec) +
      static_cast<double>(duration.nanosec) * 1e-9;

  if (!std::isfinite(duration_seconds) || duration_seconds <= 0.0)
  {
    RCLCPP_ERROR(logger, "Invalid trajectory duration.");
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "Trajectory points: %zu, duration: %.3f s",
      points.size(),
      duration_seconds);

  // ---------- 6. 根据参数决定是否执行 ----------
  const bool execute_motion =
      node->get_parameter("execute").as_bool();

  if (!execute_motion)
  {
    RCLCPP_INFO(
        logger,
        "Calculation only. No motion was executed.");
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;

  RCLCPP_INFO(logger, "Executing Cartesian trajectory...");

  const auto execution_result = move_group.execute(plan);

  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "Execution failed.");
    return 1;
  }

  RCLCPP_INFO(logger, "Execution succeeded.");
  auto final_state = move_group.getCurrentState(10.0);
  if (!final_state)
    throw std::runtime_error("Execution finished, but final robot state is unavailable.");
  final_state->update();
  const auto& actual = final_state->getGlobalLinkTransform("probe_tcp");
  const double position_error = (actual.translation() - expected_position).norm();
  const double rotation_error = expected_q.angularDistance(Eigen::Quaterniond(actual.rotation()));
  RCLCPP_INFO(logger, "Final pose error: %.6f m, %.6f rad", position_error, rotation_error);
  if (position_error > 0.001 || rotation_error > 0.01)
    throw std::runtime_error("Final pose exceeds 1 mm / 0.01 rad tolerance.");
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "cr5_orbit",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  // 不通过 launch 传入时，也提供默认值。
  if (!node->has_parameter("follow_radial"))
    node->declare_parameter<bool>("follow_radial", true);
  if (!node->has_parameter("return_center"))
    node->declare_parameter<bool>("return_center", true);
  if (!node->has_parameter("speed_scale"))
    node->declare_parameter<double>("speed_scale", 0.1);
  if (!node->has_parameter("radius"))
    node->declare_parameter<double>("radius", 0.05);
  if (!node->has_parameter("clockwise"))
    node->declare_parameter<bool>("clockwise", false);
  if (!node->has_parameter("execute"))
  {
    node->declare_parameter<bool>("execute", false);
  }

  // 后台处理关节状态、action 响应等 ROS 消息。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  int result = 0;

  try
  {
    result = run_orbit(node);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Exception: %s",
        exception.what());

    result = 1;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return result;
}
