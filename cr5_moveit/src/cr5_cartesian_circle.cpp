#include <cmath>
#include <exception>
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

// 完成轨迹计算，并根据参数决定是否执行。
// 返回 0 表示成功，非 0 表示失败。
int run_cartesian_circle(const rclcpp::Node::SharedPtr& node)
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
  // Current pose is on the circumference, not at the circle center.
  // Circle center is 21 mm in model +X from the initial probe position.
  // Travel counterclockwise when viewed from +Z toward the XY plane.
  constexpr double radius = 0.025;
  constexpr double pi = 3.14159265358979323846;
  constexpr int segments = 180;
  const double center_x = start_pose.position.x + radius;
  const double center_y = start_pose.position.y;
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.reserve(segments);
  for (int i = 1; i <= segments; ++i)
  {
    const double angle = 2.0 * pi * static_cast<double>(i) / segments;
    auto pose = start_pose;  // Preserve initial Z and quaternion at every point.
    pose.position.x = center_x - radius * std::cos(angle);
    pose.position.y = center_y - radius * std::sin(angle);
    // Use the original pose exactly for closure, avoiding roundoff.
    if (i == segments)
      pose = start_pose;
    waypoints.push_back(pose);
  }
  RCLCPP_INFO(logger,
              "XY circle in %s: radius=0.025 m, center=(%.6f, %.6f, %.6f), %d segments",
              move_group.getPoseReferenceFrame().c_str(), center_x, center_y,
              start_pose.position.z, segments);

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
        "Cartesian path is incomplete. Will not execute.");
    return 1;
  }

  // ---------- 5. 时间参数化 ----------
  robot_trajectory::RobotTrajectory trajectory(
      move_group.getRobotModel(), "cr5_group");

  trajectory.setRobotTrajectoryMsg(
      *current_state, trajectory_msg);

  trajectory_processing::IterativeParabolicTimeParameterization
      time_parameterization;

  const bool timing_ok =
      time_parameterization.computeTimeStamps(
          trajectory,
          0.1,   // 关节速度上限缩放
          0.1);  // 关节加速度上限缩放

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
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "cr5_cartesian_circle",
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true));

  // 不通过 launch 传入时，也提供默认值。
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
    result = run_cartesian_circle(node);
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
