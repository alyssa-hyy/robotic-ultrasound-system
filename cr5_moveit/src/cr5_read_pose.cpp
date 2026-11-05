#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "cr5_read_pose",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Launch overrides are already declared; provide defaults for ros2 run too.
  if (!node->has_parameter("planning_group"))
    node->declare_parameter<std::string>("planning_group", "cr5_group");
  if (!node->has_parameter("end_effector_link"))
    node->declare_parameter<std::string>("end_effector_link", "probe_tcp");
  if (!node->has_parameter("state_timeout"))
    node->declare_parameter<double>("state_timeout", 10.0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int result = 0;

  try
  {
    const auto group = node->get_parameter("planning_group").as_string();
    const auto link = node->get_parameter("end_effector_link").as_string();
    const double timeout = node->get_parameter("state_timeout").as_double();
    if (!std::isfinite(timeout) || timeout <= 0.0)
      throw std::runtime_error("state_timeout must be finite and greater than zero.");

    moveit::planning_interface::MoveGroupInterface move_group(node, group);
    const auto model = move_group.getRobotModel();
    if (link.empty() || !model->hasLinkModel(link))
      throw std::runtime_error("Unknown end-effector link: " + link);

    RCLCPP_INFO(node->get_logger(), "Planning group: %s", group.c_str());
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
                move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", link.c_str());

    // Wait for joint states, then compute FK from this same state snapshot.
    auto state = move_group.getCurrentState(timeout);
    if (!state)
      throw std::runtime_error(
          "Failed to obtain current robot state. Check /joint_states and use_sim_time.");
    state->update();
    const auto& transform = state->getGlobalLinkTransform(link);
    const auto& position = transform.translation();
    const Eigen::Quaterniond orientation(transform.rotation());

    RCLCPP_INFO(node->get_logger(), "Pose frame: %s", model->getModelFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "Position [m]: x=%.6f, y=%.6f, z=%.6f",
                position.x(), position.y(), position.z());
    RCLCPP_INFO(node->get_logger(),
                "Orientation [quaternion]: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
                orientation.x(), orientation.y(), orientation.z(), orientation.w());
  }
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
