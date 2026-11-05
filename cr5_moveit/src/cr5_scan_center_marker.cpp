#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int run(const rclcpp::Node::SharedPtr& node)
{
  const double radius = node->get_parameter("radius").as_double();
  if (!std::isfinite(radius) || radius <= 0.0)
    throw std::runtime_error("radius must be finite and positive.");
  moveit::planning_interface::MoveGroupInterface group(node, "cr5_group");
  auto state = group.getCurrentState(10.0);
  if (!state) throw std::runtime_error("No current robot state.");
  state->update();
  // Copy once. Never follow the probe after it starts moving.
  const Eigen::Vector3d center = state->getGlobalLinkTransform("probe_tcp").translation();
  const auto frame = group.getRobotModel()->getModelFrame();
  auto publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/cr5_scan_center", rclcpp::QoS(1).reliable().transient_local());
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker dot;
  dot.header.frame_id = frame;
  dot.ns = "scan_center";
  dot.id = 0;
  dot.type = visualization_msgs::msg::Marker::SPHERE;
  dot.action = visualization_msgs::msg::Marker::ADD;
  dot.pose.orientation.w = 1.0;
  dot.pose.position.x = center.x();
  dot.pose.position.y = center.y();
  dot.pose.position.z = center.z();
  dot.scale.x = dot.scale.y = dot.scale.z = 0.008;
  dot.color.r = 1.0f; dot.color.g = 0.15f; dot.color.b = 0.05f; dot.color.a = 1.0f;
  markers.markers.push_back(dot);
  auto text = dot;
  text.id = 1;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.pose.position.z += 0.03;
  text.scale.z = 0.018;
  text.color.r = text.color.g = text.color.b = 1.0f;
  text.text = "Scan center";
  markers.markers.push_back(text);
  auto circle = dot;
  circle.id = 2;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.pose.position.x = circle.pose.position.y = circle.pose.position.z = 0.0;
  circle.scale.x = 0.0015;
  circle.color.r = 1.0f; circle.color.g = 0.75f; circle.color.b = 0.0f; circle.color.a = 0.8f;
  circle.action = node->get_parameter("show_circle").as_bool() ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
  constexpr double pi = 3.14159265358979323846;
  for (int i = 0; i <= 180; ++i)
  {
    const double a = 2.0 * pi * i / 180.0;
    geometry_msgs::msg::Point p;
    p.x = center.x() + radius * std::cos(a);
    p.y = center.y() + radius * std::sin(a);
    p.z = center.z();
    circle.points.push_back(p);
  }
  markers.markers.push_back(circle);
  RCLCPP_INFO(node->get_logger(), "Fixed scan center in %s: x=%.6f y=%.6f z=%.6f m; radius=%.3f m",
              frame.c_str(), center.x(), center.y(), center.z(), radius);
  RCLCPP_INFO(node->get_logger(), "Start orbit from this same robot pose. Ctrl+C stops marker publishing.");
  rclcpp::WallRate rate(2.0);
  while (rclcpp::ok())
  {
    publisher->publish(markers);
    rate.sleep();
  }
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cr5_scan_center_marker",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!node->has_parameter("radius")) node->declare_parameter<double>("radius", 0.05);
  if (!node->has_parameter("show_circle")) node->declare_parameter<bool>("show_circle", true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int result = 0;
  try { result = run(node); }
  catch (const std::exception& exception)
  {
    if (rclcpp::ok()) { RCLCPP_ERROR(node->get_logger(), "%s", exception.what()); result = 1; }
  }
  executor.cancel(); spinner.join(); rclcpp::shutdown();
  return result;
}
