#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// Observe state feedback only; this node never commands motion.
int run_trace(const rclcpp::Node::SharedPtr& node)
{
  const auto link = node->get_parameter("end_effector_link").as_string();
  moveit::planning_interface::MoveGroupInterface move_group(node, "cr5_group");
  const auto model = move_group.getRobotModel();
  if (link.empty() || !model->hasLinkModel(link))
    throw std::runtime_error("Unknown link: " + link);

  auto publisher = node->create_publisher<visualization_msgs::msg::Marker>(
      "/end_effector_trace", rclcpp::QoS(1).reliable().transient_local());
  visualization_msgs::msg::Marker trace;
  trace.header.frame_id = model->getModelFrame();
  trace.ns = "end_effector_trace";
  trace.id = 0;
  trace.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trace.action = visualization_msgs::msg::Marker::ADD;
  trace.pose.orientation.w = 1.0;
  trace.scale.x = 0.002;
  trace.color.r = 0.1f;
  trace.color.g = 1.0f;
  trace.color.b = 0.1f;
  trace.color.a = 1.0f;

  if (!move_group.getCurrentState(10.0))
  {
    if (!rclcpp::ok())
      return 0;
    RCLCPP_ERROR(node->get_logger(), "No robot state. Check /joint_states and use_sim_time.");
    return 1;
  }
  // Replace the previous run's marker, even before the first movement.
  publisher->publish(trace);
  RCLCPP_INFO(node->get_logger(), "Tracing %s in %s at up to 20 Hz. Ctrl+C to stop.",
              link.c_str(), trace.header.frame_id.c_str());

  rclcpp::WallRate rate(20.0);
  constexpr double minimum_distance = 0.0005;
  constexpr std::size_t maximum_points = 5000;
  auto next_publish = std::chrono::steady_clock::now();
  while (rclcpp::ok())
  {
    auto state = move_group.getCurrentState(1.0);
    if (!rclcpp::ok())
      break;
    if (!state)
    {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                           "No fresh robot state available.");
      // Do not connect positions across a state feedback interruption.
      trace.points.clear();
      publisher->publish(trace);
      rate.sleep();
      continue;
    }
    state->update();
    const auto& transform = state->getGlobalLinkTransform(link);
    geometry_msgs::msg::Point point;
    point.x = transform.translation().x();
    point.y = transform.translation().y();
    point.z = transform.translation().z();
    bool append = trace.points.empty();
    if (!append)
    {
      const auto& last = trace.points.back();
      const double dx = point.x - last.x;
      const double dy = point.y - last.y;
      const double dz = point.z - last.z;
      append = dx * dx + dy * dy + dz * dz >= minimum_distance * minimum_distance;
    }
    if (append)
    {
      trace.points.push_back(point);
      if (trace.points.size() > maximum_points)
        trace.points.erase(trace.points.begin());
    }
    const auto now = std::chrono::steady_clock::now();
    // Repeat while stationary as well, for RViz subscribers using volatile QoS.
    if (append || now >= next_publish)
    {
      publisher->publish(trace);
      next_publish = now + std::chrono::seconds(1);
    }
    rate.sleep();
  }
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "cr5_trace_end_effector",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  if (!node->has_parameter("end_effector_link"))
    node->declare_parameter<std::string>("end_effector_link", "probe_tcp");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  int result = 0;
  try
  {
    result = run_trace(node);
  }
  catch (const std::exception& exception)
  {
    if (rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "%s", exception.what());
      result = 1;
    }
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result;
}
