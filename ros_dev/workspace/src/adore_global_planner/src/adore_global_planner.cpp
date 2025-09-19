#include "adore_global_planner/adore_global_planner.hpp"
//#include "adore_global_planner.hpp"

namespace adore_global_planner
{

void Planner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string /*name*/,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Store the LifecycleNode weakly (for logging, clock, etc.)
  parent_node_ = parent;

  // (Optional) keep the costmap if you need it:
  costmap_ros_ = costmap_ros;
  if (costmap_ros_) {
    costmap_ = costmap_ros_->getCostmap();
  } else {
    costmap_ = nullptr;
  }

  // 1) Subscribe to the external ROS 1 Path (via rosbridge):
  if (auto node = parent_node_.lock()) {
    external_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "/external_path_from_ros1",
      rclcpp::QoS(10),
      std::bind(&Planner::externalPathCallback, this, std::placeholders::_1));
  }

  // 2) Create a publisher so Nav2 can see the Path on "/plan":
  if (auto node = parent_node_.lock()) {
    internal_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
      "/plan",
      rclcpp::QoS(10));

    RCLCPP_INFO(
      node->get_logger(),
      "[Planner] configured. Subscribed to /external_path_from_ros1, publishing on /plan");
  }
}

void Planner::cleanup()
{
  external_path_sub_.reset();
  internal_path_pub_.reset();

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    have_path_ = false;
    last_external_path_ = nav_msgs::msg::Path();
  }

  if (auto node = parent_node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[Planner] cleaned up");
  }
}

void Planner::activate()
{
  if (auto node = parent_node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[Planner] activated");
  }
}

void Planner::deactivate()
{
  if (auto node = parent_node_.lock()) {
    RCLCPP_INFO(node->get_logger(), "[Planner] deactivated");
  }
  // You could reset external_path_sub_ here if you want to stop callbacks while deactivated.
}

nav_msgs::msg::Path Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & /*start*/,
  const geometry_msgs::msg::PoseStamped & /*goal*/)
{
  std::lock_guard<std::mutex> lock(path_mutex_);

  if (have_path_) {
    if (auto node = parent_node_.lock()) {
      RCLCPP_DEBUG(
        node->get_logger(),
        "[Planner] createPlan(): returning stored external Path with %zu poses",
        last_external_path_.poses.size());
    }
    return last_external_path_;
  }

  // No external Path yet: return an “empty” Path so Nav2 doesn’t crash.
  nav_msgs::msg::Path empty_msg;
  if (auto node = parent_node_.lock()) {
    empty_msg.header.stamp = node->get_clock()->now();
  } else {
    empty_msg.header.stamp = rclcpp::Time(0);
  }
  empty_msg.header.frame_id = "map";

  if (auto node = parent_node_.lock()) {
    RCLCPP_WARN(
      node->get_logger(),
      "[Planner] createPlan(): no external Path received → returning empty path");
  }
  return empty_msg;
}

void Planner::externalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    last_external_path_ = *msg;
    have_path_ = true;
  }

  internal_path_pub_->publish(*msg);

  if (auto node = parent_node_.lock()) {
    RCLCPP_INFO(
      node->get_logger(),
      "[Planner] Received external Path (%zu poses). Republishing on /plan.",
      msg->poses.size());
  }
}

}  // namespace adore_global_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  adore_global_planner::Planner,
  nav2_core::GlobalPlanner)
