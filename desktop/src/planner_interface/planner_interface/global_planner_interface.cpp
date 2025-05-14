#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <mutex>
#include <condition_variable>

class AdoreGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent;
    path_received_ = false;

    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/bridged_path", 10,
      std::bind(&AdoreGlobalPlanner::pathCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "AdoreGlobalPlanner configured");
  }

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    path_received_ = false;

    RCLCPP_INFO(node_->get_logger(), "Waiting for path from ROS1...");
    bool got_path = cv_.wait_for(lock, std::chrono::seconds(15), [this] { return path_received_; });

    if (!got_path) {
      RCLCPP_WARN(node_->get_logger(), "Timeout waiting for ROS1 path. Returning empty path.");
      return nav_msgs::msg::Path();
    }

    RCLCPP_INFO(node_->get_logger(), "Received path from ROS1.");
    return received_path_;
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_path_ = *msg;
    path_received_ = true;
    cv_.notify_one();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  nav_msgs::msg::Path received_path_;
  bool path_received_;
  std::mutex mutex_;
  std::condition_variable cv_;
};


PLUGINLIB_EXPORT_CLASS(AdoreGlobalPlanner, nav2_core::GlobalPlanner)
