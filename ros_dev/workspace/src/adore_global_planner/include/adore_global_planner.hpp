#pragma once

#include <memory>
#include <mutex>

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace adore_global_planner
{

class Planner : public nav2_core::GlobalPlanner
{
public:
  Planner() = default;
  ~Planner() override = default;

  
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  // ===== Bridge-specific members: =====
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr external_path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr internal_path_pub_;

  std::mutex path_mutex_;
  nav_msgs::msg::Path last_external_path_;
  bool have_path_{false};

  void externalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
};

}  // namespace adore_global_planner
