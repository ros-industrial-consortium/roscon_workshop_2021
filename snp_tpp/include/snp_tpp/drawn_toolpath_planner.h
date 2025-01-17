#pragma once

#include <vector>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <pcl/PolygonMesh.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace snp_tpp
{
class DrawnToolpathPlanner : public noether::ToolPathPlanner
{
public:
  DrawnToolpathPlanner(rclcpp::Node::SharedPtr node, std::vector<geometry_msgs::msg::PolygonStamped> boundaries);

  noether::ToolPaths plan(const pcl::PolygonMesh& mesh) const override final;

  void setPointSpacing(const double point_spacing);
  void setMinSegmentSize(const double min_segment_size);
  void setSearchRadius(const double search_radius);

protected:
  /** @brief Distance between waypoints on the same raster line (m) */
  double point_spacing_;
  /** @brief Minimum length of a segment to generate waypoints on (m) */
  double min_segment_size_;
  /** @brief Search radius around a point to calculate normal (m) */
  double search_radius_;

  std::vector<geometry_msgs::msg::PolygonStamped> boundaries_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace snp_tpp
