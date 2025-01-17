#include <snp_tpp/drawn_toolpath_planner_widget.h>
#include "ui_drawn_toolpath_planner_widget.h"

#include <noether_gui/utils.h>
#include <yaml-cpp/yaml.h>

namespace snp_tpp
{
DrawnToolpathPlannerWidget::DrawnToolpathPlannerWidget(QWidget* parent)
  : noether::ToolPathPlannerWidget(parent)
  , ui_(new Ui::DrawnToolpathPlanner())
  , node_(std::make_shared<rclcpp::Node>("roi_selection_mesh_modifier"))
  , client_(node_->create_client<rviz_polygon_selection_tool::srv::GetSelection>("get_selection"))
  , thread_(std::bind(&DrawnToolpathPlannerWidget::spin, this))
{
  ui_->setupUi(this);
}

DrawnToolpathPlannerWidget::~DrawnToolpathPlannerWidget()
{
  executor_.cancel();
  thread_.join();
}

void DrawnToolpathPlannerWidget::spin()
{
  executor_.add_node(node_);
  executor_.spin();
}

void DrawnToolpathPlannerWidget::configure(const YAML::Node& config)
{
  ui_->double_spin_box_point_spacing->setValue(noether::getEntry<double>(config, "point_spacing"));
  ui_->double_spin_box_min_seg_length->setValue(noether::getEntry<double>(config, "min_segment_length"));
  ui_->double_spin_box_normal_radius->setValue(noether::getEntry<double>(config, "normal_radius"));
}

void DrawnToolpathPlannerWidget::save(YAML::Node& config) const
{
  config["point_spacing"] = ui_->double_spin_box_point_spacing->value();
  config["min_segment_length"] = ui_->double_spin_box_min_seg_length->value();
  config["normal_radius"] = ui_->double_spin_box_normal_radius->value();
}

noether::ToolPathPlanner::ConstPtr DrawnToolpathPlannerWidget::create() const
{
  if (!client_->service_is_ready())
    throw std::runtime_error("Drawn toolpath selection service is not available");

  auto request = std::make_shared<rviz_polygon_selection_tool::srv::GetSelection::Request>();
  auto future = client_->async_send_request(request);
  switch (future.wait_for(std::chrono::seconds(3)))
  {
    case std::future_status::ready:
      break;
    case std::future_status::timeout:
      throw std::runtime_error("Drawn toolpath selection service call to '" + std::string(client_->get_service_name()) +
                               "' timed out");
    default:
      throw std::runtime_error("Drawn toolpath selection service call to '" + std::string(client_->get_service_name()) +
                               "' failed");
  }

  rviz_polygon_selection_tool::srv::GetSelection::Response::SharedPtr response = future.get();

  auto planner = std::make_unique<DrawnToolpathPlanner>(node_, response->selection);

  planner->setPointSpacing(ui_->double_spin_box_point_spacing->value());
  planner->setMinSegmentSize(ui_->double_spin_box_min_seg_length->value());
  planner->setSearchRadius(ui_->double_spin_box_normal_radius->value());

  return planner;
}

}  // namespace snp_tpp
