#pragma once

#include <snp_tpp/drawn_toolpath_planner.h>

#include <noether_gui/widgets.h>

#include <QWidget>
#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

namespace Ui
{
class DrawnToolpathPlanner;
}

namespace snp_tpp
{
class DrawnToolpathPlannerWidget : public noether::ToolPathPlannerWidget
{
  Q_OBJECT
public:
  DrawnToolpathPlannerWidget(QWidget* parent = nullptr);
  virtual ~DrawnToolpathPlannerWidget();

  noether::ToolPathPlanner::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  void spin();

  Ui::DrawnToolpathPlanner* ui_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rviz_polygon_selection_tool::srv::GetSelection>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

}  // namespace snp_tpp
