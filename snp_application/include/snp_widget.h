#pragma once

#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

namespace Ui
{
class SNPWidget;
}

namespace snp_application
{
class SNPWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SNPWidget(rclcpp::Node::SharedPtr rviz_node, QWidget* parent = nullptr);

private:
  void runTreeWithThread();

  rclcpp::Node::SharedPtr bt_node_;
  rclcpp::Node::SharedPtr tpp_node_;
  rclcpp::executors::SingleThreadedExecutor tpp_node_executor_;
  std::future<void> tpp_node_future_;
  Ui::SNPWidget* ui_;
  BT::Blackboard::Ptr board_;
  std::shared_ptr<BT::StatusChangeLogger> logger_;
};

}  // namespace snp_application
