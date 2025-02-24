#include <snp_application/snp_widget.h>
#include "ui_snp_widget.h"
// BT
#include <snp_application/bt/bt_thread.h>
#include <snp_application/bt/text_edit_logger.h>
#include <snp_application/bt/utils.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/plugins.hpp>
#include <boost_plugin_loader/plugin_loader.h>
#include <QMessageBox>
#include <QTextStream>
#include <QScrollBar>
#include <QTextEdit>
#include <QStackedWidget>
#include <sensor_msgs/msg/joint_state.hpp>
#include <snp_tpp/tpp_widget.h>
#include <trajectory_preview/trajectory_preview_widget.h>

static const char* BT_FILES_PARAM = "bt_files";
static const char* BT_PLUGIN_LIBS_PARAM = "bt_plugin_libs";
static const char* BT_ROS_PLUGIN_LIBS_PARAM = "bt_ros_plugin_libs";
static const char* BT_PARAM = "tree";
static const char* BT_FREESPACE_PARAM = "freespace_tree";
static const char* BT_TIMEOUT_PARAM = "bt_timeout";
static const char* FOLLOW_JOINT_TRAJECTORY_ACTION = "follow_joint_trajectory_action";
static const char* HOME_STATE_NAME = "home_state";

class TPPDialog : public QDialog
{
public:
  TPPDialog(rclcpp::Node::SharedPtr node, QWidget* parent = nullptr) : QDialog(parent)
  {
    setWindowTitle("Tool Path Planner");

    // Set non-modal, so it can launch the load and save dialogs within itself
    setModal(false);

    boost_plugin_loader::PluginLoader loader;
    loader.search_libraries.insert(NOETHER_GUI_PLUGINS);
    loader.search_libraries.insert(SNP_TPP_GUI_PLUGINS);
    loader.search_libraries_env = NOETHER_GUI_PLUGIN_LIBS_ENV;
    loader.search_paths_env = NOETHER_GUI_PLUGIN_PATHS_ENV;

    auto* widget = new snp_tpp::TPPWidget(node, std::move(loader), this);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(widget);
  }
};

namespace snp_application
{
SNPWidget::SNPWidget(rclcpp::Node::SharedPtr rviz_node, QWidget* parent)
  : QWidget(parent)
  , bt_node_(std::make_shared<rclcpp::Node>("snp_application_bt"))
  , tpp_node_(std::make_shared<rclcpp::Node>("snp_application_tpp"))
  , ui_(new Ui::SNPWidget())
  , board_(BT::Blackboard::create())
{
  ui_->setupUi(this);
  ui_->group_box_operation->setEnabled(false);
  ui_->push_button_reset->setEnabled(false);
  ui_->push_button_home->setEnabled(true);

  // Add the TPP widget
  {
    auto* tpp_dialog = new TPPDialog(tpp_node_, this);
    tpp_dialog->hide();
    connect(ui_->tool_button_tpp, &QToolButton::clicked, tpp_dialog, &QWidget::show);
    tpp_node_executor_.add_node(tpp_node_);
    tpp_node_future_ = std::async(std::launch::async, [this]() { tpp_node_executor_.spin(); });
  }

  // Add the trajectory preview widget
  {
    auto* preview = new trajectory_preview::TrajectoryPreviewWidget(this);
    preview->initializeROS(rviz_node, "motion_plan", "preview");

    auto* layout = new QVBoxLayout(ui_->frame_preview_widget);
    layout->addWidget(preview);
  }

  // Reset
  connect(ui_->push_button_reset, &QPushButton::clicked, [this]() {
    ui_->push_button_reset->setEnabled(false);
    ui_->push_button_start->setEnabled(true);
    ui_->push_button_home->setEnabled(true);
  });

  // Start
  connect(ui_->push_button_start, &QPushButton::clicked, [this]() {
    ui_->push_button_start->setEnabled(false);
    ui_->push_button_reset->setEnabled(true);
    ui_->push_button_home->setEnabled(false);
    ui_->text_edit_log->clear();
    ui_->stacked_widget->setCurrentIndex(0);
    ui_->group_box_operation->setEnabled(true);

    try
    {
      runTreeWithThread(snp_application::get_parameter<std::string>(bt_node_, BT_PARAM));
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  });

  // Go Home
  connect(ui_->push_button_home, &QPushButton::clicked, [this]() {
    ui_->push_button_start->setEnabled(false);
    ui_->push_button_reset->setEnabled(true);
    ui_->push_button_home->setEnabled(false);
    ui_->stacked_widget->setCurrentIndex(0);
    ui_->group_box_operation->setEnabled(true);

    try
    {
      runTreeWithThread(snp_application::get_parameter<std::string>(bt_node_, BT_FREESPACE_PARAM));
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Error", ex.what());
    }
  });

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->text_edit_log->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->text_edit_log->verticalScrollBar()->setSliderPosition(ui_->text_edit_log->verticalScrollBar()->maximum());
  });

  // Declare parameters
  bt_node_->declare_parameter<std::vector<std::string>>(BT_FILES_PARAM, {});
  bt_node_->declare_parameter<std::vector<std::string>>(BT_PLUGIN_LIBS_PARAM, {});
  bt_node_->declare_parameter<std::vector<std::string>>(BT_ROS_PLUGIN_LIBS_PARAM, {});
  bt_node_->declare_parameter<std::string>(BT_PARAM, "");
  bt_node_->declare_parameter<int>(BT_TIMEOUT_PARAM, 6000);  // seconds
  bt_node_->declare_parameter<std::string>(FOLLOW_JOINT_TRAJECTORY_ACTION, "follow_joint_trajectory");

  // Home state
  bt_node_->declare_parameter<std::string>(BT_FREESPACE_PARAM, "");
  bt_node_->declare_parameter<std::vector<double>>(HOME_STATE_JOINT_VALUES_PARAM, {});
  bt_node_->declare_parameter<std::vector<std::string>>(HOME_STATE_JOINT_NAMES_PARAM, {});

  // Set the error message key in the blackboard
  board_->set(ERROR_MESSAGE_KEY, "");

  // Populate the blackboard with buttons
  board_->set("stacked_widget", ui_->stacked_widget);
  board_->set("progress_bar", ui_->progress_bar);
  board_->set("reset", static_cast<QAbstractButton*>(ui_->push_button_reset));
  board_->set("halt", static_cast<QAbstractButton*>(ui_->push_button_halt));

  board_->set("back", static_cast<QAbstractButton*>(ui_->push_button_back));
  board_->set("scan", static_cast<QAbstractButton*>(ui_->push_button_scan));
  board_->set("tpp", static_cast<QAbstractButton*>(ui_->push_button_tpp));
  board_->set("plan", static_cast<QAbstractButton*>(ui_->push_button_motion_plan));
  board_->set("execute", static_cast<QAbstractButton*>(ui_->push_button_motion_execution));
  board_->set("tpp_config", static_cast<QAbstractButton*>(ui_->tool_button_tpp));
  board_->set("skip_scan", false);
}

BT::BehaviorTreeFactory SNPWidget::createBTFactory(int ros_timeout)
{
  BT::BehaviorTreeFactory bt_factory;

  // Register non-ROS plugins
  {
    auto bt_plugins = get_parameter<std::vector<std::string>>(bt_node_, BT_PLUGIN_LIBS_PARAM);
    for (const std::string& plugin : bt_plugins)
      bt_factory.registerFromPlugin(std::filesystem::path(plugin));
  }

  // Register ROS plugins
  {
    BT::RosNodeParams ros_params;
    ros_params.nh = bt_node_;
    ros_params.wait_for_server_timeout = std::chrono::seconds(0);
    ros_params.server_timeout = std::chrono::seconds(ros_timeout);

    auto bt_ros_plugins = get_parameter<std::vector<std::string>>(bt_node_, BT_ROS_PLUGIN_LIBS_PARAM);
    for (const std::string& plugin : bt_ros_plugins)
      RegisterRosNode(bt_factory, std::filesystem::path(plugin), ros_params);
  }

  // Get joint trajectory action topic name from parameter and store it in the blackboard
  board_->set(FOLLOW_JOINT_TRAJECTORY_ACTION,
              snp_application::get_parameter<std::string>(bt_node_, FOLLOW_JOINT_TRAJECTORY_ACTION));

  sensor_msgs::msg::JointState home_state;
  home_state.name = snp_application::get_parameter<std::vector<std::string>>(bt_node_, HOME_STATE_JOINT_NAMES_PARAM);
  home_state.position = snp_application::get_parameter<std::vector<double>>(bt_node_, HOME_STATE_JOINT_VALUES_PARAM);
  board_->set(HOME_STATE_NAME, home_state);

  return bt_factory;
}

void SNPWidget::runTreeWithThread(const std::string& bt_tree_name)
{
  try
  {
    auto* thread = new BTThread(this);

    // Create the BT factory
    BT::BehaviorTreeFactory bt_factory = createBTFactory(get_parameter<int>(bt_node_, BT_TIMEOUT_PARAM));

    auto bt_files = get_parameter<std::vector<std::string>>(bt_node_, BT_FILES_PARAM);
    if (bt_files.empty())
      throw std::runtime_error("Parameter '" + std::string(BT_FILES_PARAM) + "' is empty");

    for (const std::string& file : bt_files)
      bt_factory.registerBehaviorTreeFromFile(file);

    thread->tree = bt_factory.createTree(bt_tree_name, board_);
    logger_ = std::make_shared<TextEditLogger>(thread->tree.rootNode(), ui_->text_edit_log);

    connect(thread, &BTThread::finished, [thread, this]() {
      QString message;
      QTextStream stream(&message);
      switch (thread->result)
      {
        case BT::NodeStatus::SUCCESS:
          stream << "Behavior tree completed successfully";
          break;
        default:
          stream << "Behavior tree did not complete successfully";
          break;
      }

      if (!thread->message.isEmpty())
        stream << ": '" << thread->message << "'";

      QMetaObject::invokeMethod(ui_->text_edit_log, "append", Qt::QueuedConnection, Q_ARG(QString, message));
    });

    thread->start();
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, QString::fromStdString("Error"), QString::fromStdString(ex.what()));
    return;
  }
}

QStackedWidget* SNPWidget::getStackedWidget()
{
  return ui_->stacked_widget;
}

QTextEdit* SNPWidget::getTextEdit()
{
  return ui_->text_edit_log;
}

}  // namespace snp_application
