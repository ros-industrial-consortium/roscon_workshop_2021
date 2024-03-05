#include <snp_application/snp_widget.h>
#include "ui_snp_widget.h"
// BT
#include <snp_application/bt/bt_thread.h>
#include <snp_application/bt/button_approval_node.h>
#include <snp_application/bt/button_monitor_node.h>
#include <snp_application/bt/progress_decorator_node.h>
#include <snp_application/bt/set_page_decorator_node.h>
#include <snp_application/bt/snp_bt_ros_nodes.h>
#include <snp_application/bt/snp_sequence_with_memory_node.h>
#include <snp_application/bt/text_edit_logger.h>
#include <snp_application/bt/utils.h>

#include <behaviortree_cpp/bt_factory.h>
#include <boost_plugin_loader/plugin_loader.h>
#include <QMessageBox>
#include <QTextStream>
#include <QScrollBar>
#include <snp_tpp/tpp_widget.h>
#include <trajectory_preview/trajectory_preview_widget.h>

static const std::string BT_FILES_PARAM = "bt_files";
static const std::string BT_PARAM = "tree";
static const std::string BT_SHORT_TIMEOUT_PARAM = "bt_short_timeout";
static const std::string BT_LONG_TIMEOUT_PARAM = "bt_long_timeout";

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
  });

  // Start
  connect(ui_->push_button_start, &QPushButton::clicked, [this]() {
    ui_->push_button_start->setEnabled(false);
    ui_->push_button_reset->setEnabled(true);
    ui_->text_edit_log->clear();
    ui_->stacked_widget->setCurrentIndex(0);
    ui_->group_box_operation->setEnabled(true);
    runTreeWithThread();
  });

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->text_edit_log->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->text_edit_log->verticalScrollBar()->setSliderPosition(ui_->text_edit_log->verticalScrollBar()->maximum());
  });

  // Declare parameters
  bt_node_->declare_parameter<std::string>(MOTION_GROUP_PARAM, "");
  bt_node_->declare_parameter<std::string>(REF_FRAME_PARAM, "");
  bt_node_->declare_parameter<std::string>(TCP_FRAME_PARAM, "");
  bt_node_->declare_parameter<std::string>(CAMERA_FRAME_PARAM, "");
  bt_node_->declare_parameter<std::string>(MESH_FILE_PARAM, "");
  bt_node_->declare_parameter<double>(START_STATE_REPLACEMENT_TOLERANCE_PARAM, 1.0 * M_PI / 180.0);
  bt_node_->declare_parameter<std::vector<std::string>>(BT_FILES_PARAM, {});
  bt_node_->declare_parameter<std::string>(BT_PARAM, "");
  bt_node_->declare_parameter<int>(BT_SHORT_TIMEOUT_PARAM, 5);    // seconds
  bt_node_->declare_parameter<int>(BT_LONG_TIMEOUT_PARAM, 6000);  // seconds

  bt_node_->declare_parameter<float>(IR_TSDF_VOXEL_PARAM, 0.01f);
  bt_node_->declare_parameter<float>(IR_TSDF_SDF_PARAM, 0.03f);
  bt_node_->declare_parameter<double>(IR_TSDF_MIN_X_PARAM, 0.0);
  bt_node_->declare_parameter<double>(IR_TSDF_MIN_Y_PARAM, 0.0);
  bt_node_->declare_parameter<double>(IR_TSDF_MIN_Z_PARAM, 0.0);
  bt_node_->declare_parameter<double>(IR_TSDF_MAX_X_PARAM, 0.0);
  bt_node_->declare_parameter<double>(IR_TSDF_MAX_Y_PARAM, 0.0);
  bt_node_->declare_parameter<double>(IR_TSDF_MAX_Z_PARAM, 0.0);
  bt_node_->declare_parameter<float>(IR_RGBD_DEPTH_SCALE_PARAM, 1000.0);
  bt_node_->declare_parameter<float>(IR_RGBD_DEPTH_TRUNC_PARAM, 1.1f);
  bt_node_->declare_parameter<bool>(IR_LIVE_PARAM, true);

  // Set the error message key in the blackboard
  board_->set(ERROR_MESSAGE_KEY, "");

  // Populate the blackboard with buttons
  board_->set(SetPageDecoratorNode::STACKED_WIDGET_KEY, ui_->stacked_widget);
  board_->set(ProgressDecoratorNode::PROGRESS_BAR_KEY, ui_->progress_bar);
  board_->set("reset", static_cast<QAbstractButton*>(ui_->push_button_reset));
  board_->set("halt", static_cast<QAbstractButton*>(ui_->push_button_halt));

  board_->set("back", static_cast<QAbstractButton*>(ui_->push_button_back));
  board_->set("scan", static_cast<QAbstractButton*>(ui_->push_button_scan));
  board_->set("tpp", static_cast<QAbstractButton*>(ui_->push_button_tpp));
  board_->set("plan", static_cast<QAbstractButton*>(ui_->push_button_motion_plan));
  board_->set("execute", static_cast<QAbstractButton*>(ui_->push_button_motion_execution));
}

BT::BehaviorTreeFactory SNPWidget::createBTFactory(int ros_short_timeout, int ros_long_timeout)
{
  BT::BehaviorTreeFactory bt_factory;

  // Register custom nodes
  bt_factory.registerNodeType<ButtonMonitorNode>("ButtonMonitor");
  bt_factory.registerNodeType<ButtonApprovalNode>("ButtonApproval");
  bt_factory.registerNodeType<ProgressDecoratorNode>("Progress");
  bt_factory.registerNodeType<SetPageDecoratorNode>("SetPage");
  bt_factory.registerNodeType<SNPSequenceWithMemory>("SNPSequenceWithMemory");
  bt_factory.registerNodeType<RosSpinnerNode>("RosSpinner", bt_node_);

  BT::RosNodeParams ros_params;
  ros_params.nh = bt_node_;
  ros_params.wait_for_server_timeout = std::chrono::seconds(0);
  ros_params.server_timeout = std::chrono::seconds(ros_short_timeout);

  // Publishers/Subscribers
  bt_factory.registerNodeType<ToolPathsPubNode>("ToolPathsPub", ros_params);
  bt_factory.registerNodeType<MotionPlanPubNode>("MotionPlanPub", ros_params);
  bt_factory.registerNodeType<UpdateTrajectoryStartStateNode>("UpdateTrajectoryStartState", ros_params);
  // Short-running services
  bt_factory.registerNodeType<TriggerServiceNode>("TriggerService", ros_params);
  bt_factory.registerNodeType<GenerateToolPathsServiceNode>("GenerateToolPathsService", ros_params);
  bt_factory.registerNodeType<StartReconstructionServiceNode>("StartReconstructionService", ros_params);
  bt_factory.registerNodeType<StopReconstructionServiceNode>("StopReconstructionService", ros_params);

  // Long-running services/actions
  ros_params.server_timeout = std::chrono::seconds(ros_long_timeout);
  bt_factory.registerNodeType<ExecuteMotionPlanServiceNode>("ExecuteMotionPlanService", ros_params);
  bt_factory.registerNodeType<GenerateMotionPlanServiceNode>("GenerateMotionPlanService", ros_params);
  bt_factory.registerNodeType<GenerateScanMotionPlanServiceNode>("GenerateScanMotionPlanService", ros_params);
  bt_factory.registerNodeType<FollowJointTrajectoryActionNode>("FollowJointTrajectoryAction", ros_params);

  return bt_factory;
}

void SNPWidget::runTreeWithThread()
{
  try
  {
    auto* thread = new BTThread(this);

    // Create the BT factory
    BT::BehaviorTreeFactory bt_factory = createBTFactory(get_parameter<int>(bt_node_, BT_SHORT_TIMEOUT_PARAM),
                                                         get_parameter<int>(bt_node_, BT_LONG_TIMEOUT_PARAM));

    auto bt_files = get_parameter<std::vector<std::string>>(bt_node_, BT_FILES_PARAM);
    if (bt_files.empty())
      throw std::runtime_error("Parameter '" + BT_FILES_PARAM + "' is empty");

    for (const std::string& file : bt_files)
      bt_factory.registerBehaviorTreeFromFile(file);

    auto bt_tree_name = get_parameter<std::string>(bt_node_, BT_PARAM);
    if (bt_tree_name.empty())
      throw std::runtime_error("Parameter '" + BT_PARAM + "' is not set");

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

}  // namespace snp_application
