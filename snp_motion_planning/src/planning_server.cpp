#include "planner_profiles.hpp"
#include "plugins/tasks/constant_tcp_speed_time_parameterization_profile.hpp"
#include "plugins/tasks/kinematic_limits_check_profile.hpp"
#include "plugins/tasks/tcp_speed_limiter_profile.hpp"

#include <rclcpp/rclcpp.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/planning/planning_task_composer_problem.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/planning/profiles/iterative_spline_parameterization_profile.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <tf2_eigen/tf2_eigen.h>

static const std::string TRANSITION_PLANNER = "TRANSITION";
static const std::string FREESPACE_PLANNER = "FREESPACE";
static const std::string RASTER_PLANNER = "RASTER";
static const std::string PROFILE = "SNPD";
static const std::string SCAN_LINK_NAME = "scan";
static const std::string OCTREE_RESOLUTION_PARAM = "octree_resolution";
static const std::string COLLISION_OBJECT_TYPE_PARAM = "collision_object_type";

// Profile parameters
static const std::string VERBOSE_PARAM = "verbose";
static const std::string TOUCH_LINKS_PARAM = "touch_links";
static const std::string MAX_TRANS_VEL_PARAM = "max_translational_vel";
static const std::string MAX_ROT_VEL_PARAM = "max_rotational_vel";
static const std::string MAX_TRANS_ACC_PARAM = "max_translational_acc";
static const std::string MAX_ROT_ACC_PARAM = "max_rotational_acc";
static const std::string CHECK_JOINT_ACC_PARAM = "check_joint_accelerations";
static const std::string VEL_SCALE_PARAM = "velocity_scaling_factor";
static const std::string ACC_SCALE_PARAM = "acceleration_scaling_factor";
static const std::string LVS_PARAM = "contact_check_longest_valid_segment";
static const std::string CONTACT_DIST_PARAM = "contact_check_distance";
static const std::string OMPL_MAX_PLANNING_TIME_PARAM = "ompl_max_planning_time";
static const std::string TCP_MAX_SPEED_PARAM = "tcp_max_speed";

// Task composer parameters
static const std::string TASK_COMPOSER_CONFIG_FILE_PARAM = "task_composer_config_file";
static const std::string TASK_NAME_PARAM = "task_name";

// Topics
static const std::string TESSERACT_MONITOR_NAMESPACE = "snp_environment";

// Services
static const std::string PLANNING_SERVICE = "create_motion_plan";
static const std::string REMOVE_SCAN_LINK_SERVICE = "remove_scan_link";

tesseract_common::Toolpath fromMsg(const std::vector<snp_msgs::msg::ToolPath>& paths)
{
  tesseract_common::Toolpath tps;
  tps.reserve(paths.size());
  for (const auto& path : paths)
  {
    for (const auto& segment : path.segments)
    {
      tesseract_common::VectorIsometry3d seg;
      seg.reserve(segment.poses.size());
      for (const auto& pose : segment.poses)
      {
        Eigen::Isometry3d p;
        tf2::fromMsg(pose, p);

        // Rotate the pose 180 degrees about the x-axis such that the z-axis faces into the part
        p *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

        seg.push_back(p);
      }
      tps.push_back(seg);
    }
  }
  return tps;
}

template <typename T>
T get(rclcpp::Node::SharedPtr node, const std::string& key)
{
  T val;
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

double clamp(const double val, const double min, const double max)
{
  return std::min(std::max(val, min), max);
}

static std::vector<tesseract_geometry::Geometry::Ptr> scanMeshToMesh(const std::string& filename)
{
  std::vector<tesseract_geometry::Mesh::Ptr> meshes =
      tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(filename);

  std::vector<tesseract_geometry::Geometry::Ptr> out;
  for (const auto& mesh : meshes)
    out.push_back(mesh);

  return out;
}

static std::vector<tesseract_geometry::Geometry::Ptr> scanMeshToConvexMesh(const std::string& filename)
{
  std::vector<tesseract_geometry::Mesh::Ptr> meshes =
      tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(filename);

  std::vector<tesseract_geometry::Geometry::Ptr> convex_meshes;
  for (tesseract_geometry::Mesh::Ptr mesh : meshes)
    convex_meshes.push_back(tesseract_collision::makeConvexMesh(*mesh));

  return convex_meshes;
}

static tesseract_geometry::Octree::Ptr
scanMeshToOctree(const std::string& filename, const double resolution,
                 const tesseract_geometry::Octree::SubType type = tesseract_geometry::Octree::SubType::SPHERE_INSIDE)
{
  std::vector<tesseract_geometry::Mesh::Ptr> geometries =
      tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>(filename);

  auto octree = std::make_shared<octomap::OcTree>(resolution);
  for (const auto& g : geometries)
  {
    for (const Eigen::Vector3d& v : *(g->getVertices()))
    {
      octree->updateNode(v.x(), v.y(), v.z(), true, true);
    }
  }
  octree->updateInnerOccupancy();

  return std::make_shared<tesseract_geometry::Octree>(octree, type);
}

static tesseract_environment::Commands
createScanAdditionCommands(const std::vector<tesseract_geometry::Geometry::Ptr>& geos, const std::string& mesh_frame,
                           const std::vector<std::string>& touch_links)
{
  tesseract_scene_graph::Link link(SCAN_LINK_NAME);

  for (tesseract_geometry::Geometry::Ptr geo : geos)
  {
    auto collision = std::make_shared<tesseract_scene_graph::Collision>();
    collision->geometry = geo;
    link.collision.push_back(collision);
  }

  tesseract_scene_graph::Joint joint("world_to_scan");
  joint.type = tesseract_scene_graph::JointType::FIXED;
  joint.parent_link_name = mesh_frame;
  joint.child_link_name = link.getName();

  tesseract_environment::Commands cmds;
  cmds.push_back(std::make_shared<tesseract_environment::AddLinkCommand>(link, joint, true));
  std::transform(touch_links.begin(), touch_links.end(), std::back_inserter(cmds),
                 [&link](const std::string& touch_link) {
                   tesseract_common::AllowedCollisionMatrix acm;
                   acm.addAllowedCollision(touch_link, link.getName(), "USER_DEFINED");
                   return std::make_shared<tesseract_environment::ModifyAllowedCollisionsCommand>(
                       acm, tesseract_environment::ModifyAllowedCollisionsType::ADD);
                 });

  return cmds;
}

class PlanningServer
{
public:
  PlanningServer(rclcpp::Node::SharedPtr node)
    : node_(node), env_(std::make_shared<tesseract_environment::Environment>())
  {
    // Declare ROS parameters
    node_->declare_parameter("robot_description", "");
    node_->declare_parameter("robot_description_semantic", "");
    node_->declare_parameter(VERBOSE_PARAM, false);
    node_->declare_parameter<std::vector<std::string>>(TOUCH_LINKS_PARAM, {});
    node_->declare_parameter(OCTREE_RESOLUTION_PARAM, 0.010);
    node_->declare_parameter(COLLISION_OBJECT_TYPE_PARAM, "convex_mesh");

    // Profiles
    node_->declare_parameter(MAX_TRANS_VEL_PARAM, 0.05);
    node_->declare_parameter(MAX_ROT_VEL_PARAM, 1.571);
    node_->declare_parameter(MAX_TRANS_ACC_PARAM, 0.1);
    node_->declare_parameter(MAX_ROT_ACC_PARAM, 3.14159);
    node_->declare_parameter<bool>(CHECK_JOINT_ACC_PARAM, false);
    node_->declare_parameter<double>(VEL_SCALE_PARAM, 1.0);
    node_->declare_parameter<double>(ACC_SCALE_PARAM, 1.0);
    node_->declare_parameter<double>(LVS_PARAM, 0.05);
    node_->declare_parameter<double>(CONTACT_DIST_PARAM, 0.0);
    node_->declare_parameter<double>(OMPL_MAX_PLANNING_TIME_PARAM, 5.0);
    node_->declare_parameter<double>(TCP_MAX_SPEED_PARAM, 0.25);

    // Task composer
    node_->declare_parameter(TASK_COMPOSER_CONFIG_FILE_PARAM, "");
    node_->declare_parameter(TASK_NAME_PARAM, "");

    {
      auto urdf_string = get<std::string>(node_, "robot_description");
      auto srdf_string = get<std::string>(node_, "robot_description_semantic");
      auto resource_locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
      if (!env_->init(urdf_string, srdf_string, resource_locator))
        throw std::runtime_error("Failed to initialize environment");
    }

    // Create the plotter
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getRootLinkName());

    // Create the environment monitor
    tesseract_monitor_ =
        std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node_, env_, TESSERACT_MONITOR_NAMESPACE);
    tesseract_monitor_->setEnvironmentPublishingFrequency(30.0);
    tesseract_monitor_->startPublishingEnvironment();
    tesseract_monitor_->startStateMonitor(tesseract_monitoring::DEFAULT_JOINT_STATES_TOPIC, false);

    // Advertise the ROS2 service
    server_ = node_->create_service<snp_msgs::srv::GenerateMotionPlan>(
        PLANNING_SERVICE, std::bind(&PlanningServer::plan, this, std::placeholders::_1, std::placeholders::_2));
    remove_scan_link_server_ = node_->create_service<std_srvs::srv::Empty>(
        REMOVE_SCAN_LINK_SERVICE,
        std::bind(&PlanningServer::removeScanLinkCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "Started SNP motion planning server");
  }

private:
  tesseract_planning::CompositeInstruction createProgram(const tesseract_common::ManipulatorInfo& info,
                                                         const tesseract_common::Toolpath& raster_strips)
  {
    std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

    tesseract_planning::CompositeInstruction program(PROFILE, tesseract_planning::CompositeInstructionOrder::ORDERED,
                                                     info);

    // Perform a freespace move to the first waypoint
    tesseract_planning::StateWaypoint swp1(joint_names, env_->getCurrentJointValues(joint_names));

    // Create object needed for move instruction
    tesseract_planning::StateWaypointPoly swp1_poly{ swp1 };

    tesseract_planning::MoveInstruction start_instruction(swp1_poly, tesseract_planning::MoveInstructionType::FREESPACE,
                                                          PROFILE, info);

    for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
    {
      if (rs == 0)
      {
        // Define from start composite instruction
        tesseract_planning::CartesianWaypoint wp1 = raster_strips[rs][0];
        tesseract_planning::CartesianWaypointPoly wp1_poly{ wp1 };
        tesseract_planning::MoveInstruction move_f0(wp1_poly, tesseract_planning::MoveInstructionType::FREESPACE,
                                                    PROFILE, info);
        move_f0.setDescription("from_start_plan");
        tesseract_planning::CompositeInstruction from_start(PROFILE);
        from_start.setDescription("from_start");
        from_start.appendMoveInstruction(start_instruction);
        from_start.appendMoveInstruction(move_f0);
        program.push_back(from_start);
      }

      // Define raster
      tesseract_planning::CompositeInstruction raster_segment(PROFILE);
      raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

      for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
      {
        tesseract_planning::CartesianWaypoint wp = raster_strips[rs][i];
        tesseract_planning::CartesianWaypointPoly wp_poly{ wp };
        raster_segment.appendMoveInstruction(tesseract_planning::MoveInstruction(
            wp_poly, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info));
      }
      program.push_back(raster_segment);

      if (rs < raster_strips.size() - 1)
      {
        // Add transition
        tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();
        tesseract_planning::CartesianWaypointPoly twp_poly{ twp };

        tesseract_planning::MoveInstruction transition_instruction1(
            twp_poly, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info);
        transition_instruction1.setDescription("Transition #" + std::to_string(rs + 1));

        tesseract_planning::CompositeInstruction transition(PROFILE);
        transition.setDescription("Transition #" + std::to_string(rs + 1));
        transition.appendMoveInstruction(transition_instruction1);

        program.push_back(transition);
      }
      else
      {
        // Add to end instruction
        tesseract_planning::MoveInstruction plan_f2(swp1_poly, tesseract_planning::MoveInstructionType::FREESPACE,
                                                    PROFILE, info);
        plan_f2.setDescription("to_end_plan");
        tesseract_planning::CompositeInstruction to_end(PROFILE);
        to_end.setDescription("to_end");
        to_end.appendMoveInstruction(plan_f2);
        program.push_back(to_end);
      }
    }

    return program;
  }

  void removeScanLink()
  {
    if (env_->getSceneGraph()->getLink(SCAN_LINK_NAME))
      env_->applyCommand(std::make_shared<tesseract_environment::RemoveLinkCommand>(SCAN_LINK_NAME));
  }

  void removeScanLinkCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
  {
    removeScanLink();
  }

  void plan(const snp_msgs::srv::GenerateMotionPlan::Request::SharedPtr req,
            snp_msgs::srv::GenerateMotionPlan::Response::SharedPtr res)
  {
    try
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Received motion planning request");

      tesseract_planning::ProfileDictionary::Ptr profile_dict =
          std::make_shared<tesseract_planning::ProfileDictionary>();
      // Add custom profiles
      {
        profile_dict->addProfile<tesseract_planning::SimplePlannerPlanProfile>(SIMPLE_DEFAULT_NAMESPACE, PROFILE,
                                                                               createSimplePlannerProfile());
        {
          auto profile = createOMPLProfile();
          profile->planning_time = get<double>(node_, OMPL_MAX_PLANNING_TIME_PARAM);
          profile_dict->addProfile<tesseract_planning::OMPLPlanProfile>(OMPL_DEFAULT_NAMESPACE, PROFILE, profile);
        }
        profile_dict->addProfile<tesseract_planning::TrajOptPlanProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE,
                                                                         createTrajOptToolZFreePlanProfile());
        profile_dict->addProfile<tesseract_planning::TrajOptCompositeProfile>(TRAJOPT_DEFAULT_NAMESPACE, PROFILE,
                                                                              createTrajOptProfile());
        profile_dict->addProfile<tesseract_planning::DescartesPlanProfile<float>>(DESCARTES_DEFAULT_NAMESPACE, PROFILE,
                                                                                  createDescartesPlanProfile<float>());
        profile_dict->addProfile<tesseract_planning::MinLengthProfile>(
            MIN_LENGTH_DEFAULT_NAMESPACE, PROFILE, std::make_shared<tesseract_planning::MinLengthProfile>(6));
        auto velocity_scaling_factor =
            clamp(get<double>(node_, VEL_SCALE_PARAM), std::numeric_limits<double>::epsilon(), 1.0);
        auto acceleration_scaling_factor =
            clamp(get<double>(node_, ACC_SCALE_PARAM), std::numeric_limits<double>::epsilon(), 1.0);

        // ISP profile
        profile_dict->addProfile<tesseract_planning::IterativeSplineParameterizationProfile>(
            ISP_DEFAULT_NAMESPACE, PROFILE,
            std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>(velocity_scaling_factor,
                                                                                         acceleration_scaling_factor));

        // Discrete contact check profile
        auto contact_check_lvs = get<double>(node_, LVS_PARAM);
        auto contact_check_dist = get<double>(node_, CONTACT_DIST_PARAM);
        profile_dict->addProfile<tesseract_planning::ContactCheckProfile>(
            CONTACT_CHECK_DEFAULT_NAMESPACE, PROFILE,
            std::make_shared<tesseract_planning::ContactCheckProfile>(contact_check_lvs, contact_check_dist));

        // Constant TCP time parameterization profile
        auto vel_trans = get<double>(node_, MAX_TRANS_VEL_PARAM);
        auto vel_rot = get<double>(node_, MAX_ROT_VEL_PARAM);
        auto acc_trans = get<double>(node_, MAX_TRANS_ACC_PARAM);
        auto acc_rot = get<double>(node_, MAX_ROT_ACC_PARAM);
        auto cart_time_param_profile =
            std::make_shared<snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile>(
                vel_trans, vel_rot, acc_trans, acc_rot, velocity_scaling_factor, acceleration_scaling_factor);
        profile_dict->addProfile<snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile>(
            CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME, PROFILE, cart_time_param_profile);

        // Kinematic limit check
        auto check_joint_acc = get<bool>(node_, CHECK_JOINT_ACC_PARAM);
        auto kin_limit_check_profile =
            std::make_shared<snp_motion_planning::KinematicLimitsCheckProfile>(true, true, check_joint_acc);
        profile_dict->addProfile<snp_motion_planning::KinematicLimitsCheckProfile>(KINEMATIC_LIMITS_CHECK_TASK_NAME,
                                                                                   PROFILE, kin_limit_check_profile);

        // TCP speed limit task
        double tcp_max_speed = get<double>(node_, TCP_MAX_SPEED_PARAM);  // m/s
        auto tcp_speed_limiter_profile = std::make_shared<snp_motion_planning::TCPSpeedLimiterProfile>(tcp_max_speed);
        profile_dict->addProfile<snp_motion_planning::TCPSpeedLimiterProfile>(TCP_SPEED_LIMITER_TASK_NAME, PROFILE,
                                                                              tcp_speed_limiter_profile);
      }

      // Create a manipulator info and program from the service request
      const std::string& base_frame = req->tool_paths.at(0).segments.at(0).header.frame_id;
      if (base_frame.empty())
      {
        throw std::runtime_error("Base frame is empty!");
      }
      if (req->motion_group.empty())
      {
        throw std::runtime_error("Motion group is empty!");
      }
      if (req->tcp_frame.empty())
      {
        throw std::runtime_error("TCP frame is empty!");
      }
      tesseract_common::ManipulatorInfo manip_info(req->motion_group, base_frame, req->tcp_frame);

      // Set up composite instruction and environment
      tesseract_planning::CompositeInstruction program = createProgram(manip_info, fromMsg(req->tool_paths));

      // Remove any previously added collision object
      removeScanLink();

      // Add the scan as a collision object to the environment
      if (!req->mesh_filename.empty())
      {
        auto collision_object_type = get<std::string>(node_, COLLISION_OBJECT_TYPE_PARAM);
        std::vector<tesseract_geometry::Geometry::Ptr> collision_objects;
        if (collision_object_type == "convex_mesh")
          collision_objects = scanMeshToConvexMesh(req->mesh_filename);
        else if (collision_object_type == "mesh")
          collision_objects = scanMeshToMesh(req->mesh_filename);
        else if (collision_object_type == "octree")
        {
          double octree_resolution = get<double>(node_, OCTREE_RESOLUTION_PARAM);
          if (octree_resolution < std::numeric_limits<double>::epsilon())
            throw std::runtime_error("Octree resolution must be > 0.0");
          collision_objects = { scanMeshToOctree(req->mesh_filename, octree_resolution) };
        }
        else
        {
          std::stringstream ss;
          ss << "Invalid collision object type (" << collision_object_type
             << ") for adding scan mesh to planning environment. Supported types are 'convex_mesh', 'mesh', and "
                "'octree'";
          throw std::runtime_error(ss.str());
        }

        tesseract_environment::Commands env_cmds = createScanAdditionCommands(
            collision_objects, req->mesh_frame, get<std::vector<std::string>>(node_, TOUCH_LINKS_PARAM));

        env_->applyCommands(env_cmds);
      }

      // Set up task composer problem
      auto task_composer_config_file = get<std::string>(node_, TASK_COMPOSER_CONFIG_FILE_PARAM);
      const YAML::Node task_composer_config = YAML::LoadFile(task_composer_config_file);
      tesseract_planning::TaskComposerPluginFactory factory(task_composer_config);

      auto task_name = get<std::string>(node_, TASK_NAME_PARAM);
      auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
      tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
      if (!task)
        throw std::runtime_error("Failed to create '" + task_name + "' task");

      // Save dot graph
      {
        std::ofstream tc_out_data(tesseract_common::getTempPath() + task_name + ".dot");
        task->dump(tc_out_data);
      }

      const std::string input_key = task->getInputKeys().front();
      const std::string output_key = task->getOutputKeys().front();
      auto task_data = std::make_shared<tesseract_planning::TaskComposerDataStorage>();
      task_data->setData(input_key, program);
      tesseract_planning::TaskComposerProblem::Ptr problem =
          std::make_shared<tesseract_planning::PlanningTaskComposerProblem>(env_, profile_dict);
      problem->dotgraph = true;

      // Update log level for debugging
      auto log_level = console_bridge::getLogLevel();
      if (get<bool>(node_, VERBOSE_PARAM))
      {
        console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

        // Dump dotgraphs of each task for reference
        const YAML::Node& task_plugins = task_composer_config["task_composer_plugins"]["tasks"]["plugins"];
        for (auto it = task_plugins.begin(); it != task_plugins.end(); ++it)
        {
          auto task_plugin_name = it->first.as<std::string>();
          std::ofstream f(tesseract_common::getTempPath() + task_plugin_name + ".dot");
          tesseract_planning::TaskComposerNode::Ptr task = factory.createTaskComposerNode(task_plugin_name);
          if (!task)
            throw std::runtime_error("Failed to load task: '" + task_plugin_name + "'");
          task->dump(f);
        }
      }

      // Run problem
      tesseract_planning::TaskComposerFuture::UPtr result = executor->run(*task, problem, task_data);
      result->wait();

      // Save the output dot graph
      {
        std::ofstream tc_out_results(tesseract_common::getTempPath() + task_name + "_results.dot");
        static_cast<const tesseract_planning::TaskComposerGraph&>(*task).dump(tc_out_results, nullptr,
                                                                              result->context->task_infos.getInfoMap());
      }

      // Reset the log level
      console_bridge::setLogLevel(log_level);

      // Check for successful plan
      if (!result->context->isSuccessful() || result->context->isAborted())
        throw std::runtime_error("Failed to create motion plan");

      // Get results of successful plan
      tesseract_planning::CompositeInstruction program_results =
          result->context->data_storage->getData(output_key).as<tesseract_planning::CompositeInstruction>();

      // Convert to joint trajectory
      tesseract_common::JointTrajectory jt = toJointTrajectory(program_results);

      // Send joint trajectory to Tesseract plotter widget
      plotter_->plotTrajectory(jt, *env_->getStateSolver());

      // Return results
      res->motion_plan = tesseract_rosutils::toMsg(jt, env_->getState());
      res->message = "Successfully planned motion";
      res->success = true;
    }
    catch (const std::exception& ex)
    {
      res->message = ex.what();
      res->success = false;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), res->message);
  }

  rclcpp::Node::SharedPtr node_;

  tesseract_environment::Environment::Ptr env_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr tesseract_monitor_;
  tesseract_rosutils::ROSPlottingPtr plotter_;
  rclcpp::Service<snp_msgs::srv::GenerateMotionPlan>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr remove_scan_link_server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("snp_planning_server");
  auto server = std::make_shared<PlanningServer>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
