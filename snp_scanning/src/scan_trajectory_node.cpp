#include <rclcpp/rclcpp.hpp>
#include <snp_msgs/srv/generate_scan_motion_plan.hpp>
#include <snp_msgs/srv/generate_motion_plan.hpp>
#include <snp_msgs/msg/tool_paths.h>
#include <noether_tpp/tool_path_planners/flat_plane_toolpath_planner.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>

static const std::string MESH_FILE_PARAMETER = "mesh_file";
static const std::string TARGET_FRAME_PARAMETER = "target_frame";
static const std::string BASE_FRAME_PARAMETER = "base_frame";
static const std::string PLANE_LENGTH_PARAMETER = "plane_length";
static const std::string PLANE_WIDTH_PARAMETER = "plane_width";
static const std::string X_SPACING_PARAMETER = "x_spacing";
static const std::string Y_SPACING_PARAMETER = "y_spacing";
static const std::string MOTION_GROUP_PARAMETER = "motion_group";
static const std::string TCP_FRAME_PARAMETER = "tcp_frame";
static const std::string REFERENCE_FRAME_PARAMETER = "reference_frame";
static const std::string HEIGHT_OFFSET_PARAMETER = "height_offset";

class ScanTrajServer : public rclcpp::Node
{
public:
  explicit ScanTrajServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options), cb_group_(create_callback_group(rclcpp::CallbackGroupType::Reentrant))
  {
    using namespace std::placeholders;

    service_ = create_service<snp_msgs::srv::GenerateScanMotionPlan>(
        "generate_scan_motion_plan",
        std::bind(&ScanTrajServer::generateScanTrajCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_);
    gen_scan_traj_cli_ = create_client<snp_msgs::srv::GenerateMotionPlan>("create_motion_plan");

    declare_parameter<std::string>(TARGET_FRAME_PARAMETER, "");
    declare_parameter<std::string>(BASE_FRAME_PARAMETER, "");
    declare_parameter<double>(PLANE_LENGTH_PARAMETER, 0.0);
    declare_parameter<double>(PLANE_WIDTH_PARAMETER, 0.0);
    declare_parameter<double>(X_SPACING_PARAMETER, 0.0);
    declare_parameter<double>(Y_SPACING_PARAMETER, 0.0);
    declare_parameter<double>(HEIGHT_OFFSET_PARAMETER, 0.0);
    declare_parameter<std::string>(MOTION_GROUP_PARAMETER, "");
    declare_parameter<std::string>(TCP_FRAME_PARAMETER, "");
    declare_parameter<std::string>(MESH_FILE_PARAMETER, "");
    declare_parameter<std::string>(REFERENCE_FRAME_PARAMETER, "");
  }

private:
  rclcpp::Client<snp_msgs::srv::GenerateMotionPlan>::SharedPtr gen_scan_traj_cli_;
  rclcpp::Service<snp_msgs::srv::GenerateScanMotionPlan>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment& segment)
  {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = this->get_parameter(TARGET_FRAME_PARAMETER).as_string();
    pose_array.poses.reserve(segment.size());
    for (auto waypoint : segment)
    {
      // Renormalize orientation
      Eigen::Quaterniond q(waypoint.linear());
      q.normalize();
      waypoint.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();

      pose_array.poses.push_back(tf2::toMsg(waypoint));
    }
    return pose_array;
  }

  snp_msgs::msg::ToolPath toMsg(const noether::ToolPath& path)
  {
    snp_msgs::msg::ToolPath path_msg;
    path_msg.segments.reserve(path.size());
    for (const auto& segment : path)
      path_msg.segments.push_back(toMsg(segment));
    return path_msg;
  }

  std::vector<snp_msgs::msg::ToolPath> toMsg(const noether::ToolPaths& paths)
  {
    std::vector<snp_msgs::msg::ToolPath> paths_msg;
    paths_msg.reserve(paths.size());
    for (const auto& path : paths)
      paths_msg.push_back(toMsg(path));
    return paths_msg;
  }

  void generateScanTrajCallback(const std::shared_ptr<snp_msgs::srv::GenerateScanMotionPlan::Request> req,
                                std::shared_ptr<snp_msgs::srv::GenerateScanMotionPlan::Response> res)
  {
    double plane_length_val = this->get_parameter(PLANE_LENGTH_PARAMETER).as_double();
    double plane_width_val = this->get_parameter(PLANE_WIDTH_PARAMETER).as_double();
    double x_spacing_val = this->get_parameter(X_SPACING_PARAMETER).as_double();
    double y_spacing_val = this->get_parameter(Y_SPACING_PARAMETER).as_double();
    double height_offset_val = this->get_parameter(HEIGHT_OFFSET_PARAMETER).as_double();

    Eigen::Vector2d plane_dims(plane_length_val, plane_width_val);

    Eigen::Vector2d point_spacing(x_spacing_val, y_spacing_val);

    Eigen::Isometry3d offset =
        Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, height_offset_val)) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::Translation3d(Eigen::Vector3d(plane_length_val / 2.0, -plane_width_val / 2.0, 0.0));

    noether::FlatPlaneToolPathPlanner flat_plane_tpp(plane_dims, point_spacing, offset);
    noether::ToolPaths tps = flat_plane_tpp.plan(pcl::PolygonMesh());

    auto request_ = std::make_shared<snp_msgs::srv::GenerateMotionPlan::Request>();
    request_->motion_group = this->get_parameter(MOTION_GROUP_PARAMETER).as_string();
    request_->tcp_frame = this->get_parameter(TCP_FRAME_PARAMETER).as_string();
    ;
    request_->tool_paths = this->toMsg(tps);

    auto future = gen_scan_traj_cli_->async_send_request(request_);
    future.wait();

    snp_msgs::srv::GenerateMotionPlan::Response::SharedPtr response_ = future.get();
    res->success = response_->success;
    res->message = response_->message;
    res->motion_plan = response_->motion_plan;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ScanTrajServer>("scan_traj_service");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
