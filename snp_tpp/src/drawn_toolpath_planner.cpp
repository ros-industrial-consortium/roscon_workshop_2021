#include <snp_tpp/drawn_toolpath_planner.h>

#include <noether_tpp/utils.h>

#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkStripper.h>
#include <vtkAppendPolyData.h>
#include <vtkKdTreePointLocator.h>
#include <vtkCellLocator.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkMath.h>
#include <vtkErrorCode.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>

#include <Eigen/Geometry>

#include <tf2_eigen/tf2_eigen.h>

namespace snp_tpp
{
DrawnToolpathPlanner::DrawnToolpathPlanner(rclcpp::Node::SharedPtr node,
                                           std::vector<geometry_msgs::msg::PolygonStamped> boundaries)
  : boundaries_(std::move(boundaries)), buffer_(node->get_clock()), listener_(buffer_)
{
}

void DrawnToolpathPlanner::setPointSpacing(const double point_spacing)
{
  point_spacing_ = point_spacing;
}
void DrawnToolpathPlanner::setMinSegmentSize(const double min_segment_size)
{
  min_segment_size_ = min_segment_size;
}
void DrawnToolpathPlanner::setSearchRadius(const double search_radius)
{
  search_radius_ = search_radius;
}

noether::ToolPaths DrawnToolpathPlanner::plan(const pcl::PolygonMesh& mesh) const
{
  if (!noether::hasNormals(mesh))
    throw std::runtime_error("Mesh does not have vertex normals");

  // Convert input mesh to VTK type & calculate normals if necessary
  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(mesh, mesh_data);
  mesh_data->BuildLinks();
  mesh_data->BuildCells();

  // Build locators
  vtkSmartPointer<vtkKdTreePointLocator> kd_tree = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kd_tree->SetDataSet(mesh_data);
  kd_tree->BuildLocator();
  vtkSmartPointer<vtkCellLocator> cell_locator = vtkSmartPointer<vtkCellLocator>::New();
  cell_locator->SetDataSet(mesh_data);
  cell_locator->BuildLocator();

  noether::ToolPaths tool_paths;

  // Iterate over each polygon for each raster
  for (const auto& polygon : boundaries_)
  {
    // Need at least two points
    if (polygon.polygon.points.size() < 2)
      continue;

    // Transform polygon points to mesh frame
    Eigen::Isometry3d transform;
    try
    {
      geometry_msgs::msg::TransformStamped tf = buffer_.lookupTransform(mesh.header.frame_id, polygon.header.frame_id,
                                                                        tf2::TimePointZero, tf2::durationFromSec(3.0));
      transform = tf2::transformToEigen(tf.transform);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(rclcpp::get_logger("DrawnToolpathPlanner"), "Could not transform polygon: %s", ex.what());
      continue;
    }

    // Convert polygon points to Eigen vectors
    std::vector<Eigen::Vector3d> transformed_points;
    for (const auto& pt : polygon.polygon.points)
    {
      Eigen::Vector3d v(pt.x, pt.y, pt.z);
      transformed_points.push_back(transform * v);
    }

    // Generate waypoints along the polygon edges with cumulative length consideration
    vtkSmartPointer<vtkPoints> waypoints = vtkSmartPointer<vtkPoints>::New();
    double cumulative_distance = 0.0;
    double next_distance = 0.0;

    // Add first waypoint
    waypoints->InsertNextPoint(transformed_points.front().data());

    // Add waypoints along polygon edges, tracking total distance traveled
    for (size_t i = 0; i < transformed_points.size() - 1; ++i)
    {
      const Eigen::Vector3d& p0 = transformed_points[i];
      const Eigen::Vector3d& p1 = transformed_points[(i + 1)];
      Eigen::Vector3d direction = p1 - p0;
      double segment_length = direction.norm();
      direction.normalize();

      double segment_end_distance = cumulative_distance + segment_length;

      while (next_distance + point_spacing_ <= segment_end_distance)
      {
        next_distance += point_spacing_;
        double t = (next_distance - cumulative_distance) / segment_length;
        Eigen::Vector3d point = p0 + t * (p1 - p0);
        waypoints->InsertNextPoint(point.data());
      }

      cumulative_distance = segment_end_distance;
    }

    // Check segment length
    double line_length = noether::computeLength(waypoints);
    if (line_length < min_segment_size_)
      continue;

    // Project waypoints onto the mesh
    vtkSmartPointer<vtkPoints> projected_points = vtkSmartPointer<vtkPoints>::New();
    for (vtkIdType i = 0; i < waypoints->GetNumberOfPoints(); ++i)
    {
      double wp[3];
      waypoints->GetPoint(i, wp);
      double closest_point[3];
      double dist2;
      vtkIdType cell_id;
      int sub_id;
      cell_locator->FindClosestPoint(wp, closest_point, cell_id, sub_id, dist2);
      projected_points->InsertNextPoint(closest_point);
    }

    // Create segment data
    vtkSmartPointer<vtkPolyData> segment_data = vtkSmartPointer<vtkPolyData>::New();
    segment_data->SetPoints(projected_points);

    // Insert normals
    if (!noether::insertNormals(search_radius_, mesh_data, kd_tree, segment_data))
    {
      throw std::runtime_error("Could not insert normals for polygon raster");
    }

    // Convert to tool path
    noether::ToolPathSegment tool_path_segment;
    for (vtkIdType i = 0; i < segment_data->GetNumberOfPoints(); ++i)
    {
      double p[3];
      segment_data->GetPoint(i, p);
      double n[3];
      segment_data->GetPointData()->GetNormals()->GetTuple(i, n);

      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = Eigen::Vector3d(p[0], p[1], p[2]);

      // Compute orientation from normal
      Eigen::Vector3d normal(n[0], n[1], n[2]);
      normal.normalize();

      // Define a reference direction for the tool orientation
      Eigen::Vector3d ref_direction(1.0, 0.0, 0.0);
      if (std::abs(normal.dot(ref_direction)) > 0.99)
      {
        ref_direction = Eigen::Vector3d(0.0, 1.0, 0.0);
      }

      Eigen::Vector3d x_direction = ref_direction.cross(normal).normalized();
      Eigen::Vector3d y_direction = normal.cross(x_direction).normalized();

      pose.linear().col(0) = x_direction;
      pose.linear().col(1) = y_direction;
      pose.linear().col(2) = normal;

      tool_path_segment.push_back(pose);
    }

    // Add the segment to a tool path and then to tool paths
    noether::ToolPath tool_path;
    tool_path.push_back(tool_path_segment);
    tool_paths.push_back(tool_path);
  }

  return tool_paths;
}

}  // namespace snp_tpp
