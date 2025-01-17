#include <snp_tpp/roi_selection_mesh_modifier_widget.h>
#include <snp_tpp/snp_raster_planner_widget.h>
#include <snp_tpp/drawn_toolpath_planner_widget.h>

#include <noether_gui/plugin_interface.h>
#include <yaml-cpp/yaml.h>
#include <noether_gui/widgets/tool_path_planners/raster/raster_planner_widget.h>

namespace snp_tpp
{
struct ROISelectionMeshModifierWidgetPlugin : public noether::MeshModifierWidgetPlugin
{
  QWidget* create(QWidget* parent, const YAML::Node& config = {}) const override
  {
    auto widget = new ROISelectionMeshModifierWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

// Raster Tool Path Planners
struct SNPRasterPlannerWidgetPlugin : public noether::ToolPathPlannerWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override final
  {
    auto widget = new SNPRasterPlannerWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

struct DrawnToolpathPlannerWidgetPlugin : public noether::ToolPathPlannerWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override final
  {
    auto widget = new DrawnToolpathPlannerWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

}  // namespace snp_tpp

EXPORT_MESH_MODIFIER_WIDGET_PLUGIN(snp_tpp::ROISelectionMeshModifierWidgetPlugin, ROISelectionMeshModifier)
EXPORT_TPP_WIDGET_PLUGIN(snp_tpp::SNPRasterPlannerWidgetPlugin, SNPRasterPlanner)
EXPORT_TPP_WIDGET_PLUGIN(snp_tpp::DrawnToolpathPlannerWidgetPlugin, DrawnToolpathPlanner)
