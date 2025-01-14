#include "constant_tcp_speed_time_parameterization.hpp"
#include "constant_tcp_speed_time_parameterization_profile.hpp"

#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>

#include <tesseract_common/macros.h>
#include <console_bridge/console.h>
#include <boost/serialization/string.hpp>

#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory_utils.h>
#include <tesseract_task_composer/core/task_composer_task_plugin_factory.h>

namespace snp_motion_planning
{
class ConstantTCPSpeedTimeParameterizationTask : public tesseract_planning::TaskComposerTask
{
public:
  using Ptr = std::shared_ptr<ConstantTCPSpeedTimeParameterizationTask>;
  using ConstPtr = std::shared_ptr<const ConstantTCPSpeedTimeParameterizationTask>;
  using UPtr = std::unique_ptr<ConstantTCPSpeedTimeParameterizationTask>;
  using ConstUPtr = std::unique_ptr<const ConstantTCPSpeedTimeParameterizationTask>;

  static tesseract_planning::TaskComposerNodePorts ports()
  {
    tesseract_planning::TaskComposerNodePorts ports;
    ports.input_required["program"] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required["environment"] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.input_required["profiles"] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    ports.output_required["program"] = tesseract_planning::TaskComposerNodePorts::SINGLE;
    return ports;
  }

  ConstantTCPSpeedTimeParameterizationTask()
      : tesseract_planning::TaskComposerTask(CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME, ports(), true)
  {
  }

  explicit ConstantTCPSpeedTimeParameterizationTask(std::string name,
                                                    std::string input_key,
                                                    std::string output_key,
                                                    bool is_conditional = true)
      : tesseract_planning::TaskComposerTask(std::move(name), ports(), is_conditional)
  {
    input_keys_.add("program", std::move(input_key));
    output_keys_.add("program", std::move(output_key));
    validatePorts();
  }

  explicit ConstantTCPSpeedTimeParameterizationTask(
      std::string name, const YAML::Node& config,
      const tesseract_planning::TaskComposerPluginFactory& /*plugin_factory*/)
      : tesseract_planning::TaskComposerTask(std::move(name), ports(), config)
  {
  }

  ~ConstantTCPSpeedTimeParameterizationTask() override = default;
  ConstantTCPSpeedTimeParameterizationTask(const ConstantTCPSpeedTimeParameterizationTask&) = delete;
  ConstantTCPSpeedTimeParameterizationTask& operator=(const ConstantTCPSpeedTimeParameterizationTask&) = delete;
  ConstantTCPSpeedTimeParameterizationTask(ConstantTCPSpeedTimeParameterization&&) = delete;
  ConstantTCPSpeedTimeParameterizationTask& operator=(ConstantTCPSpeedTimeParameterizationTask&&) = delete;

  bool operator==(const ConstantTCPSpeedTimeParameterizationTask& rhs) const
  {
    bool equal = true;
    equal &= tesseract_planning::TaskComposerTask::operator==(rhs);
    return equal;
  }
  bool operator!=(const ConstantTCPSpeedTimeParameterizationTask& rhs) const
  {
    return !operator==(rhs);
  }

protected:
  friend struct tesseract_common::Serialization;
  friend class boost::serialization::access;
  tesseract_planning::TaskComposerNodeInfo::UPtr runImpl(tesseract_planning::TaskComposerContext& context,
                                                         OptionalTaskComposerExecutor /*executor*/) const override final
  {
    auto info = std::make_unique<tesseract_planning::TaskComposerNodeInfo>(*this);
    info->return_value = 0;

    // Get the composite instruction input
    auto input_data_poly = getData(*context.data_storage, "program");
    if (input_data_poly.isNull() ||
        input_data_poly.getType() != std::type_index(typeid(tesseract_planning::CompositeInstruction)))
    {
      info->status_message = "Input data " + input_keys_.get("program") + " is missing or of incorrect type";
      CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
      return info;
    }
    auto& ci = input_data_poly.as<tesseract_planning::CompositeInstruction>();
    const tesseract_common::ManipulatorInfo& manip_info = ci.getManipulatorInfo();

    // Get the environment input
    auto env_poly = getData(*context.data_storage, "environment");
    if (env_poly.isNull() ||
        env_poly.getType() != std::type_index(typeid(std::shared_ptr<const tesseract_environment::Environment>)))
    {
      info->status_message = "Input data " + input_keys_.get("environment") + " is missing or of incorrect type";
      CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
      return info;
    }
    auto env = env_poly.as<std::shared_ptr<const tesseract_environment::Environment>>();

    // Get the profile dictionary input
    auto profiles_poly = getData(*context.data_storage, "profiles");
    if (profiles_poly.isNull() ||
        profiles_poly.getType() != std::type_index(typeid(std::shared_ptr<tesseract_planning::ProfileDictionary>)))
    {
      info->status_message = "Input data " + input_keys_.get("profiles") + " is missing or of incorrect type";
      CONSOLE_BRIDGE_logError("%s", info->status_message.c_str());
      return info;
    }
    auto profiles = profiles_poly.as<std::shared_ptr<tesseract_planning::ProfileDictionary>>();

    // Get Composite Profile
    std::string profile = ci.getProfile();
    // profile = tesseract_planning::getProfileString(name_, profile, problem.composite_profile_remapping);
    auto cur_composite_profile = tesseract_planning::getProfile<ConstantTCPSpeedTimeParameterizationProfile>(
        name_, profile, *profiles, std::make_shared<ConstantTCPSpeedTimeParameterizationProfile>());
    cur_composite_profile =
        tesseract_planning::applyProfileOverrides(name_, profile, cur_composite_profile, ci.getProfileOverrides());

    // Create data structures for checking for plan profile overrides
    auto flattened = ci.flatten(tesseract_planning::moveFilter);
    if (flattened.empty())
    {
      info->status_message = "Cartesian time parameterization found no MoveInstructions to process";
      info->return_value = 1;
      return info;
    }

    // Solve using parameters
    tesseract_planning::TrajectoryContainer::Ptr trajectory =
        std::make_shared<tesseract_planning::InstructionsTrajectory>(ci);

    ConstantTCPSpeedTimeParameterization solver(
        env, manip_info.manipulator, manip_info.tcp_frame, cur_composite_profile->max_translational_velocity,
        cur_composite_profile->max_rotational_velocity, cur_composite_profile->max_translational_acceleration,
        cur_composite_profile->max_rotational_acceleration);

    if (!solver.compute(*trajectory, cur_composite_profile->max_velocity_scaling_factor,
                        cur_composite_profile->max_acceleration_scaling_factor))
    {
      info->status_message =
          "Failed to perform constant TCP speed time parameterization for process input: " + ci.getDescription();
      return info;
    }

    info->color = "green";
    info->status_message = "Constant TCP speed time parameterization succeeded";
    context.data_storage->setData("program", input_data_poly);
    info->return_value = 1;
    return info;
  }

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)
  {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(tesseract_planning::TaskComposerTask);
  }
};

}  // namespace snp_motion_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask)
BOOST_CLASS_EXPORT_IMPLEMENT(snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask)

TESSERACT_ADD_TASK_COMPOSER_NODE_PLUGIN(
    tesseract_planning::TaskComposerTaskFactory<snp_motion_planning::ConstantTCPSpeedTimeParameterizationTask>,
    ConstantTCPSpeedTimeParameterizationTaskFactory)
