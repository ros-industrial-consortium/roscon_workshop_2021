#pragma once

#include <descartes_light/edge_evaluators/compound_edge_evaluator.h>
#include <descartes_light/edge_evaluators/euclidean_distance_edge_evaluator.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";
static const std::string MIN_LENGTH_DEFAULT_NAMESPACE = "MinLengthTask";
static const std::string CONTACT_CHECK_DEFAULT_NAMESPACE = "DiscreteContactCheckTask";
static const std::string ISP_DEFAULT_NAMESPACE = "IterativeSplineParameterizationTask";

/**
 * @brief Container for a collision pair with a specifiable minimum allowable contact distance between the link pair
 */
struct ExplicitCollisionPair
{
  explicit ExplicitCollisionPair(std::string first_, std::string second_, double distance_)
    : first(first_), second(second_), distance(distance_)
  {
  }

  /** @brief Name of the first link */
  std::string first;
  /** @brief Name of the second link */
  std::string second;
  /** @brief Minimum allowable contact distance between the two links */
  double distance;
};

template <typename FloatType>
typename tesseract_planning::DescartesDefaultPlanProfile<FloatType>::Ptr
createDescartesPlanProfile(FloatType min_contact_distance = static_cast<FloatType>(0.0),
                           const std::vector<ExplicitCollisionPair>& unique_collision_pairs = {})
{
  auto profile = std::make_shared<tesseract_planning::DescartesDefaultPlanProfile<FloatType>>();
  profile->num_threads = static_cast<int>(std::thread::hardware_concurrency());
  profile->use_redundant_joint_solutions = false;

  // Collision checking
  profile->allow_collision = false;
  profile->enable_collision = true;
  profile->vertex_collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->vertex_collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(
      min_contact_distance);

  profile->vertex_collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->vertex_collision_check_config.contact_manager_config.margin_data.setPairCollisionMargin(
        pair.first, pair.second, pair.distance);

  profile->enable_edge_collision = false;
  profile->edge_collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->edge_collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(
      min_contact_distance);

  profile->edge_collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->edge_collision_check_config.contact_manager_config.margin_data.setPairCollisionMargin(
        pair.first, pair.second, pair.distance);

  // Use the default state and edge evaluators
  profile->state_evaluator = nullptr;
  profile->edge_evaluator = [](const tesseract_planning::DescartesProblem<FloatType> & /*prob*/) ->
      typename descartes_light::EdgeEvaluator<FloatType>::Ptr {
        auto eval = std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>();

        // Nominal Euclidean distance
        eval->evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>());

        // Penalize wrist motion
        //        Eigen::Matrix<FloatType, Eigen::Dynamic, 1> wrist_mask(prob.manip->numJoints());
        //        FloatType weight = static_cast<FloatType>(5.0);
        //        wrist_mask << 0.0, 0.0, 0.0, weight, weight, weight;
        //        eval->evaluators.push_back(
        //            std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(wrist_mask));

        return eval;
      };

  profile->vertex_evaluator = nullptr;

  profile->target_pose_sampler =
      std::bind(tesseract_planning::sampleToolZAxis, std::placeholders::_1, 10.0 * M_PI / 180.0);

  return profile;
}

tesseract_planning::OMPLDefaultPlanProfile::Ptr createOMPLProfile(
    const double min_contact_distance = 0.0, const std::vector<ExplicitCollisionPair>& unique_collision_pairs = {})
{
  // OMPL freespace and transition profiles
  // Create the RRT parameters
  auto n = static_cast<Eigen::Index>(std::thread::hardware_concurrency());
  auto range = Eigen::VectorXd::LinSpaced(n, 0.05, 0.5);

  // Add as many planners as available threads so mulitple OMPL plans can happen in parallel
  auto profile = std::make_shared<tesseract_planning::OMPLDefaultPlanProfile>();
  profile->planning_time = 5.0;
  profile->max_solutions = 1;

  profile->planners.clear();
  profile->planners.reserve(static_cast<std::size_t>(n));
  for (Eigen::Index i = 0; i < n; ++i)
  {
    auto rrt = std::make_shared<tesseract_planning::RRTConnectConfigurator>();
    rrt->range = range(i);
    profile->planners.push_back(rrt);
  }

  // Collision checking
  profile->collision_check_config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->collision_check_config.contact_manager_config.margin_data.setDefaultCollisionMargin(min_contact_distance);
  profile->collision_check_config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;

  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->collision_check_config.contact_manager_config.margin_data.setPairCollisionMargin(pair.first, pair.second,
                                                                                              pair.distance);

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptPlanProfile>
createTrajOptToolZFreePlanProfile(const Eigen::VectorXd& cart_tolerance = Eigen::VectorXd::Zero(6),
                                  const Eigen::VectorXd& cart_coeff = Eigen::VectorXd::Constant(6, 1, 2.5))
{
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();

  profile->cartesian_cost_config.enabled = true;
  profile->cartesian_cost_config.use_tolerance_override = true;
  profile->cartesian_cost_config.lower_tolerance = Eigen::VectorXd::Zero(6);
  profile->cartesian_cost_config.upper_tolerance = Eigen::VectorXd::Zero(6);
  profile->cartesian_cost_config.coeff = cart_coeff;

  profile->cartesian_constraint_config.enabled = true;
  profile->cartesian_constraint_config.use_tolerance_override = true;
  profile->cartesian_constraint_config.lower_tolerance = -1 * cart_tolerance;
  profile->cartesian_constraint_config.upper_tolerance = cart_tolerance;
  profile->cartesian_constraint_config.coeff = cart_coeff;

  profile->joint_cost_config.enabled = false;
  profile->joint_cost_config.coeff = Eigen::VectorXd::Constant(1, 1, 5);

  profile->joint_constraint_config.enabled = true;
  profile->joint_constraint_config.coeff = Eigen::VectorXd::Constant(1, 1, 5);

  return profile;
}

std::shared_ptr<tesseract_planning::TrajOptDefaultCompositeProfile> createTrajOptProfile(
    double min_contact_distance = 0.0, const std::vector<ExplicitCollisionPair>& unique_collision_pairs = {})
{
  // TrajOpt profiles
  auto profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
  profile->smooth_velocities = true;
  profile->velocity_coeff = Eigen::VectorXd::Constant(1, 1, 10.0);
  profile->acceleration_coeff = Eigen::VectorXd::Constant(1, 1, 25.0);
  profile->jerk_coeff = Eigen::VectorXd::Constant(1, 1, 50.0);

  profile->contact_test_type = tesseract_collision::ContactTestType::CLOSEST;

  profile->collision_cost_config.enabled = false;
  profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  profile->collision_cost_config.safety_margin = min_contact_distance;
  profile->collision_cost_config.safety_margin_buffer = std::max(0.020, 2 * min_contact_distance);
  profile->collision_cost_config.coeff = 10.0;

  profile->special_collision_cost =
      std::make_shared<trajopt_common::SafetyMarginData>(min_contact_distance, profile->collision_cost_config.coeff);
  for (const ExplicitCollisionPair& pair : unique_collision_pairs)
    profile->special_collision_cost->setPairSafetyMarginData(pair.first, pair.second, pair.distance,
                                                             profile->collision_cost_config.coeff);

  profile->collision_constraint_config.enabled = true;
  profile->collision_constraint_config.type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;
  profile->collision_constraint_config.safety_margin = min_contact_distance;
  profile->collision_constraint_config.safety_margin_buffer = std::max(0.020, 2 * min_contact_distance);
  profile->collision_constraint_config.coeff = 10.0;

  return profile;
}

std::shared_ptr<tesseract_planning::SimplePlannerLVSPlanProfile> createSimplePlannerProfile()
{
  return std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>(5 * M_PI / 180, 0.1, 5 * M_PI / 180, 1);
}

tesseract_planning::ContactCheckProfile::Ptr
createContactCheckProfile(double longest_valid_segment_distance, double min_contact_distance = 0.0,
                          const std::vector<ExplicitCollisionPair>& collision_pairs = {})
{
  auto profile =
      std::make_shared<tesseract_planning::ContactCheckProfile>(longest_valid_segment_distance, min_contact_distance);
  profile->config.contact_request.type = tesseract_collision::ContactTestType::FIRST;
  profile->config.contact_manager_config.margin_data_override_type =
      tesseract_common::CollisionMarginOverrideType::MODIFY;
  for (const ExplicitCollisionPair& pair : collision_pairs)
    profile->config.contact_manager_config.margin_data.setPairCollisionMargin(pair.first, pair.second, pair.distance);

  return profile;
}
