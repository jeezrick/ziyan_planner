#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <chrono>

#include "Eigen/Core"

#include "pathplanner/astar_planner_hybrid.hpp"
#include "pathplanner/logger.hpp"
#include "pathplanner/cfg.hpp"

#define BENCHMARK_TESTING

namespace ziyan_planner
{

using namespace std::chrono;  // NOLINT

AstarPlannerHybrid::AstarPlannerHybrid(const std::string & cfg_path) : AstarPlanner(cfg_path),
_a_star(nullptr),
_collision_checker(nullptr, 1),
_smoother(nullptr),
_costmap(nullptr),
// _costmap_downsampler(nullptr)
_costmap_manager(nullptr)
{
  _node = std::make_shared<Info>();
  auto configMap = parseConfigFile(cfg_path);
  readConfigFileToInfo(configMap, _node); 

  ZIYAN_INFO("Read cfg done.");

  init();
}

AstarPlannerHybrid::AstarPlannerHybrid(
  const Info::SharedPtr & parent) : AstarPlanner(parent), 
_a_star(nullptr),
_collision_checker(nullptr, 1),
_smoother(nullptr),
_costmap(nullptr),
// _costmap_downsampler(nullptr)
_costmap_manager(nullptr)
{
  _node = parent;
  init();
}

void AstarPlannerHybrid::init() 
{
  auto node = &(_node -> plannerhybrid_params);

  int angle_quantizations;
  bool smooth_path;

  // General planner params
  _tolerance = node->tolerance;
  _downsample_costmap = node->downsample_costmap;
  _downsampling_factor = node->downsampling_factor;
  _search_info.cost_penalty = node->cost_travel_multiplier;
  _allow_unknown = node->allow_unknown;
  _max_iterations = node->max_iterations;
  _max_on_approach_iterations = node->max_on_approach_iterations;
  _terminal_checking_interval = node->terminal_checking_interval;
  _max_planning_time = node->max_planning_time;

  angle_quantizations = node->angle_quantization_bins;
  _angle_bin_size = 2.0 * M_PI / angle_quantizations;
  _angle_quantizations = static_cast<unsigned int>(angle_quantizations);
  smooth_path = node->smooth_path;

  _motion_model = MotionModel::TWOD;

  _minimum_turning_radius_global_coords = node->minimum_turning_radius;

  _search_info.allow_primitive_interpolation = node->allow_primitive_interpolation;
  _search_info.cache_obstacle_heuristic = node->cache_obstacle_heuristic;
  _search_info.reverse_penalty = node->reverse_penalty;
  _search_info.change_penalty = node->change_penalty;
  _search_info.non_straight_penalty = node->non_straight_penalty;
  _search_info.cost_penalty = node->cost_penalty;
  _search_info.retrospective_penalty = node->retrospective_penalty;
  _search_info.analytic_expansion_ratio = node->analytic_expansion_ratio;
  _search_info.analytic_expansion_max_cost = node->analytic_expansion_max_cost;
  _search_info.analytic_expansion_max_cost_override = node->analytic_expansion_max_cost_override;
  _search_info.use_quadratic_cost_penalty = node->use_quadratic_cost_penalty;
  _search_info.downsample_obstacle_heuristic = node->downsample_obstacle_heuristic;

  analytic_expansion_max_length_m = node->analytic_expansion_max_length;

  _lookup_table_size = node->lookup_table_size;
  _debug_visualizations = node->_debug_visualizations;

  _motion_model_for_search = node->motion_model_for_search;
  _motion_model = fromString(_motion_model_for_search);

  if (_motion_model == MotionModel::UNKNOWN) {
    ZIYAN_INFO(
      "Unable to get MotionModel search type. Given '%s', "
      "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP, STATE_LATTICE.",
      _motion_model_for_search.c_str());
  }

  if (_max_on_approach_iterations <= 0) {
    ZIYAN_INFO(
      "On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    ZIYAN_INFO(
      "maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    params.get(_node);
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
  }
}

AstarPlannerHybrid::~AstarPlannerHybrid()
{
  ZIYAN_INFO("Destroying plugin of type AstarPlannerHybrid");
}

void AstarPlannerHybrid::setMap(
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
  double origin_x, double origin_y, uint8_t* data_ptr) 
{
  _costmap_manager = std::make_shared<CostmapManager>(
    _node, cells_size_x, cells_size_y, resolution, origin_x, origin_y, data_ptr);
  _setMap_core();
}

void AstarPlannerHybrid::setMap(
  std::shared_ptr<CostmapManager> costmap_manager) 
{
  _costmap_manager = costmap_manager;
  _setMap_core();
}

void AstarPlannerHybrid::_setMap_core() 
{
  ZIYAN_INFO("Configuring AstarPlannerHybrid");

  _costmap = _costmap_manager->getCostmapPtr();

  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

  if (_minimum_turning_radius_global_coords < _costmap->getResolution() * _downsampling_factor) {
    ZIYAN_INFO(
      "Min turning radius cannot be less than the search grid cell resolution!");
    _minimum_turning_radius_global_coords = _costmap->getResolution() * _downsampling_factor;
  }

  // convert to grid coordinates
  if (!_downsample_costmap) {
    _downsampling_factor = 1;
  }
  _search_info.minimum_turning_radius =
    _minimum_turning_radius_global_coords / (_costmap->getResolution() * _downsampling_factor);
  _lookup_table_dim =
    static_cast<float>(_lookup_table_size) /
    static_cast<float>(_costmap->getResolution() * _downsampling_factor);

  // Make sure its a whole number
  _lookup_table_dim = static_cast<float>(static_cast<int>(_lookup_table_dim));

  // Make sure its an odd number
  if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
    ZIYAN_INFO(
      "Even sized heuristic lookup table size set %f, increasing size by 1 to make odd",
      _lookup_table_dim);
    _lookup_table_dim += 1.0;
  }

  // Initialize collision checker
  _collision_checker = GridCollisionChecker(_costmap_manager, _angle_quantizations);
  _collision_checker.setFootprint(
    _costmap_manager->getRobotFootprint(),
    _costmap_manager->getUseRadius(),
    findCircumscribedCost(_costmap_manager)
  );

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<NodeHybrid>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _terminal_checking_interval,
    _max_planning_time,
    _lookup_table_dim,
    _angle_quantizations
  );

  // // Initialize costmap downsampler TODO
  // if (_downsample_costmap && _downsampling_factor > 1) {
  //   _costmap_downsampler = std::make_unique<CostmapDownsampler>();
  //   std::string topic_name = "downsampled_costmap";
  //   _costmap_downsampler->on_configure(
  //     node, _global_frame, topic_name, _costmap, _downsampling_factor);
  // }

  // _raw_plan_publisher = node->create_publisher<nav_msgs::msg::Path>("unsmoothed_plan", 1);

  // if (_debug_visualizations) {
  //   _expansions_publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("expansions", 1);
  //   _planned_footprints_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
  //     "planned_footprints", 1);
  // }

  ZIYAN_INFO(
    "Minimum turning radius: %.2f m , %.2f, _lookup_table_dim: %.2f, _angle_quantizations: %d", 
    _minimum_turning_radius_global_coords, _search_info.minimum_turning_radius, 
    _lookup_table_dim, _angle_quantizations
  );

  ZIYAN_INFO(
    "Configured AstarPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

void AstarPlannerHybrid::cleanup()
{
  ZIYAN_INFO("Cleaning up AstarPlannerHybrid");
  _a_star.reset();
  _smoother.reset();
}

Path AstarPlannerHybrid::createPlan(
  const PoseStamped & start,
  const PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // Downsample costmap, if required
  Costmap2D * costmap = _costmap;
  // if (_costmap_downsampler) {
  //   costmap = _costmap_downsampler->downsample(_downsampling_factor);
  //   _collision_checker.setCostmap(costmap);
  // }

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_manager->getRobotFootprint(),
    _costmap_manager->getUseRadius(),
    findCircumscribedCost(_costmap_manager));
  _a_star->setCollisionChecker(&_collision_checker);


  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    throw std::runtime_error(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  double orientation_bin = getYaw(start.pose.orientation) / _angle_bin_size;
  /* convert [-pi, pi] to [0, 2pi] */
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  unsigned int orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setStart(mx, my, orientation_bin_id);
  ZIYAN_INFO("Start: %d, %d, %d", mx, my, orientation_bin_id);

  // Set goal point, in A* bin search coordinates
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    throw std::runtime_error(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  orientation_bin = getYaw(goal.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setGoal(mx, my, orientation_bin_id);
  ZIYAN_INFO("Goal: %d, %d, %d", mx, my, orientation_bin_id);

  steady_clock::time_point a = steady_clock::now();

  // Setup message
  Path plan;
  PoseStamped pose;

  // Compute plan
  NodeHybrid::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  std::unique_ptr<std::vector<std::tuple<float, float, float>>> expansions = nullptr;
  if (_debug_visualizations) {
    expansions = std::make_unique<std::vector<std::tuple<float, float, float>>>();
  }
  // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
      path, num_iterations,
      _tolerance / static_cast<float>(costmap->getResolution()), cancel_checker, expansions.get()))
  {
    // if (_debug_visualizations) {
    //   geometry_msgs::msg::PoseArray msg;
    //   geometry_msgs::msg::Pose msg_pose;
    //   msg.header.stamp = _clock->now();
    //   msg.header.frame_id = _global_frame;
    //   for (auto & e : *expansions) {
    //     msg_pose.position.x = std::get<0>(e);
    //     msg_pose.position.y = std::get<1>(e);
    //     msg_pose.orientation = getWorldOrientation(std::get<2>(e));
    //     msg.poses.push_back(msg_pose);
    //   }
    //   _expansions_publisher->publish(msg);
    // }

    ZIYAN_INFO("No path found after %d iterations", num_iterations);

    // Note: If the start is blocked only one iteration will occur before failure
    if (num_iterations == 1) {
      // throw nav2_core::StartOccupied("Start occupied");
      throw std::runtime_error("Start occupied");
    }

    if (num_iterations < _a_star->getMaxIterations()) {
      // throw nav2_core::NoValidPathCouldBeFound("no valid path found");
      throw std::runtime_error("no valid path found");
    } else {
      throw std::runtime_error("exceeded maximum iterations");
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  plan.xyt_vec.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    plan.poses.push_back(pose);
    plan.xyt_vec.push_back(XYT{path[i].x, path[i].y, path[i].theta});
  }

  // Publish raw path for debug
  // if (_raw_plan_publisher->get_subscription_count() > 0) {
  //   _raw_plan_publisher->publish(plan);
  // }

  // if (_debug_visualizations) {
  //   // Publish expansions for debug
  //   geometry_msgs::msg::PoseArray msg;
  //   geometry_msgs::msg::Pose msg_pose;
  //   msg.header.stamp = _clock->now();
  //   msg.header.frame_id = _global_frame;
  //   for (auto & e : *expansions) {
  //     msg_pose.position.x = std::get<0>(e);
  //     msg_pose.position.y = std::get<1>(e);
  //     msg_pose.orientation = getWorldOrientation(std::get<2>(e));
  //     msg.poses.push_back(msg_pose);
  //   }
  //   _expansions_publisher->publish(msg);

  //   // plot footprint path planned for debug
  //   if (_planned_footprints_publisher->get_subscription_count() > 0) {
  //     auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  //     for (size_t i = 0; i < plan.poses.size(); i++) {
  //       const std::vector<geometry_msgs::msg::Point> edge =
  //         transformFootprintToEdges(plan.poses[i].pose, _costmap_ros->getRobotFootprint());
  //       marker_array->markers.push_back(createMarker(edge, i, _global_frame, _clock->now()));
  //     }

  //     if (marker_array->markers.empty()) {
  //       visualization_msgs::msg::Marker clear_all_marker;
  //       clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  //       marker_array->markers.push_back(clear_all_marker);
  //     }
  //     _planned_footprints_publisher->publish(std::move(marker_array));
  //   }
  // }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  ZIYAN_INFO("BENCHMARK_TESTING: It took %f milliseconds with %i iterations.", time_span.count() * 1000, num_iterations);
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  ZIYAN_INFO("BENCHMARK_TESTING: It took %f milliseconds to smooth path.", time_span2.count() * 1000);
#endif

  return plan;
}

Path AstarPlannerHybrid::createPlan(
  const XYT & start,
  const XYT & goal,
  std::function<bool()> cancel_checker)
{
  PoseStamped start_pose, goal_pose;
  start_pose.pose.position.x = start.x;
  start_pose.pose.position.y = start.y;
  start_pose.pose.orientation = yawToQuaternion(start.t);
  goal_pose.pose.position.x = goal.x;
  goal_pose.pose.position.y = goal.y;
  goal_pose.pose.orientation = yawToQuaternion(goal.t);
  return createPlan(start_pose, goal_pose, cancel_checker);
}

}