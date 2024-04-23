// Copyright (c) 2020, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "ziyan_io/logger.hpp"

#define BENCHMARK_TESTING
#define PLANNER_HYBRID_DEBUG_ 

namespace nav2_smac_planner
{

using namespace std::chrono;  // NOLINT

SmacPlannerHybrid::SmacPlannerHybrid()
: _a_star(nullptr),
  _collision_checker(nullptr, 1, nullptr),
  _smoother(nullptr),
  _costmap(nullptr),
  // _costmap_downsampler(nullptr)
  _costmap_ziyan(nullptr)
{
}

SmacPlannerHybrid::~SmacPlannerHybrid()
{
  ZIYAN_INFO(
    "Destroying plugin %s of type SmacPlannerHybrid",
    _name.c_str());
}

void SmacPlannerHybrid::configure(
  const ZiYan_IO::Info::WeakPtr & parent,
  std::string name, 
  std::shared_ptr<ZiYan_IO::Costmap2DZiYan> costmap_ziyan) 
{
  ZIYAN_INFO("Configuring SmacPlannerHybrid");
  _name = name;

  _node = parent;
  auto node = &((parent.lock()) -> plannerhybrid_params);

  _costmap = costmap_ziyan->getCostmap();
  _costmap_ziyan = costmap_ziyan;
  _global_frame = _costmap_ziyan->getGlobalFrameID();

  int angle_quantizations;
  double analytic_expansion_max_length_m;
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
  _search_info.analytic_expansion_max_length =
    analytic_expansion_max_length_m / _costmap->getResolution();

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
  _collision_checker = GridCollisionChecker(_costmap_ziyan, _angle_quantizations, parent.lock());
  _collision_checker.setFootprint(
    _costmap_ziyan->getRobotFootprint(),
    _costmap_ziyan->getUseRadius(),
    findCircumscribedCost(_costmap_ziyan)
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

  // Initialize path smoother
  if (smooth_path) {
    SmootherParams params;
    params.get(parent.lock());
    _smoother = std::make_unique<Smoother>(params);
    _smoother->initialize(_minimum_turning_radius_global_coords);
  }

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

  ZIYAN_INFO( "Minimum turning radius: %.2f, _lookup_table_dim: %.2f", _search_info.minimum_turning_radius, _lookup_table_dim);
  ZIYAN_INFO(
    "Configured SmacPlannerHybrid with "
    "maximum iterations %i, max on approach iterations %i, and %s. Tolerance %.2f."
    "Using motion model: %s.",
    _max_iterations, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal",
    _tolerance, toString(_motion_model).c_str());
}

void SmacPlannerHybrid::activate()
{
  ZIYAN_INFO("Activating SmacPlannerHybrid");
}

void SmacPlannerHybrid::deactivate()
{
  ZIYAN_INFO("Deactivating SmacPlannerHybrid");
}

void SmacPlannerHybrid::cleanup()
{
  ZIYAN_INFO("Cleaning up SmacPlannerHybrid");
  _a_star.reset();
  _smoother.reset();
}

ZiYan_IO::Path SmacPlannerHybrid::createPlan(
  const ZiYan_IO::PoseStamped & start,
  const ZiYan_IO::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  // std::lock_guard<std::mutex> lock_reinit(_mutex);
  steady_clock::time_point a = steady_clock::now();

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  nav2_costmap_2d::Costmap2D * costmap = _costmap;
  // if (_costmap_downsampler) {
  //   costmap = _costmap_downsampler->downsample(_downsampling_factor);
  //   _collision_checker.setCostmap(costmap);
  // }

  // Set collision checker and costmap information
  _collision_checker.setFootprint(
    _costmap_ziyan->getRobotFootprint(),
    _costmap_ziyan->getUseRadius(),
    findCircumscribedCost(_costmap_ziyan));
  _a_star->setCollisionChecker(&_collision_checker);


  // Set starting point, in A* bin search coordinates
  unsigned int mx, my;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    throw std::runtime_error(
            "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was outside bounds");
  }

  double orientation_bin = ZiYan_IO::getYaw(start.pose.orientation) / _angle_bin_size;
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

  #ifdef PLANNER_HYBRID_DEBUG_
  ZIYAN_INFO("Start: %d, %d, %d", mx, my, orientation_bin_id);
  #endif

  // Set goal point, in A* bin search coordinates
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    throw std::runtime_error(
            "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  orientation_bin = ZiYan_IO::getYaw(goal.pose.orientation) / _angle_bin_size;
  while (orientation_bin < 0.0) {
    orientation_bin += static_cast<float>(_angle_quantizations);
  }
  // This is needed to handle precision issues
  if (orientation_bin >= static_cast<float>(_angle_quantizations)) {
    orientation_bin -= static_cast<float>(_angle_quantizations);
  }
  orientation_bin_id = static_cast<unsigned int>(floor(orientation_bin));
  _a_star->setGoal(mx, my, orientation_bin_id);
  #ifdef PLANNER_HYBRID_DEBUG_
  ZIYAN_INFO("Goal: %d, %d, %d", mx, my, orientation_bin_id);
  #endif

  // Setup message
  ZiYan_IO::Path plan;
  plan.header.stamp = 1; // TODO
  plan.header.frame_id = _global_frame;
  ZiYan_IO::PoseStamped pose;
  pose.header = plan.header;

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
  plan.path.reserve(path.size());
  ZIYAN_INFO("path.size(): %d", path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    ZiYan_IO::Entry map_pos(path[i].x, path[i].y);
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    plan.path.push_back(map_pos);
    plan.poses.push_back(pose);
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
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  // Smooth plan
  if (_smoother && num_iterations > 1) {
    _smoother->smooth(plan, costmap, time_remaining);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point c = steady_clock::now();
  duration<double> time_span2 = duration_cast<duration<double>>(c - b);
  std::cout << "It took " << time_span2.count() * 1000 <<
    " milliseconds to smooth path." << std::endl;
#endif

  return plan;
}

}  // namespace nav2_smac_planner
