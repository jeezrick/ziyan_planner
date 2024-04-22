#include <string>
#include <memory>
#include <vector>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <assert.h>

#include "pathplanner/smac_planner_2d.hpp"
#include "pathplanner/logger.hpp"


#define BENCHMARK_TESTING

namespace ziyan_planner 
{

using namespace std::chrono;  // NOLINT

AstarPlanner2D::AstarPlanner2D(
  const ziyan_planner::Info::WeakPtr & parent) : AstarPlanner(parent), 
_a_star(nullptr),
_collision_checker(nullptr, 1),
_smoother(nullptr),
// _costmap_downsampler(nullptr)
_costmap(nullptr)
{
   _node = parent;
  auto node = parent.lock(); // get SharedPtr

  ZIYAN_INFO("Configuring of type AstarPlanner2D");

  // General planner params
  _tolerance = node->planner2d_params.tolerance;
  _downsample_costmap = node->planner2d_params.downsample_costmap;
  _downsampling_factor = node->planner2d_params.downsampling_factor;
  _search_info.cost_penalty = node->planner2d_params.cost_travel_multiplier;
  _allow_unknown = node->planner2d_params.allow_unknown;
  _max_iterations = node->planner2d_params.max_iterations;
  _max_on_approach_iterations = node->planner2d_params.max_on_approach_iterations;
  _terminal_checking_interval = node->planner2d_params.terminal_checking_interval;
  _use_final_approach_orientation = node->planner2d_params.use_final_approach_orientation;
  _max_planning_time = node->planner2d_params.max_planning_time;

  _motion_model = MotionModel::TWOD;

  if (_max_on_approach_iterations <= 0) {
    ZIYAN_INFO("On approach iteration selected as <= 0, "
      "disabling tolerance and on approach iterations.");
    _max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (_max_iterations <= 0) {
    ZIYAN_INFO("maximum iteration selected as <= 0, "
      "disabling maximum iterations.");
    _max_iterations = std::numeric_limits<int>::max();
  }

  // Initialize A* template
  _a_star = std::make_unique<AStarAlgorithm<Node2D>>(_motion_model, _search_info);
  _a_star->initialize(
    _allow_unknown,
    _max_iterations,
    _max_on_approach_iterations,
    _terminal_checking_interval,
    _max_planning_time,
    0.0 /*unused for 2D*/,
    1.0 /*unused for 2D*/
  );

  // Initialize path smoother
  SmootherParams params;
  params.get(node);
  params.holonomic_ = true;  // So smoother will treat this as a grid search
  _smoother = std::make_unique<Smoother>(params);
  _smoother->initialize(1e-50 /*No valid minimum turning radius for 2D*/);
}

AstarPlanner2D::~AstarPlanner2D() {
  ZIYAN_INFO("Destroying plugin of type AstarPlanner2D");
}

void AstarPlanner2D::setMap(
  std::shared_ptr<ziyan_costmap::CostmapManager> costmap_ziyan) 
{
  _costmap = costmap_ziyan->getCostmapPtr();
  _global_frame = costmap_ziyan->getGlobalFrameID();

  // Initialize collision checker
  _collision_checker = ziyan_costmap::GridCollisionChecker(costmap_ziyan, 1 /*for 2D, most be 1*/);
  _collision_checker.setFootprint(
    costmap_ziyan->getRobotFootprint(),
    true /*for 2D, most use radius*/,
    0.0 /*for 2D cost at inscribed isn't relevent*/
  );

  // Initialize costmap downsampler
  // if (_downsample_costmap && _downsampling_factor > 1) {
  //   std::string topic_name = "downsampled_costmap";
  //   _costmap_downsampler = std::make_unique<CostmapDownsampler>();
  //   _costmap_downsampler->on_configure(
  //     node, _global_frame, topic_name, _costmap, _downsampling_factor);
  // }

  ZIYAN_INFO(
    "Configured plugin of type AstarPlanner2D with "
    "tolerance %.2f, maximum iterations %i, "
    "downsampling_factor %d, "
    "max on approach iterations %i, and %s.",
    _tolerance, _max_iterations, _downsampling_factor, _max_on_approach_iterations,
    _allow_unknown ? "allowing unknown traversal" : "not allowing unknown traversal"
  );
}

void AstarPlanner2D::cleanup()
{
  ZIYAN_INFO("Cleaning up plugin of type AstarPlanner2D");
  _a_star.reset();
  _smoother.reset();
}

Path AstarPlanner2D::createPlan(
  const ziyan_planner::PoseStamped & start,
  const ziyan_planner::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  steady_clock::time_point a = steady_clock::now();

  // Downsample costmap, if required
  ziyan_costmap::Costmap2D* costmap = _costmap;
  // if (_costmap_downsampler) {
  //   costmap = _costmap_downsampler->downsample(_downsampling_factor);
  //   _collision_checker.setCostmap(costmap);
  // }

  // Set collision checker and costmap information
  _a_star->setCollisionChecker(&_collision_checker);

  // Set starting point
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) 
  {
    throw std::runtime_error(
      "Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
      std::to_string(start.pose.position.y) + ") was outside bounds");
  }
  _a_star->setStart(mx_start, my_start, 0);
  ZIYAN_INFO("Start: %d, %d", mx_start, my_start);

  // Set goal point
  if (!costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) 
  {
    throw std::runtime_error(
      "Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
      std::to_string(goal.pose.position.y) + ") was outside bounds");
  }
  _a_star->setGoal(mx_goal, my_goal, 0);
  ZIYAN_INFO("Goal: %d, %d", mx_goal, my_goal);

  // Setup message
  Path plan;
  plan.header.stamp = 1; // todo
  plan.header.frame_id = _global_frame;
  PoseStamped pose;
  pose.header = plan.header;

  // Corner case of start and goal beeing on the same cell
  if (mx_start == mx_goal && my_start == my_goal) {
    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !_use_final_approach_orientation) {
      pose.pose.orientation = goal.pose.orientation;
    }
    plan.poses.push_back(pose);
    return plan;
  }

  // Compute plan
  Node2D::CoordinateVector path;
  int num_iterations = 0;
  // Note: All exceptions thrown are handled by the planner server and returned to the action
  if (!_a_star->createPath(
    path, 
    num_iterations,
    _tolerance / static_cast<float>(costmap->getResolution()), 
    cancel_checker))
  {
    // Note: If the start is blocked only one iteration will occur before failure
    if (num_iterations == 1) {
      throw std::runtime_error("Start occupied");
    }

    if (num_iterations < _a_star->getMaxIterations()) {
      throw std::runtime_error("no valid path found");
    } else {
      throw std::runtime_error("exceeded maximum iterations");
    }
  }

  // Convert to world coordinates
  plan.poses.reserve(path.size());
  plan.path.reserve(path.size());
  for (int i = path.size() - 1; i >= 0; --i) {
    Entry map_pos(path[i].x, path[i].y);
    pose.pose = getWorldCoords(path[i].x, path[i].y, costmap);
    plan.path.push_back(map_pos);
    plan.poses.push_back(pose);
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());

#ifdef BENCHMARK_TESTING
  ZIYAN_INFO("BENCHMARK_TESTING: It took %f milliseconds with %i iterations.", time_span.count() * 1000, num_iterations);
#endif

  // Smooth plan
  _smoother->smooth(plan, costmap, time_remaining);

  // If use_final_approach_orientation=true, interpolate the last pose orientation from the
  // previous pose to set the orientation to the 'final approach' orientation of the robot so
  // it does not rotate.
  // And deal with corner case of plan of length 1
  // If use_final_approach_orientation=false (default), override last pose orientation to match goal
  size_t plan_size = plan.poses.size();
  if (_use_final_approach_orientation) {
    if (plan_size == 1) {
      plan.poses.back().pose.orientation = start.pose.orientation;
    } else if (plan_size > 1) {
      double dx, dy, theta;
      auto last_pose = plan.poses.back().pose.position;
      auto approach_pose = plan.poses[plan_size - 2].pose.position;
      dx = last_pose.x - approach_pose.x;
      dy = last_pose.y - approach_pose.y;
      theta = atan2(dy, dx);
      plan.poses.back().pose.orientation = orientationAroundZAxis(theta);
    }
  } else if (plan_size > 0) {
    plan.poses.back().pose.orientation = goal.pose.orientation;
  }

  return plan;
}

}  
