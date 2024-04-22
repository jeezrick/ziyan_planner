#ifndef ZIYAN_PLANNER__PLANNER_2D_HPP_
#define ZIYAN_PLANNER__PLANNER_2D_HPP_

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "pathplanner/global_planner.hpp"
#include "pathplanner/a_star.hpp"
#include "pathplanner/smoother.hpp"
#include "pathplanner/utils.hpp"
#include "pathplanner/costmap_downsampler.hpp"
#include "pathplanner/costmap_2d.hpp"
#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/ziyan_io.hpp"


namespace ziyan_planner
{

class AstarPlanner2D : public AstarPlanner
{
public:
  AstarPlanner2D(const ziyan_planner::Info::WeakPtr & parent);

  ~AstarPlanner2D() override;

  void setMap(std::shared_ptr<ziyan_costmap::CostmapManager> costmap_ziyan) override;

  ziyan_planner::Path createPlan(
    const ziyan_planner::PoseStamped & start,
    const ziyan_planner::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  void cleanup() override;

protected:
  std::unique_ptr<AStarAlgorithm<Node2D>> _a_star;
  ziyan_costmap::GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  ziyan_costmap::Costmap2D * _costmap;
  // std::unique_ptr<CostmapDownsampler> _costmap_downsampler;

  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _max_planning_time;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  bool _use_final_approach_orientation;
  SearchInfo _search_info;
  std::string _motion_model_for_search;
  MotionModel _motion_model;

  ziyan_planner::Info::WeakPtr _node;
  std::string _global_frame;
};

}

#endif
