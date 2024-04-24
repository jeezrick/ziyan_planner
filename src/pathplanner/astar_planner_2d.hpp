#ifndef ZIYAN_PLANNER__PLANNER_2D_HPP_
#define ZIYAN_PLANNER__PLANNER_2D_HPP_

#include <memory>
#include <vector>
#include <string>

#include "pathplanner/global_planner.hpp"
#include "pathplanner/a_star.hpp"
#include "pathplanner/smoother.hpp"
#include "pathplanner/utils.hpp"
#include "pathplanner/costmap_downsampler.hpp"
#include "pathplanner/costmap_2d.hpp"
#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/planner_io.hpp"


namespace ziyan_planner
{

class AstarPlanner2D : public AstarPlanner
{
public:
  AstarPlanner2D(const Info::SharedPtr & parent);
  AstarPlanner2D(const std::string & cfg_path);

  ~AstarPlanner2D() override;

  void setMap(std::shared_ptr<CostmapManager> costmap_manager) override;

  void setMap(
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
  double origin_x, double origin_y, uint8_t* data_ptr) override;

  Path createPlan(
    const PoseStamped & start,
    const PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  Path createPlan(
    const XYT& start,
    const XYT& goal,
    std::function<bool()> cancel_checker) override;

  void cleanup() override;

protected:
  void init();
  void _setMap_core();

  void setStartFrom(unsigned int & mx, unsigned int & my, const XYT & start, Costmap2D* costmap);
  void setGoalFrom(unsigned int & mx, unsigned int & my, const XYT & goal, Costmap2D* costmap);

  void setStartFrom(unsigned int & mx, unsigned int & my, const PoseStamped & start, Costmap2D* costmap);
  void setGoalFrom(unsigned int & mx, unsigned int & my, const PoseStamped & goal, Costmap2D* costmap);

  Path setPathWithStartOrGoal(
    const XYT & start, const XYT & goal, bool use_final_approach_orientation);

  Path setPathWithStartOrGoal(
    const PoseStamped & start, const PoseStamped & goal, bool use_final_approach_orientation);


  std::unique_ptr<AStarAlgorithm<Node2D>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  Costmap2D * _costmap;
  std::shared_ptr<CostmapManager> _costmap_manager_ptr;
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

  std::string _global_frame;

  Info::SharedPtr _node;
};

}

#endif
