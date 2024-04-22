#ifndef ZIYAN_PLANNER__ASTAR_PLANNER_HYBRID_HPP_
#define ZIYAN_PLANNER__ASTAR_PLANNER_HYBRID_HPP_

#include <memory>
#include <vector>
#include <string>

#include "pathplanner/a_star.hpp"
#include "pathplanner/smoother.hpp"
#include "pathplanner/utils.hpp"
#include "pathplanner/costmap_downsampler.hpp"
#include "pathplanner/global_planner.hpp"
#include "pathplanner/costmap_2d.hpp"
#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/ziyan_io.hpp"


namespace ziyan_planner
{

class AstarPlannerHybrid : public AstarPlanner
{
public:
  AstarPlannerHybrid(const ziyan_planner::Info::WeakPtr & parent);
  ~AstarPlannerHybrid() override;
  void setMap(
    std::shared_ptr<ziyan_costmap::CostmapManager> costmap_ziyan) override; 

  Path createPlan(
    const PoseStamped & start,
    const PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  void cleanup() override;

protected:
  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  ziyan_costmap::GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;

  ziyan_costmap::Costmap2D * _costmap;
  std::shared_ptr<ziyan_costmap::CostmapManager> _costmap_ziyan; 
  std::unique_ptr<ziyan_costmap::CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame;
  float _lookup_table_dim;
  float _tolerance;
  bool _downsample_costmap;
  int _downsampling_factor;
  double _angle_bin_size;
  unsigned int _angle_quantizations;
  bool _allow_unknown;
  int _max_iterations;
  int _max_on_approach_iterations;
  int _terminal_checking_interval;
  SearchInfo _search_info;
  double _max_planning_time;
  double _lookup_table_size;
  double _minimum_turning_radius_global_coords;
  bool _debug_visualizations;

  float analytic_expansion_max_length_m;

  std::string _motion_model_for_search;
  MotionModel _motion_model;

  ziyan_planner::Info::WeakPtr _node;
};

}  

#endif
