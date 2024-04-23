// Copyright (c) 2020, Samsung Research America
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

#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_

#include <memory>
#include <vector>
#include <string>

#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_smac_planner/utils.hpp"
#include "nav2_smac_planner/costmap_downsampler.hpp"
#include "nav2_smac_planner/global_planner.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"

#include "ziyan_io/ziyan_io.hpp"
#include "ziyan_io/costmap_2d_ziyan.hpp"


namespace nav2_smac_planner
{

class SmacPlannerHybrid : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlannerHybrid();

  /**
   * @brief destructor
   */
  ~SmacPlannerHybrid();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const ZiYan_IO::Info::WeakPtr & parent,
    std::string name, 
    std::shared_ptr<ZiYan_IO::Costmap2DZiYan> costmap_ziyan) override; 

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @param cancel_checker Function to check if the action has been canceled
   * @return nav2_msgs::Path of the generated path
   */
  ZiYan_IO::Path createPlan(
    const ZiYan_IO::PoseStamped & start,
    const ZiYan_IO::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  std::unique_ptr<AStarAlgorithm<NodeHybrid>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;

  nav2_costmap_2d::Costmap2D * _costmap;
  std::shared_ptr<ZiYan_IO::Costmap2DZiYan> _costmap_ziyan; 
  std::unique_ptr<CostmapDownsampler> _costmap_downsampler;
  std::string _global_frame, _name;
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
  std::string _motion_model_for_search;
  MotionModel _motion_model;

  // std::mutex _mutex;
  ZiYan_IO::Info::WeakPtr _node;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_HYBRID_HPP_
