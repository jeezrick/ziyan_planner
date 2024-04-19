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

#ifndef NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
#define NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_

#include <memory>
#include <vector>
#include <string>
#include <mutex>

#include "planner/global_planner.hpp"
#include "planner/a_star.hpp"
#include "planner/smoother.hpp"
#include "planner/utils.hpp"
// #include "nav2_smac_planner/costmap_downsampler.hpp"
#include "map/costmap_2d.hpp"
#include "map/costmap_2d_ziyan.hpp"
#include "ziyan_io/ziyan_io.hpp"


namespace nav2_smac_planner
{

class SmacPlanner2D : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief constructor
   */
  SmacPlanner2D();

  /**
   * @brief destructor
   */
  ~SmacPlanner2D();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param costmap_ros Costmap2DROS object
   */
  void configure(
    const ZiYan_IO::Info::WeakPtr & parent,
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
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @param cancel_checker Function to check if the action has been canceled
   * @return      The sequence of poses to get from start to goal, if any
   */
  ZiYan_IO::Path createPlan(
    const ZiYan_IO::PoseStamped & start,
    const ZiYan_IO::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  std::unique_ptr<AStarAlgorithm<Node2D>> _a_star;
  GridCollisionChecker _collision_checker;
  std::unique_ptr<Smoother> _smoother;
  nav2_costmap_2d::Costmap2D * _costmap;
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
  ZiYan_IO::Info::WeakPtr _node;
  std::string _global_frame;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__SMAC_PLANNER_2D_HPP_
