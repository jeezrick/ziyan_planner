// Copyright (c) 2019 Samsung Research America
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
// limitations under the License.

#ifndef NAV2_CORE__GLOBAL_PLANNER_HPP_
#define NAV2_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>
#include <functional>

#include "ziyan_io/ziyan_io.hpp"
#include "ziyan_io/costmap_2d_ziyan.hpp"

namespace nav2_core
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  /**
   * @brief Virtual destructor
   */
  virtual ~GlobalPlanner() {}

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(
    const ZiYan_IO::Info::WeakPtr & parent,
    std::string name,
    std::shared_ptr<ZiYan_IO::Costmap2DZiYan> costmap_ziyan) = 0;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @param cancel_checker Function to check if the action has been canceled
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual ZiYan_IO::Path createPlan(
    const ZiYan_IO::PoseStamped & start,
    const ZiYan_IO::PoseStamped & goal,
    std::function<bool()> cancel_checker) = 0;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__GLOBAL_PLANNER_HPP_
