#ifndef ZIYAN_PLANNER__ASTAR_PLANNER_HPP_
#define ZIYAN_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <functional>

#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/ziyan_io.hpp"

namespace ziyan_planner
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
class AstarPlanner
{
public:

  AstarPlanner(const ziyan_planner::Info::WeakPtr & parent){};

  virtual ~AstarPlanner() {}

  virtual void setMap(
    std::shared_ptr<ziyan_costmap::CostmapManager> costmap_ziyan
  ) = 0;

  virtual void cleanup() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @param cancel_checker Function to check if the action has been canceled
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual ziyan_planner::Path createPlan(
    const ziyan_planner::PoseStamped & start,
    const ziyan_planner::PoseStamped & goal,
    std::function<bool()> cancel_checker) = 0;
};

}

#endif
