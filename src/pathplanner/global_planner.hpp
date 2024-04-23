#ifndef ZIYAN_PLANNER__ASTAR_PLANNER_HPP_
#define ZIYAN_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <functional>

#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/planner_io.hpp"


namespace ziyan_planner
{

/**
 * @class GlobalPlanner
 * @brief Abstract interface for global planners to adhere to with pluginlib
 */
class AstarPlanner
{
public:

  AstarPlanner(const Info::WeakPtr & parent){};

  virtual ~AstarPlanner() {}

  virtual void setMap(
    std::shared_ptr<CostmapManager> costmap_manager
  ) = 0;

  virtual void cleanup() = 0;


  virtual Path createPlan(
    const PoseStamped & start,
    const PoseStamped & goal,
    std::function<bool()> cancel_checker) = 0;

  virtual Path createPlan(
    const XYT& start,
    const XYT& goal,
    std::function<bool()> cancel_checker) = 0;

};

}

#endif
