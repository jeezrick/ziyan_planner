#ifndef ZIYAN_COSTMAP__COLLISION_CHECKER_HPP_
#define ZIYAN_COSTMAP__COLLISION_CHECKER_HPP_

#include <vector>
#include <memory>
#include <algorithm>

#include "pathplanner/constants.hpp"
#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/ziyan_io.hpp"


namespace ziyan_costmap
{

typedef std::vector<ziyan_planner::Point> Footprint;

/**
 * @class nav2_smac_planner::GridCollisionChecker
 * @brief A costmap grid collision checker
 */
class GridCollisionChecker
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::GridCollisionChecker
   * for use when regular bin intervals are appropriate
   * @param costmap The costmap to collision check against
   * @param num_quantizations The number of quantizations to precompute footprint
   * @param node Node to extract clock and logger from
   * orientations for to speed up collision checking
   */
  GridCollisionChecker(
    std::shared_ptr<CostmapManager> costmap,
    unsigned int num_quantizations);

  /**
   * @brief Set the footprint to use with collision checker
   * @param footprint The footprint to collision check against
   * @param radius Whether or not the footprint is a circle and use radius collision checking
   */
  void setFootprint(
    const Footprint & footprint,
    const bool & radius,
    const double & possible_collision_cost);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param x X coordinate of pose to check against
   * @param y Y coordinate of pose to check against
   * @param theta Angle bin number of pose to check against (NOT radians)
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const float & x,
    const float & y,
    const float & theta,
    const bool & traverse_unknown);

  /**
   * @brief Check if in collision with costmap and footprint at pose
   * @param i Index to search collision status of
   * @param traverse_unknown Whether or not to traverse in unknown space
   * @return boolean if in collision or not.
   */
  bool inCollision(
    const unsigned int & i,
    const bool & traverse_unknown);

  /**
   * @brief Get cost at footprint pose in costmap
   * @return the cost at the pose in costmap
   */
  float getCost();

  /**
   * @brief Get the angles of the precomputed footprint orientations
   * @return the ordered vector of angles corresponding to footprints
   */
  std::vector<float> & getPrecomputedAngles()
  {
    return angles_;
  }

  /**
   * @brief Get costmap ros object for inflation layer params
   * @return Costmap ros
   */
  std::shared_ptr<CostmapManager> getCostmapZIYAN() {return costmap_manager_ptr_;}
  Costmap2D* getCostmapPtr() {return costmap_manager_ptr_->getCostmapPtr();}

private:
  bool outsideRange(const unsigned int & max, const float & value);

protected:
  std::shared_ptr<CostmapManager> costmap_manager_ptr_;
  std::shared_ptr<Costmap2D> costmap_ptr_;

  float footprint_cost_;
  bool footprint_is_radius_;
  std::vector<float> angles_;
  float possible_collision_cost_{-1};
};

}  

#endif  
