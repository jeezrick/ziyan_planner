#include "pathplanner/collision_checker.hpp"
#include "pathplanner/logger.hpp"

// #define COLLISION_CHECKER_DEBUG_

namespace ziyan_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<CostmapManager> costmap_manager_ptr,
  unsigned int num_quantizations
) {
  if (costmap_manager_ptr) {
    costmap_manager_ptr_ = costmap_manager_ptr;
    costmap_ptr_ = costmap_manager_ptr_->getCostmapSharePtr();
  } else {
    // throw std::runtime_error("costmap_manager_ptr_ is nullptr");
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

void GridCollisionChecker::setFootprint(
  const Footprint & footprint,
  const bool & radius,
  const double & possible_collision_cost)
{
  #ifdef COLLISION_CHECKER_DEBUG_
  ZIYAN_INFO("is_raius: %s, possible_collision_cost: %f", radius ? "true" : "false", possible_collision_cost);
  #endif

  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  } else {
    throw std::runtime_error("Footprint collision checker is not implemented for non-circular footprints");
  }
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_ptr_->getSizeInCellsX(), x) ||
    outsideRange(costmap_ptr_->getSizeInCellsY(), y)) {
    return false;
  }

  // Assumes setFootprint already set
  double wx, wy;
  costmap_ptr_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);

  if (!footprint_is_radius_) {
    throw std::runtime_error("Footprint collision checker is not implemented for non-circular footprints");
  } else {
    // if radius, then we can check the center of the cost assuming inflation is used
    footprint_cost_ = static_cast<float>(costmap_ptr_->getCost(
        static_cast<unsigned int>(x + 0.5), static_cast<unsigned int>(y + 0.5)));
 
    #ifdef COLLISION_CHECKER_DEBUG_
    ZIYAN_INFO("footprint_is_radius_: %s, footprint_cost: %f", 
      (footprint_is_radius_ ? "true" : "false"),  footprint_cost_
    );
    #endif

    if (footprint_cost_ == f_UNKNOWN && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return footprint_cost_ >= f_INSCRIBED;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  footprint_cost_ = costmap_ptr_->getCost(i);
  if (footprint_cost_ == f_UNKNOWN && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost_ >= f_INSCRIBED;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(footprint_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

} 