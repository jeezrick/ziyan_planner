#ifndef ZIYAN_PLANNER__FOOTPRINT_HPP_
#define ZIYAN_PLANNER__FOOTPRINT_HPP_

#include <string>
#include <vector>
#include <utility>

#include "pathplanner/planner_io.hpp"

namespace ziyan_planner
{

/**
 * @brief Calculate the extreme distances for the footprint
 *
 * @param footprint The footprint to examine
 * @param min_dist Output parameter of the minimum distance
 * @param max_dist Output parameter of the maximum distance
 */
std::pair<double, double> calculateMinAndMaxDistances(
  const std::vector<Point> & footprint);

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
void padFootprint(std::vector<Point> & footprint, double padding);

/**
 * @brief Create a circular footprint from a given radius
 */
std::vector<Point> makeFootprintFromRadius(double radius);

}  

#endif  
