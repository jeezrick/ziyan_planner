#ifndef ZIYAN_COSTMAP__FOOTPRINT_HPP_
#define ZIYAN_COSTMAP__FOOTPRINT_HPP_

#include <string>
#include <vector>
#include <utility>

#include "pathplanner/ziyan_io.hpp"

namespace ziyan_costmap
{

/**
 * @brief Calculate the extreme distances for the footprint
 *
 * @param footprint The footprint to examine
 * @param min_dist Output parameter of the minimum distance
 * @param max_dist Output parameter of the maximum distance
 */
std::pair<double, double> calculateMinAndMaxDistances(
  const std::vector<ziyan_planner::Point> & footprint);

/**
 * @brief Convert Point32 to Point
 */
ziyan_planner::Point toPoint(ziyan_planner::Point32 pt);

/**
 * @brief Convert Point to Point32
 */
ziyan_planner::Point32 toPoint32(ziyan_planner::Point pt);

/**
 * @brief Convert vector of Points to Polygon msg
 */
ziyan_planner::Polygon toPolygon(std::vector<ziyan_planner::Point> pts);

/**
 * @brief Convert Polygon msg to vector of Points.
 */
std::vector<ziyan_planner::Point> toPointVector(
  ziyan_planner::Polygon::SharedPtr polygon);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(
  double x, double y, double theta,
  const std::vector<ziyan_planner::Point> & footprint_spec,
  std::vector<ziyan_planner::Point> & oriented_footprint);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(
  double x, double y, double theta,
  const std::vector<ziyan_planner::Point> & footprint_spec,
  ziyan_planner::PolygonStamped & oriented_footprint);

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
void padFootprint(std::vector<ziyan_planner::Point> & footprint, double padding);

/**
 * @brief Create a circular footprint from a given radius
 */
std::vector<ziyan_planner::Point> makeFootprintFromRadius(double radius);

}  

#endif  
