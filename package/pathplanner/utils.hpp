#ifndef ZIYAN_PLANNER__UTILS_HPP_
#define ZIYAN_PLANNER__UTILS_HPP_

#include <vector>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "pathplanner/types.hpp"
#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/ziyan_io.hpp"
#include "pathplanner/logger.hpp"

namespace ziyan_planner
{

/**
* @brief Create an Eigen Vector2D of world poses from continuous map coords
* @param mx float of map X coordinate
* @param my float of map Y coordinate
* @param costmap Costmap pointer
* @return Eigen::Vector2d eigen vector of the generated path
*/
inline ziyan_planner::Pose getWorldCoords(
  const float & mx, const float & my, const ziyan_costmap::Costmap2D * costmap)
{
  ziyan_planner::Pose msg;
  msg.position.x =
    static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  msg.position.y =
    static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return msg;
}

/**
* @brief Create quaternion from radians
* @param theta continuous bin coordinates angle
* @return quaternion orientation in map frame
*/
inline ziyan_planner::Quaternion getWorldOrientation(
  const float & theta)
{
  // theta is in radians already
  ziyan_planner::Quaternion q;
  q.setEuler(0.0, 0.0, theta);
  return q;
}

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline ziyan_planner::Quaternion orientationAroundZAxis(double angle)
{
  ziyan_planner::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return q;
}

/*!
  * \brief normalize
  *
  * Normalizes the angle to be -M_PI circle to +M_PI circle
  * It takes and returns radians.
  *
  */
static inline double normalize_angle(double angle)
{
  const double result = fmod(angle + M_PI, 2.0*M_PI);
  if(result <= 0.0) return result + M_PI;
  return result - M_PI;
}

/*!
  * \function
  * \brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and ouputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivelent angle to "to".
  */

static inline double shortest_angular_distance(double from, double to)
{
  return normalize_angle(to-from);
}

/**
* @brief Find the min cost of the inflation decay function for which the robot MAY be
* in collision in any orientation
* @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
* @return double circumscribed cost, any higher than this and need to do full footprint collision checking
* since some element of the robot could be in collision
*/
inline double findCircumscribedCost(std::shared_ptr<ziyan_costmap::CostmapManager> costmap)
{
  double result = -1.0;
  // std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>::iterator layer;

  // check if the costmap has an inflation layer
  const auto inflation_layer = costmap->getInflationLayer();
  if (inflation_layer != nullptr) {
    double circum_radius = costmap->getInflationLayer()->getCircumscribedRadius();
    double resolution = costmap->getCostmapSharePtr()->getResolution();
    result = static_cast<double>(inflation_layer->computeCost(circum_radius / resolution));
  } else {
    ZIYAN_INFO(
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times!");
  }

  return result;
}

}

#endif
