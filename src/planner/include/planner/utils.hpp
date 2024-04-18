// Copyright (c) 2021, Samsung Research America
// Copyright (c) 2023, Open Navigation LLC
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

#ifndef NAV2_SMAC_PLANNER__UTILS_HPP_
#define NAV2_SMAC_PLANNER__UTILS_HPP_

#include <vector>
#include <memory>
#include <string>

// #include "nlohmann/json.hpp"
#include "Eigen/Core"
#include "planner/types.hpp"
#include "map/costmap_2d_ziyan.hpp"
#include "ziyan_io/ziyan_io.hpp"
#include "ziyan_io/logger.hpp"

namespace nav2_smac_planner
{

/**
* @brief Create an Eigen Vector2D of world poses from continuous map coords
* @param mx float of map X coordinate
* @param my float of map Y coordinate
* @param costmap Costmap pointer
* @return Eigen::Vector2d eigen vector of the generated path
*/
inline ZiYan_IO::Pose getWorldCoords(
  const float & mx, const float & my, const nav2_costmap_2d::Costmap2D * costmap)
{
  ZiYan_IO::Pose msg;
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
inline ZiYan_IO::Quaternion getWorldOrientation(
  const float & theta)
{
  // theta is in radians already
  ZiYan_IO::Quaternion q;
  q.setEuler(0.0, 0.0, theta);
  return q;
}

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline ZiYan_IO::Quaternion orientationAroundZAxis(double angle)
{
  ZiYan_IO::Quaternion q;
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
inline double findCircumscribedCost(std::shared_ptr<ZiYan_IO::Costmap2DZiYan> costmap)
{
  double result = -1.0;
  // std::vector<std::shared_ptr<nav2_costmap_2d::Layer>>::iterator layer;

  // check if the costmap has an inflation layer
  const auto inflation_layer = costmap->getInflationLayer();
  if (inflation_layer != nullptr) {
    double circum_radius = costmap->getInflationLayer()->getCircumscribedRadius();
    double resolution = costmap->getCostmap()->getResolution();
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

// /**
//  * @brief convert json to lattice metadata
//  * @param[in] json json object
//  * @param[out] lattice meta data
//  */
// inline void fromJsonToMetaData(const nlohmann::json & json, LatticeMetadata & lattice_metadata)
// {
//   json.at("turning_radius").get_to(lattice_metadata.min_turning_radius);
//   json.at("grid_resolution").get_to(lattice_metadata.grid_resolution);
//   json.at("num_of_headings").get_to(lattice_metadata.number_of_headings);
//   json.at("heading_angles").get_to(lattice_metadata.heading_angles);
//   json.at("number_of_trajectories").get_to(lattice_metadata.number_of_trajectories);
//   json.at("motion_model").get_to(lattice_metadata.motion_model);
// }

// /**
//  * @brief convert json to pose
//  * @param[in] json json object
//  * @param[out] pose
//  */
// inline void fromJsonToPose(const nlohmann::json & json, MotionPose & pose)
// {
//   pose._x = json[0];
//   pose._y = json[1];
//   pose._theta = json[2];
// }

// /**
//  * @brief convert json to motion primitive
//  * @param[in] json json object
//  * @param[out] motion primitive
//  */
// inline void fromJsonToMotionPrimitive(
//   const nlohmann::json & json, MotionPrimitive & motion_primitive)
// {
//   json.at("trajectory_id").get_to(motion_primitive.trajectory_id);
//   json.at("start_angle_index").get_to(motion_primitive.start_angle);
//   json.at("end_angle_index").get_to(motion_primitive.end_angle);
//   json.at("trajectory_radius").get_to(motion_primitive.turning_radius);
//   json.at("trajectory_length").get_to(motion_primitive.trajectory_length);
//   json.at("arc_length").get_to(motion_primitive.arc_length);
//   json.at("straight_length").get_to(motion_primitive.straight_length);
//   json.at("left_turn").get_to(motion_primitive.left_turn);

//   for (unsigned int i = 0; i < json["poses"].size(); i++) {
//     MotionPose pose;
//     fromJsonToPose(json["poses"][i], pose);
//     motion_primitive.poses.push_back(pose);
//   }
// } TODO json

/**
 * @brief transform footprint into edges
 * @param[in] robot position , orientation and  footprint
 * @param[out] robot footprint edges
 */
// inline std::vector<ZiYan_IO::Point> transformFootprintToEdges(
//   const ZiYan_IO::Pose & pose,
//   const std::vector<ZiYan_IO::Point> & footprint)
// {
//   const double & x = pose.position.x;
//   const double & y = pose.position.y;
//   const double & yaw = tf2::getYaw(pose.orientation);

//   std::vector<ZiYan_IO::Point> out_footprint;
//   out_footprint.resize(2 * footprint.size());
//   for (unsigned int i = 0; i < footprint.size(); i++) {
//     out_footprint[2 * i].x = x + cos(yaw) * footprint[i].x - sin(yaw) * footprint[i].y;
//     out_footprint[2 * i].y = y + sin(yaw) * footprint[i].x + cos(yaw) * footprint[i].y;
//     if (i == 0) {
//       out_footprint.back().x = out_footprint[i].x;
//       out_footprint.back().y = out_footprint[i].y;
//     } else {
//       out_footprint[2 * i - 1].x = out_footprint[2 * i].x;
//       out_footprint[2 * i - 1].y = out_footprint[2 * i].y;
//     }
//   }
//   return out_footprint;
// }

/**
 * @brief initializes marker to visualize shape of linestring
 * @param edge       edge to mark of footprint
 * @param i          marker ID
 * @param frame_id   frame of the marker
 * @param timestamp  timestamp of the marker
 * @return marker populated
 */
// inline visualization_msgs::msg::Marker createMarker(
//   const std::vector<geometry_msgs::msg::Point> edge,
//   unsigned int i, const std::string & frame_id, const rclcpp::Time & timestamp)
// {
//   visualization_msgs::msg::Marker marker;
//   marker.header.frame_id = frame_id;
//   marker.header.stamp = timestamp;
//   marker.frame_locked = false;
//   marker.ns = "planned_footprint";
//   marker.action = visualization_msgs::msg::Marker::ADD;
//   marker.type = visualization_msgs::msg::Marker::LINE_LIST;
//   marker.lifetime = rclcpp::Duration(0, 0);

//   marker.id = i;
//   for (auto & point : edge) {
//     marker.points.push_back(point);
//   }

//   marker.pose.orientation.x = 0.0;
//   marker.pose.orientation.y = 0.0;
//   marker.pose.orientation.z = 0.0;
//   marker.pose.orientation.w = 1.0;
//   marker.scale.x = 0.05;
//   marker.scale.y = 0.05;
//   marker.scale.z = 0.05;
//   marker.color.r = 0.0f;
//   marker.color.g = 0.0f;
//   marker.color.b = 1.0f;
//   marker.color.a = 1.3f;
//   return marker;
// }


}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__UTILS_HPP_
