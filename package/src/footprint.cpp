#include <algorithm>
#include <limits>
#include <iostream>

#include "pathplanner/footprint.hpp"
#include "pathplanner/costmap_math.hpp"

namespace ziyan_costmap
{

std::pair<double, double> calculateMinAndMaxDistances(
  const std::vector<ziyan_planner::Point> & footprint)
{
  double min_dist = std::numeric_limits<double>::max();
  double max_dist = 0.0;

  if (footprint.size() <= 2) {
    return std::pair<double, double>(min_dist, max_dist);
  }

  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    double edge_dist = distanceToLine(
      0.0, 0.0, footprint[i].x, footprint[i].y,
      footprint[i + 1].x, footprint[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(
    0.0, 0.0, footprint.back().x, footprint.back().y,
    footprint.front().x, footprint.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));

  return std::pair<double, double>(min_dist, max_dist);
}

ziyan_planner::Point32 toPoint32(ziyan_planner::Point pt)
{
  ziyan_planner::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

ziyan_planner::Point toPoint(ziyan_planner::Point32 pt)
{
  ziyan_planner::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

ziyan_planner::Polygon toPolygon(std::vector<ziyan_planner::Point> pts)
{
  ziyan_planner::Polygon polygon;
  for (unsigned int i = 0; i < pts.size(); i++) {
    polygon.points.push_back(toPoint32(pts[i]));
  }
  return polygon;
}

std::vector<ziyan_planner::Point> toPointVector(ziyan_planner::Polygon::SharedPtr polygon)
{
  std::vector<ziyan_planner::Point> pts;
  for (unsigned int i = 0; i < polygon->points.size(); i++) {
    pts.push_back(toPoint(polygon->points[i]));
  }
  return pts;
}

void transformFootprint(
  double x, double y, double theta,
  const std::vector<ziyan_planner::Point> & footprint_spec,
  std::vector<ziyan_planner::Point> & oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.resize(footprint_spec.size());
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    double new_x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    double new_y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    ziyan_planner::Point & new_pt = oriented_footprint[i];
    new_pt.x = new_x;
    new_pt.y = new_y;
  }
}

void transformFootprint(
  double x, double y, double theta,
  const std::vector<ziyan_planner::Point> & footprint_spec,
  ziyan_planner::PolygonStamped & oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.polygon.points.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    ziyan_planner::Point32 new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

void padFootprint(std::vector<ziyan_planner::Point> & footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++) {
    ziyan_planner::Point & pt = footprint[i];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}


std::vector<ziyan_planner::Point> makeFootprintFromRadius(double radius)
{
  std::vector<ziyan_planner::Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  ziyan_planner::Point pt;
  for (int i = 0; i < N; ++i) {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}

}  
