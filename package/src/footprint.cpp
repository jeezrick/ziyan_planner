#include <algorithm>
#include <limits>
#include <iostream>

#include "pathplanner/footprint.hpp"
#include "pathplanner/costmap_math.hpp"

namespace ziyan_planner
{

std::pair<double, double> calculateMinAndMaxDistances(
  const std::vector<Point> & footprint)
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

void padFootprint(std::vector<Point> & footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++) {
    Point & pt = footprint[i];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

std::vector<Point> makeFootprintFromRadius(double radius)
{
  std::vector<Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  Point pt;
  for (int i = 0; i < N; ++i) {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}

}  
