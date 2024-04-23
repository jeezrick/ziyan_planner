/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include <limits>
#include <map>
#include <vector>
#include <algorithm>
#include <utility>

#include "map/inflation_layer.hpp"
#include "map/costmap_math.hpp"
#include "map/footprint.hpp"
#include "ziyan_io/logger.hpp"

// #define INFLATION_DEBUG

namespace nav2_costmap_2d
{

InflationLayer::InflationLayer()
: inflation_radius_(0),
  inscribed_radius_(0),
  cost_scaling_factor_(0),
  inflate_unknown_(false),
  inflate_around_unknown_(false),
  cell_inflation_radius_(0),
  cached_cell_inflation_radius_(0),
  resolution_(0),
  cache_length_(0),
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max())
{
}

InflationLayer::~InflationLayer()
{
}

void
InflationLayer::initialize(
  std::shared_ptr<Costmap2D> costmap_2d_ptr, 
  const ZiYan_IO::Info::WeakPtr & nodePtr) 
{
  costmap_2d_ptr_ = costmap_2d_ptr;

  auto node = nodePtr.lock(); // get SharedPtr
  enabled_ = node->inflation_params.enabled;
  inflation_radius_ = node->inflation_params.inflation_radius;
  cost_scaling_factor_ = node->inflation_params.cost_scaling_factor;
  inflate_unknown_ = node->inflation_params.inflate_unknown;
  inflate_around_unknown_ = node->inflation_params.inflate_around_unknown;
  footprint_padding_ = node->inflation_params.footprint_padding;

  current_ = true;
  seen_.clear();
  cached_distances_.clear();
  cached_costs_.clear();
  need_reinflation_ = false;

  matchSize();

  ZIYAN_INFO(
    "enabled_: %s, inflate_unknown: %s, inflate_around_unknown: %s, "
    "resolution: %.3f, inflation_radius: %.3f, cell_inflation_radius: %d, cost_scaling_factor: %.3f", 
    enabled_ ? "true" : "false", 
    inflate_unknown_ ? "true" : "false", 
    inflate_around_unknown_ ? "true" : "false", 
    resolution_, inflation_radius_, cell_inflation_radius_, cost_scaling_factor_
  );
}

void
InflationLayer::matchSize()
{
  // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  resolution_ = costmap_2d_ptr_->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);

  computeCaches();
  seen_ = std::vector<bool>(costmap_2d_ptr_->getSizeInCellsX() * costmap_2d_ptr_->getSizeInCellsY(), false);
}

void
InflationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (need_reinflation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_reinflation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void
InflationLayer::onFootprintChanged(double robot_radius)
{
  set_up_footprint_ = true;
  use_radius_ = true;
  unpadded_footprint_ = makeFootprintFromRadius(robot_radius);
  padded_footprint_ = unpadded_footprint_;
  padFootprint(padded_footprint_, footprint_padding_);

  std::pair<double, double> inside_outside = nav2_costmap_2d::calculateMinAndMaxDistances(padded_footprint_);

  #ifdef INFLATION_DEBUG
  for (int i = 0; i < padded_footprint_.size(); i++) {
    ZIYAN_INFO("unpadded_footprint_[%d]: %.3f, %.3f", i, unpadded_footprint_[i].x, unpadded_footprint_[i].y)
  }
  for (int i = 0; i < padded_footprint_.size(); i++) {
    ZIYAN_INFO("padded_footprint_[%d]: %.3f, %.3f", i, padded_footprint_[i].x, padded_footprint_[i].y)
  }
  ZIYAN_INFO("inside_outside: %.3f, %.3f", std::get<0>(inside_outside), std::get<1>(inside_outside))
  #endif

  inscribed_radius_ = std::get<0>(inside_outside);
  circumscribed_radius_ = std::get<1>(inside_outside);

  unsigned int new_cell_inflation_radius_ = cellDistance(inflation_radius_);
  if (new_cell_inflation_radius_ != cell_inflation_radius_) {
    ZIYAN_INFO("NOTICE!, for somereason cell_inflation_radius change from %d to %d", cell_inflation_radius_, new_cell_inflation_radius_);
    cell_inflation_radius_ = new_cell_inflation_radius_;
  }

  ZIYAN_INFO("robot_radius: %.3f, footprint_padding: %.3f, "
    "inscribed_radius: %.3f, circumscribed_radius: %.3f, "
    "inflation_radius: %.3f, cell_inflation_radius: %d",
    robot_radius, footprint_padding_, inscribed_radius_, circumscribed_radius_, 
    inflation_radius_, cell_inflation_radius_
  )

  computeCaches();
  need_reinflation_ = true;
}

void
InflationLayer::updateCosts(
  // nav2_costmap_2d::Costmap2D & master_grid, 
  int min_i, int min_j,
  int max_i,
  int max_j)
{
  // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_ || (cell_inflation_radius_ == 0)) {
    return;
  }

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  for (auto & dist : inflation_cells_) {
    if (!dist.empty())
    ZIYAN_ERROR(
      "The inflation list must be empty at the beginning of inflation");
  }

  unsigned char * master_array = costmap_2d_ptr_ -> getCharMap();
  unsigned int size_x = costmap_2d_ptr_ -> getSizeInCellsX(), size_y = costmap_2d_ptr_ -> getSizeInCellsY();

  if (seen_.size() != size_x * size_y) {
    ZIYAN_INFO(
      "InflationLayer::updateCosts(): seen_ vector size is wrong");
    seen_ = std::vector<bool>(size_x * size_y, false);
  }

  std::fill(begin(seen_), end(seen_), false);

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  const int base_min_i = min_i;
  const int base_min_j = min_j;
  const int base_max_i = max_i;
  const int base_max_j = max_j;
  min_i -= static_cast<int>(cell_inflation_radius_);
  min_j -= static_cast<int>(cell_inflation_radius_);
  max_i += static_cast<int>(cell_inflation_radius_);
  max_j += static_cast<int>(cell_inflation_radius_);

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  ZIYAN_INFO("Inflation range: min_i: %d, min_j: %d, max_i: %d, max_j: %d", min_i, min_j, max_i, max_j);

  // Inflation list; we append cells to visit in a list associated with
  // its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before,
  // with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  auto & obs_bin = inflation_cells_[0];
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = static_cast<int>(costmap_2d_ptr_ -> getIndex(i, j));
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE || (inflate_around_unknown_ && cost == NO_INFORMATION)) {
        obs_bin.emplace_back(index, i, j, i, j);
      }
    }
  }


  // Process cells by increasing distance; new cells are appended to the
  // corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  for (const auto & dist_bin : inflation_cells_) {
    for (std::size_t i = 0; i < dist_bin.size(); ++i) {
      // Do not use iterator or for-range based loops to
      // iterate though dist_bin, since it's size might
      // change when a new cell is enqueued, invalidating all iterators
      unsigned int index = dist_bin[i].index_;

      // ignore if already visited
      if (seen_[index]) {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = dist_bin[i].x_;
      unsigned int my = dist_bin[i].y_;
      unsigned int sx = dist_bin[i].src_x_;
      unsigned int sy = dist_bin[i].src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      unsigned char cost = costLookup(mx, my, sx, sy);
      unsigned char old_cost = master_array[index];
      // In order to avoid artifacts appeared out of boundary areas
      // when some layer is going after inflation_layer,
      // we need to apply inflation_layer only to inside of given bounds
      if (static_cast<int>(mx) >= base_min_i &&
        static_cast<int>(my) >= base_min_j &&
        static_cast<int>(mx) < base_max_i &&
        static_cast<int>(my) < base_max_j)
      {
        if (old_cost == NO_INFORMATION &&
          (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        {
          master_array[index] = cost;
        } else {
          master_array[index] = std::max(old_cost, cost);

          #ifdef INFLATION_DEBUG
          if (old_cost != cost) {
            ZIYAN_INFO("index: %d, old_cost: %d, cost: %d", index, old_cost, cost);
          }
          #endif

        }
      }

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0) {
        enqueue(index - 1, mx - 1, my, sx, sy);
      }
      if (my > 0) {
        enqueue(index - size_x, mx, my - 1, sx, sy);
      }
      if (mx < size_x - 1) {
        enqueue(index + 1, mx + 1, my, sx, sy);
      }
      if (my < size_y - 1) {
        enqueue(index + size_x, mx, my + 1, sx, sy);
      }
    }
  }

  for (auto & dist : inflation_cells_) {
    dist.clear();
    dist.reserve(200);
  }

  current_ = true;
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
void
InflationLayer::enqueue(
  unsigned int index, unsigned int mx, unsigned int my,
  unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index]) {
    // we compute our distance table one cell further than the
    // inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within
    // the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_) {
      return;
    }

    const unsigned int r = cell_inflation_radius_ + 2;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance_matrix_[mx - src_x + r][my - src_y + r]].emplace_back(
      index, mx, my, src_x, src_y);
  }
}

void
InflationLayer::computeCaches()
{
  // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (cell_inflation_radius_ == 0) {
    return;
  }

  cache_length_ = cell_inflation_radius_ + 2;

  // based on the inflation radius... compute distance and cost caches
  if (cell_inflation_radius_ != cached_cell_inflation_radius_) {
    cached_costs_.resize(cache_length_ * cache_length_);
    cached_distances_.resize(cache_length_ * cache_length_);

    for (unsigned int i = 0; i < cache_length_; ++i) {
      for (unsigned int j = 0; j < cache_length_; ++j) {
        cached_distances_[i * cache_length_ + j] = hypot(i, j);

        #ifdef INFLATION_DEBUG
        ZIYAN_INFO("cached_distances_[%d][%d]: %.3f", i, j, cached_distances_[i * cache_length_ + j]);
        #endif

      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }


  for (unsigned int i = 0; i < cache_length_; ++i) {
    for (unsigned int j = 0; j < cache_length_; ++j) {
      cached_costs_[i * cache_length_ + j] = computeCost(cached_distances_[i * cache_length_ + j]);

      #ifdef INFLATION_DEBUG
      ZIYAN_INFO("cached_costs_[%d][%d]: %d", i, j, cached_costs_[i * cache_length_ + j]);
      #endif
    }
  }

  int max_dist = generateIntegerDistances();
  inflation_cells_.clear();
  inflation_cells_.resize(max_dist + 1);
  for (auto & dist : inflation_cells_) {
    dist.reserve(200);
  }
}

int
InflationLayer::generateIntegerDistances()
{
  const int r = cell_inflation_radius_ + 2;
  const int size = r * 2 + 1;

  std::vector<std::pair<int, int>> points;

  for (int y = -r; y <= r; y++) {
    for (int x = -r; x <= r; x++) {
      if (x * x + y * y <= r * r) {
        points.emplace_back(x, y);
      }
    }
  }

  std::sort(
    points.begin(), points.end(),
    [](const std::pair<int, int> & a, const std::pair<int, int> & b) -> bool {
      return a.first * a.first + a.second * a.second < b.first * b.first + b.second * b.second;
    }
  );

  std::vector<std::vector<int>> distance_matrix(size, std::vector<int>(size, 0));
  std::pair<int, int> last = {0, 0};
  int level = 0;
  for (auto const & p : points) {
    if (p.first * p.first + p.second * p.second !=
      last.first * last.first + last.second * last.second)
    {
      level++;
    }
    distance_matrix[p.first + r][p.second + r] = level;
    last = p;
  }

  #ifdef INFLATION_DEBUG
  ZIYAN_INFO("distance matrix:");
  for (int i = 0; i < size; i++){
    for (int j = 0; j < size; j++) {
      std::cout << distance_matrix[i][j] << " ";
    }
    std::cout << std::endl;
  }
  ZIYAN_INFO("level: %d", level);
  #endif

  distance_matrix_ = distance_matrix;
  return level;
}

}  // namespace nav2_costmap_2d
