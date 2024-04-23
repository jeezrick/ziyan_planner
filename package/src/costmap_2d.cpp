#include "pathplanner/costmap_2d.hpp"
#include "pathplanner/logger.hpp"

#include <stdexcept>
#include <cstring>

namespace ziyan_planner
{

Costmap2D::Costmap2D(
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
  double origin_x, double origin_y, uint8_t default_value)
: resolution_(resolution), origin_x_(origin_x),
  origin_y_(origin_y), costmap_(NULL), default_value_(default_value)
{
  // create the costmap
  initMaps(cells_size_x, cells_size_y);
  resetMaps();
}

Costmap2D::Costmap2D(const OccupancyGrid & map)
: default_value_(FREE_SPACE)
{
  // fill local variables
  size_x_ = map.width;
  size_y_ = map.height;
  resolution_ = map.resolution;
  origin_x_ = map.origin_x;
  origin_y_ = map.origin_y;

  // create the costmap
  costmap_ = new uint8_t[size_x_ * size_y_];
  std::memcpy(costmap_, map.data, size_x_ * size_y_ * sizeof(uint8_t));

  ZIYAN_INFO(
    "Make costmap2d -> size_x: %d, size_y: %d," 
    "resolution: %.3f, origin_x: %.3f, origin_y: %.3f", 
    size_x_, size_y_, resolution_, origin_x_, origin_y_
  );
}

void Costmap2D::deleteMaps()
{
  delete[] costmap_;
  costmap_ = NULL;

  ZIYAN_INFO("delete map");
}

void Costmap2D::initMaps(unsigned int size_x, unsigned int size_y)
{
  delete[] costmap_;
  size_x_ = size_x;
  size_y_ = size_y;
  costmap_ = new uint8_t[size_x * size_y];
}

void Costmap2D::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution,
  double origin_x, double origin_y)
{
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  initMaps(size_x, size_y);

  // reset our maps to have no information
  resetMaps();
}

void Costmap2D::resetMaps()
{
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(uint8_t));
}

void Costmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  resetMapToValue(x0, y0, xn, yn, default_value_);
}

void Costmap2D::resetMapToValue(
  unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, uint8_t value)
{
  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) {
    memset(costmap_ + y, value, len * sizeof(uint8_t));
  }
}

bool Costmap2D::copyCostmapWindow(
  const Costmap2D & map, double win_origin_x, double win_origin_y,
  double win_size_x,
  double win_size_y)
{
  // check for self windowing
  if (this == &map) {
    // ROS_ERROR("Cannot convert this costmap into a window of itself");
    return false;
  }

  // clean up old data
  deleteMaps();

  // compute the bounds of our new map
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (!map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) ||
    !map.worldToMap(
      win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x,
      upper_right_y))
  {
    // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
    return false;
  }
  resolution_ = map.resolution_;
  origin_x_ = win_origin_x;
  origin_y_ = win_origin_y;

  // initialize our various maps and reset markers for inflation
  initMaps(upper_right_x - lower_left_x, upper_right_y - lower_left_y);

  // copy the window of the static map and the costmap that we're taking
  copyMapRegion(
    map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_,
    size_x_,
    size_y_);
  return true;
}

bool Costmap2D::copyWindow(
  const Costmap2D & source,
  unsigned int sx0, unsigned int sy0, unsigned int sxn, unsigned int syn,
  unsigned int dx0, unsigned int dy0)
{
  const unsigned int sz_x = sxn - sx0;
  const unsigned int sz_y = syn - sy0;

  if (sxn > source.getSizeInCellsX() || syn > source.getSizeInCellsY()) {
    return false;
  }

  if (dx0 + sz_x > size_x_ || dy0 + sz_y > size_y_) {
    return false;
  }

  copyMapRegion(
    source.costmap_, sx0, sy0, source.size_x_,
    costmap_, dx0, dy0, size_x_,
    sz_x, sz_y);
  return true;
}

Costmap2D & Costmap2D::operator=(const Costmap2D & map)
{
  // check for self assignement
  if (this == &map) {
    return *this;
  }

  // clean up old data
  deleteMaps();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  // initialize our various maps
  initMaps(size_x_, size_y_);

  // copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(uint8_t));

  ZIYAN_INFO("Copy costmap2d by operator '='");

  return *this;
}

Costmap2D::Costmap2D(const Costmap2D & map)
: costmap_(NULL)
{
  *this = map;
}

// just initialize everything to NULL by default
Costmap2D::Costmap2D()
: size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
}

Costmap2D::~Costmap2D()
{
  deleteMaps();
}

unsigned int Costmap2D::cellDistance(double world_dist)
{
  double cells_dist = std::max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

uint8_t * Costmap2D::getMapData() const
{
  return costmap_;
}

uint8_t Costmap2D::getCost(unsigned int mx, unsigned int my) const
{
  return costmap_[getIndex(mx, my)];
}

uint8_t Costmap2D::getCost(unsigned int undex) const
{
  return costmap_[undex];
}

void Costmap2D::setCost(unsigned int mx, unsigned int my, uint8_t cost)
{
  costmap_[getIndex(mx, my)] = cost;
}

void Costmap2D::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap2D::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}

void Costmap2D::worldToMapNoBounds(double wx, double wy, int & mx, int & my) const
{
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);
}

void Costmap2D::worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (wx < origin_x_) {
    mx = 0;
  } else if (wx > resolution_ * size_x_ + origin_x_) {
    mx = size_x_ - 1;
  } else {
    mx = static_cast<int>((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_) {
    my = 0;
  } else if (wy > resolution_ * size_y_ + origin_y_) {
    my = size_y_ - 1;
  } else {
    my = static_cast<int>((wy - origin_y_) / resolution_);
  }
}

void Costmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  // To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  uint8_t * local_map = new uint8_t[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x,
    cell_size_x,
    cell_size_y);

  // now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x,
    cell_size_y);

  // make sure to clean up
  delete[] local_map;
}

unsigned int Costmap2D::getSizeInCellsX() const
{
  return size_x_;
}

unsigned int Costmap2D::getSizeInCellsY() const
{
  return size_y_;
}

double Costmap2D::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

double Costmap2D::getOriginX() const
{
  return origin_x_;
}

double Costmap2D::getOriginY() const
{
  return origin_y_;
}

double Costmap2D::getResolution() const
{
  return resolution_;
}

bool Costmap2D::saveMap(std::string file_name)
{
  FILE * fp = fopen(file_name.c_str(), "w");

  if (!fp) {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++) {
    for (unsigned int ix = 0; ix < size_x_; ix++) {
      uint8_t cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

} 
