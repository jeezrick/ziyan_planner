#include <string>
#include <memory>
#include <algorithm>

#include "pathplanner/costmap_downsampler.hpp"

namespace ziyan_planner
{

CostmapDownsampler::CostmapDownsampler()
: _costmap(nullptr),
  _downsampled_costmap(nullptr)
{
}

CostmapDownsampler::~CostmapDownsampler()
{
}

void CostmapDownsampler::on_configure(
  const Info::WeakPtr & node,
  const std::string & global_frame,
  const std::string & topic_name,
  Costmap2D * const costmap,
  const unsigned int & downsampling_factor,
  const bool & use_min_cost_neighbor)
{
  _costmap = costmap;
  _downsampling_factor = downsampling_factor;
  _use_min_cost_neighbor = use_min_cost_neighbor;
  updateCostmapSize();

  _downsampled_costmap = std::make_unique<Costmap2D>(
    _downsampled_size_x, _downsampled_size_y, _downsampled_resolution,
    _costmap->getOriginX(), _costmap->getOriginY(), f_UNKNOWN);
}

void CostmapDownsampler::on_cleanup()
{
  _costmap = nullptr;
  _downsampled_costmap.reset();
}

Costmap2D * CostmapDownsampler::downsample(
  const unsigned int & downsampling_factor)
{
  _downsampling_factor = downsampling_factor;
  updateCostmapSize();

  // Adjust costmap size if needed
  if (_downsampled_costmap->getSizeInCellsX() != _downsampled_size_x ||
    _downsampled_costmap->getSizeInCellsY() != _downsampled_size_y ||
    _downsampled_costmap->getResolution() != _downsampled_resolution)
  {
    resizeCostmap();
  }

  // Assign costs
  for (unsigned int i = 0; i < _downsampled_size_x; ++i) {
    for (unsigned int j = 0; j < _downsampled_size_y; ++j) {
      setCostOfCell(i, j);
    }
  }

  return _downsampled_costmap.get();
}

void CostmapDownsampler::updateCostmapSize()
{
  _size_x = _costmap->getSizeInCellsX();
  _size_y = _costmap->getSizeInCellsY();
  _downsampled_size_x = ceil(static_cast<float>(_size_x) / _downsampling_factor);
  _downsampled_size_y = ceil(static_cast<float>(_size_y) / _downsampling_factor);
  _downsampled_resolution = _downsampling_factor * _costmap->getResolution();
}

void CostmapDownsampler::resizeCostmap()
{
  _downsampled_costmap->resizeMap(
    _downsampled_size_x,
    _downsampled_size_y,
    _downsampled_resolution,
    _costmap->getOriginX(),
    _costmap->getOriginY());
}

void CostmapDownsampler::setCostOfCell(
  const unsigned int & new_mx,
  const unsigned int & new_my)
{
  unsigned int mx, my;
  unsigned char cost = _use_min_cost_neighbor ? 255 : 0;
  unsigned int x_offset = new_mx * _downsampling_factor;
  unsigned int y_offset = new_my * _downsampling_factor;

  for (unsigned int i = 0; i < _downsampling_factor; ++i) {
    mx = x_offset + i;
    if (mx >= _size_x) {
      continue;
    }
    for (unsigned int j = 0; j < _downsampling_factor; ++j) {
      my = y_offset + j;
      if (my >= _size_y) {
        continue;
      }
      cost = _use_min_cost_neighbor ?
        std::min(cost, _costmap->getCost(mx, my)) :
        std::max(cost, _costmap->getCost(mx, my));
    }
  }

  _downsampled_costmap->setCost(new_mx, new_my, cost);
}

}  
