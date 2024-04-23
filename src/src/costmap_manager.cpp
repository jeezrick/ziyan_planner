#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/logger.hpp"

namespace ziyan_planner 
{

CostmapManager::CostmapManager(
  const Info::WeakPtr & parent,
  std::shared_ptr<Costmap2D> costmap_2d_ptr
) {
  auto node_ = parent.lock();

  costmap_2d_ptr_ = costmap_2d_ptr;
  /* 
  * wonderful's note: If instance was used here, the operator '=' will create 
  * a new unsigned char array and copy the value of the right-hand side to the left-hand side.
  */

  inflation_layer_ptr_ -> initialize(costmap_2d_ptr_, parent);
  inflation_layer_ptr_ -> onFootprintChanged(node_->inflation_params.radius);

  inflation_layer_ptr_ -> updateCosts( 
    0, 0, costmap_2d_ptr_->getSizeInCellsX(), costmap_2d_ptr_->getSizeInCellsY());
  
  resolution_ = costmap_2d_ptr_->getResolution();
  origin_x_ = costmap_2d_ptr_->getOriginX();
  origin_y_ = costmap_2d_ptr_->getOriginY();
}

CostmapManager::CostmapManager(
  const Info::WeakPtr & parent, 
  const OccupancyGrid & map
) {
  auto node_ = parent.lock();

  costmap_2d_ptr_ = std::make_shared<Costmap2D>(map);

  inflation_layer_ptr_ -> initialize(costmap_2d_ptr_, parent);
  inflation_layer_ptr_ -> onFootprintChanged(node_->inflation_params.radius);

  inflation_layer_ptr_ -> updateCosts( 
    0, 0, costmap_2d_ptr_->getSizeInCellsX(), costmap_2d_ptr_->getSizeInCellsY());
  
  resolution_ = costmap_2d_ptr_->getResolution();
  origin_x_ = costmap_2d_ptr_->getOriginX();
  origin_y_ = costmap_2d_ptr_->getOriginY();

}

CostmapManager::~CostmapManager()
{
}

}