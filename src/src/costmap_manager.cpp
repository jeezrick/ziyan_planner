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
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
  double origin_x, double origin_y, uint8_t* data_ptr) 
{
  auto node_ = parent.lock();

  costmap_2d_ptr_ = std::make_shared<Costmap2D>(
    cells_size_x, cells_size_y, resolution, origin_x, origin_y, data_ptr);

  inflation_layer_ptr_ -> initialize(costmap_2d_ptr_, parent);
  inflation_layer_ptr_ -> onFootprintChanged(node_->inflation_params.radius);

  inflation_layer_ptr_ -> updateCosts( 
    0, 0, costmap_2d_ptr_->getSizeInCellsX(), costmap_2d_ptr_->getSizeInCellsY());

  if (node_->inflation_params.save_inflated_map) {
    costmap_2d_ptr_ -> saveMap(node_->inflation_params.save_path);
  }
  
  resolution_ = costmap_2d_ptr_->getResolution();
  origin_x_ = costmap_2d_ptr_->getOriginX();
  origin_y_ = costmap_2d_ptr_->getOriginY();
}

CostmapManager::CostmapManager(
  const Info::WeakPtr & parent, 
  unsigned char* data_ptr, 
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
  double origin_x, double origin_y) 
{
  uint8_t *converted_data = reinterpret_cast<uint8_t *>(data_ptr);
  CostmapManager(parent, cells_size_x, cells_size_y, resolution, origin_x, origin_y, converted_data);
}

CostmapManager::~CostmapManager()
{
}

}