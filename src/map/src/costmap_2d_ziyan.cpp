#include "map/costmap_2d_ziyan.hpp"
#include "ziyan_io/logger.hpp"

namespace ZiYan_IO
{

Costmap2DZiYan::Costmap2DZiYan(
  const ZiYan_IO::Info::WeakPtr & parent,
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

Costmap2DZiYan::Costmap2DZiYan(
  const ZiYan_IO::Info::WeakPtr & parent, 
  const ZiYan_IO::OccupancyGrid & map
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


Costmap2DZiYan::~Costmap2DZiYan()
{
}

}