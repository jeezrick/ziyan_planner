#ifndef ZIYAN_PLANNER__COSTMAP_MANAGER_HPP_
#define ZIYAN_PLANNER__COSTMAP_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pathplanner/inflation_layer.hpp"
#include "pathplanner/costmap_2d.hpp"
#include "pathplanner/planner_io.hpp"


namespace ziyan_planner
{

class CostmapManager
{

public:
  explicit CostmapManager(
    const Info::WeakPtr & parent, 
    std::shared_ptr<Costmap2D> costmap_2d_ptr
  );

  explicit CostmapManager(
    const Info::WeakPtr & parent, 
    unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
    double origin_x, double origin_y, uint8_t* data_ptr
  );

  explicit CostmapManager(
    const Info::WeakPtr & parent, 
    unsigned char* data_ptr, 
    unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
    double origin_x, double origin_y 
  );

  ~CostmapManager();

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap().
   */
  std::shared_ptr<Costmap2D> getCostmapSharePtr()
  {
    return costmap_2d_ptr_;
  }

  Costmap2D * getCostmapPtr()
  {
    return costmap_2d_ptr_.get();
  }

  std::shared_ptr<InflationLayer> getInflationLayer() 
  {
    return inflation_layer_ptr_;
  } 

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<Point> getRobotFootprint()
  {
    return inflation_layer_ptr_ -> getRobotFootprint();
  }

  /**
   * @brief  Get the costmap's use_radius_ parameter, corresponding to
   * whether the footprint for the robot is a circle with radius robot_radius_
   * or an arbitrarily defined footprint in footprint_.
   * @return  use_radius_
   */
  bool getUseRadius() 
  {
    return inflation_layer_ptr_ -> getUseRadius();
  }

protected:
  std::shared_ptr<Costmap2D> costmap_2d_ptr_ = nullptr;
  std::shared_ptr<InflationLayer> inflation_layer_ptr_ = std::make_shared<InflationLayer>();

  double resolution_{0};
  int map_height_meters_{0};
  int map_width_meters_{0};
  double origin_x_{0};
  double origin_y_{0};
};

}

#endif  // ZIYAN_IO__COSTMAP_MANAGER_HPP_