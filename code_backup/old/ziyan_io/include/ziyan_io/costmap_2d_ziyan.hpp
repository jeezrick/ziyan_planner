#ifndef ZIYAN_IO__COSTMAP_2D_ZIYAN_HPP_
#define ZIYAN_IO__COSTMAP_2D_ZIYAN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ziyan_io/inflation_layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "ziyan_io/ziyan_io.hpp"

namespace ZiYan_IO
{

using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::InflationLayer; 

class Costmap2DZiYan
{

public:
  explicit Costmap2DZiYan(
    const ZiYan_IO::Info::WeakPtr & parent, 
    std::shared_ptr<Costmap2D> costmap_2d_ptr
  );

  ~Costmap2DZiYan();

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap().
   */
  Costmap2D * getCostmap()
  {
    return costmap_2d_ptr_.get();
  }

  std::string getGlobalFrameID()
  {
    return global_frame_;
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<ZiYan_IO::Point> getRobotFootprint()
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

  std::shared_ptr<InflationLayer> getInflationLayer() {return inflation_layer_ptr_;} 

protected:
  // Costmap2D costmap2d_;
  std::shared_ptr<Costmap2D> costmap_2d_ptr_ = std::make_shared<Costmap2D>();
  std::shared_ptr<InflationLayer> inflation_layer_ptr_ = std::make_shared<InflationLayer>();

  std::string global_frame_{1};          ///< The global frame for the costmap
  bool rolling_window_{false};          ///< Whether to use a rolling window version of the costmap
  bool track_unknown_space_{false};
  double resolution_{0};
  int map_height_meters_{0};
  int map_width_meters_{0};
  double origin_x_{0};
  double origin_y_{0};
};

}

#endif  // ZIYAN_IO__COSTMAP_2D_ZIYAN_HPP_