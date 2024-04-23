#ifndef ZIYAN_PLANNER__COSTMAP_DOWNSAMPLER_HPP_
#define ZIYAN_PLANNER__COSTMAP_DOWNSAMPLER_HPP_

#include <algorithm>
#include <string>
#include <memory>

#include "pathplanner/constants.hpp"
#include "pathplanner/costmap_2d.hpp"
#include "pathplanner/costmap_manager.hpp"
#include "pathplanner/planner_io.hpp"

namespace ziyan_planner
{

class CostmapDownsampler
{
public:
  CostmapDownsampler();
  ~CostmapDownsampler();

  /**
   * @brief Configure the downsampled costmap object and the ROS publisher
   * @param node Lifecycle node pointer
   * @param global_frame The ID of the global frame used by the costmap
   * @param topic_name The name of the topic to publish the downsampled costmap
   * @param costmap The costmap we want to downsample
   * @param downsampling_factor Multiplier for the costmap resolution
   * @param use_min_cost_neighbor If true, min function is used instead of max for downsampling
   */
  void on_configure(
    const Info::WeakPtr & node,
    const std::string & global_frame,
    const std::string & topic_name,
    Costmap2D * const costmap,
    const unsigned int & downsampling_factor,
    const bool & use_min_cost_neighbor = false);

  /**
   * @brief Cleanup the publisher of the downsampled costmap
   */
  void on_cleanup();

  /**
   * @brief Downsample the given costmap by the downsampling factor, and publish the downsampled costmap
   * @param downsampling_factor Multiplier for the costmap resolution
   * @return A ptr to the downsampled costmap
   */
  Costmap2D * downsample(const unsigned int & downsampling_factor);

  /**
   * @brief Resize the downsampled costmap. Used in case the costmap changes and we need to update the downsampled version
   */
  void resizeCostmap();

protected:
  /**
   * @brief Update the sizes X-Y of the costmap and its downsampled version
   */
  void updateCostmapSize();

  /**
   * @brief Explore all subcells of the original costmap and assign the max cost to the new (downsampled) cell
   * @param new_mx The X-coordinate of the cell in the new costmap
   * @param new_my The Y-coordinate of the cell in the new costmap
   */
  void setCostOfCell(
    const unsigned int & new_mx,
    const unsigned int & new_my);

  unsigned int _size_x;
  unsigned int _size_y;
  unsigned int _downsampled_size_x;
  unsigned int _downsampled_size_y;
  unsigned int _downsampling_factor;
  bool _use_min_cost_neighbor;
  float _downsampled_resolution;
  Costmap2D * _costmap;
  std::unique_ptr<Costmap2D> _downsampled_costmap;
};

}  

#endif  
