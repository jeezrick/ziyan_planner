#ifndef ZIYAN_PLANNER__INFLATION_LAYER_HPP_
#define ZIYAN_PLANNER__INFLATION_LAYER_HPP_

#include <map>
#include <vector>
#include <memory>
#include <string>
#include <stdexcept>

#include "pathplanner/costmap_2d.hpp"
#include "pathplanner/planner_io.hpp"


namespace ziyan_planner
{
 
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy)
  : index_(static_cast<unsigned int>(i)), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};

/**
 * @class InflationLayer
 * @brief Layer to convolve costmap by robot's radius or footprint to prevent
 * collisions and largely simply collision checking
 */
class InflationLayer
{
public:
  /**
    * @brief A constructor
    */
  InflationLayer();

  /**
    * @brief A destructor
    */
  ~InflationLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  void initialize(
    std::shared_ptr<Costmap2D> costmap_2d_ptr, 
    const Info::WeakPtr & node
  );

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateCosts(
    // nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  bool isClearable() {return false;}

  /**
   * @brief Reset this costmap
   */
  void reset()
  {
    matchSize();
    current_ = false;
  }

  /** @brief  Given a distance, compute a cost.
   * @param  distance The distance from an obstacle in cells
   * @return A cost value for the distance */
  inline uint8_t computeCost(double distance) const
  {
    uint8_t cost = 0;
    if (distance == 0) {
      cost = LETHAL_OBSTACLE;
    } else if (distance * resolution_ <= inscribed_radius_) {
      cost = INSCRIBED_INFLATED_OBSTACLE;
    } else {
      // make sure cost falls off by Euclidean distance
      double factor =
        exp(-1.0 * cost_scaling_factor_ * (distance * resolution_ - inscribed_radius_));
      cost = static_cast<uint8_t>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

  double getCostScalingFactor()
  {
    return cost_scaling_factor_;
  }

  double getInflationRadius()
  {
    return inflation_radius_;
  }

  double getInscribedRadius()
  {
    return inscribed_radius_;
  }

  double getCircumscribedRadius()
  {
    return circumscribed_radius_;
  }

  /**
   * @brief Process updates on footprint changes to the inflation layer
   */
  // void onFootprintChanged();
  void onFootprintChanged(double robot_radius);

  std::vector<Point> getRobotFootprint()
  {
    if (!set_up_footprint_) 
      throw std::runtime_error("Footprint not set up yet");
    return padded_footprint_;
  }

  /**
   * @brief  Get the costmap's use_radius_ parameter, corresponding to
   * whether the footprint for the robot is a circle with radius robot_radius_
   * or an arbitrarily defined footprint in footprint_.
   * @return  use_radius_
   */
  bool getUseRadius() {
    if (!set_up_footprint_) 
      throw std::runtime_error("Footprint not set up yet");
    return use_radius_;
  }


protected:
  /**
   * @brief  Lookup pre-computed distances
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline double distanceLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_distances_[dx * cache_length_ + dy];
  }

  /**
   * @brief  Lookup pre-computed costs
   * @param mx The x coordinate of the current cell
   * @param my The y coordinate of the current cell
   * @param src_x The x coordinate of the source cell
   * @param src_y The y coordinate of the source cell
   * @return
   */
  inline uint8_t costLookup(
    unsigned int mx, unsigned int my, unsigned int src_x,
    unsigned int src_y)
  {
    unsigned int dx = (mx > src_x) ? mx - src_x : src_x - mx;
    unsigned int dy = (my > src_y) ? my - src_y : src_y - my;
    return cached_costs_[dx * cache_length_ + dy];
  }

  /**
   * @brief Compute cached dsitances
   */
  void computeCaches();

  /**
   * @brief Compute cached dsitances
   */
  int generateIntegerDistances();

  /**
   * @brief Compute cached dsitances
   */
  unsigned int cellDistance(double world_dist)
  {
    return costmap_2d_ptr_->cellDistance(world_dist);
  }

  /**
   * @brief Enqueue new cells in cache distance update search
   */
  inline void enqueue(
    unsigned int index, unsigned int mx, unsigned int my,
    unsigned int src_x, unsigned int src_y);

  double inflation_radius_, cost_scaling_factor_;
  bool inflate_unknown_, inflate_around_unknown_;
  float footprint_padding_{0};

  unsigned int cell_inflation_radius_;
  unsigned int cached_cell_inflation_radius_;
  std::vector<std::vector<CellData>> inflation_cells_;

  double resolution_;

  std::vector<bool> seen_;

  std::vector<uint8_t> cached_costs_;
  std::vector<double> cached_distances_;
  std::vector<std::vector<int>> distance_matrix_;
  unsigned int cache_length_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire costmap should be reinflated next time around.
  bool need_reinflation_;

  // wonderful add
  bool current_;
  bool enabled_;

  std::shared_ptr<Costmap2D> costmap_2d_ptr_; 

  // Derived parameters
  bool use_radius_{true};
  bool set_up_footprint_{false};
  std::vector<Point> unpadded_footprint_;
  std::vector<Point> padded_footprint_;
  double inscribed_radius_, circumscribed_radius_;
};

}  

#endif // ZIYAN_PLANNER__INFLATION_LAYER_HPP_ 
