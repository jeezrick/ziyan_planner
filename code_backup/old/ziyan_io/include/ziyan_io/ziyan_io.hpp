#ifndef ZIYAN_IO__ZIYAN_IO_HPP_
#define ZIYAN_IO__ZIYAN_IO_HPP_

#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <cstdint>
#include <math.h>


namespace ZiYan_IO
{


class Info 
{
public:
  using WeakPtr = std::weak_ptr<Info>;
  using SharedPtr = std::shared_ptr<Info>;

  struct SmootherInfo{
    double tolerance = 1e-10;
    int max_iterations = 1000;
    double w_data = 0.2;
    double w_smooth = 0.3;
    bool do_refinement = true;
    int refinement_num = 2;
  };
  SmootherInfo smoother;

  struct Planner2DParams{
    float tolerance = 0.125; // 0.25 for node hybrid
    bool downsample_costmap = false;
    int downsampling_factor = 1;

    float cost_travel_multiplier = 1.0;

    bool allow_unknown = true;

    int max_iterations = 1000000;
    int max_on_approach_iterations = 1000;
    int terminal_checking_interval = 5000;

    bool use_final_approach_orientation = false;

    double max_planning_time = 2.0; // seconds // 5.0 for node hybrid
  };
  Planner2DParams planner2d_params; 

  // for node_hybrid
  struct PlannerHybridParams{
    int angle_quantization_bins = 72;
    bool smooth_path = true;
    double minimum_turning_radius = 0; // 0.4
    double lookup_table_size = 20.0;
    bool _debug_visualizations = false;
    std::string motion_model_for_search = "DUBIN";

    //// search info
    bool allow_primitive_interpolation = false;
    bool cache_obstacle_heuristic = false;
    float reverse_penalty = 2.0;
    float change_penalty = 0.0;
    float non_straight_penalty = 1.2;
    float cost_penalty = 2.0;
    float retrospective_penalty = 0.015;
    float analytic_expansion_ratio = 3.5;
    float analytic_expansion_max_cost = 200.0;
    bool analytic_expansion_max_cost_override = false;
    bool use_quadratic_cost_penalty = false;
    bool downsample_obstacle_heuristic = false;
    float analytic_expansion_max_length = 3.0;

    float tolerance = 0.25;

    bool downsample_costmap = false;
    int downsampling_factor = 1;

    float cost_travel_multiplier = 1.0;

    bool allow_unknown = true;

    int max_iterations = 1000000;
    int max_on_approach_iterations = 1000;
    int terminal_checking_interval = 5000;

    double max_planning_time = 5.0; 
  };
  PlannerHybridParams plannerhybrid_params;

  struct InflationParams { 
    bool enabled = true;
    double inflation_radius = 0.9; // meter
    double cost_scaling_factor = 10.0;
    bool inflate_unknown = false;
    bool inflate_around_unknown = false;

    double radius = 0.05; // meter, to make footprint
    float footprint_padding = 0.0;
  };
  InflationParams inflation_params;

  struct CostmapZiYanParams { 
  };
  CostmapZiYanParams costmapziyan_params;

  struct OccupancyMapParams { 
    double resolution = 0.2;
    unsigned int width = 500;
    unsigned int height = 300; 
    double origin_x = 0.0;
    double origin_y = 0.0; 
    int start_x = 20;
    int start_y = 40;
    int end_x = 150;
    int end_y = 70;
  };
  OccupancyMapParams occupancymap_params;
}; 


struct Entry
{
  int x=0, y=0;

  Entry(float x, float y){
    this -> x = x;
    this -> y = y;
  }
};


class Position
{
public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Position & operator=(const Position& other);
};


class Quaternion
{
public:
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  bool operator==(const Quaternion& other) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z) && (w == other.w);
  }

  bool operator!=(const Quaternion& other) const
  {
    return !(*this == other); // 使用先前定义的 operator==
  }

  Quaternion & operator=(const Quaternion& other);

  /**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
  inline void	setValue(
    const double& x, const double& y, const double& z,const double& w)
  {
    this->x=x;
    this->y=y;
    this->z=z;
    this->w=w;
  }

  /**@brief Set the quaternion using Euler angles
   * @param yaw Angle around Y
   * @param pitch Angle around X
   * @param roll Angle around Z */
	void setEuler(const double& yaw, const double& pitch, const double& roll);

  /**@brief Set the quaternion using fixed axis RPY
   * @param roll Angle around X 
   * @param pitch Angle around Y
   * @param yaw Angle around Z*/
  void setRPY(const double& roll, const double& pitch, const double& yaw);

};


struct Header
{
  int stamp= 0;
  std::string frame_id = "0";
};


class Pose
{
public:
  Position position;
  Quaternion orientation;
  Pose & operator=(const Pose& other);
};


class PoseStamped
{
public:
  Pose pose;
  Header header;
};


class Path
{
public:
  Header header;
  std::vector<PoseStamped> poses;
  std::vector<Entry> path;
};


class Point
{
public:
  double x=0.0, y=0.0, z=0.0;

  Point() {}
  Point(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  Point(const Position& position) : x(position.x), y(position.y), z(position.z) {} 

  Point & operator=(const Point& other);

  bool operator==(const Point & rhs) const
  {
    return (this->x == rhs.x) && (this->y == rhs.y) && (this->z == rhs.z);
  }

};


class OccupancyGrid
{
public:
  OccupancyGrid(
    unsigned int width, unsigned int height, 
    double resolution, 
    double origin_x, double origin_y, 
    int8_t* data);
  
  ~OccupancyGrid();

  unsigned int width;
  unsigned int height;
  double resolution;
  Pose origin;

  int8_t* data; // occupancy array
};


class Point32
{
public:
  int x, y, z;
};


class Polygon
{
public:
  using SharedPtr = std::shared_ptr<Polygon>;
  std::vector<Point32> points;
};


class PolygonStamped
{
public:
  Polygon polygon;
};


inline
double getYaw(const Quaternion& q)
{
  double yaw;

  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x * q.x;
  sqy = q.y * q.y;
  sqz = q.z * q.z;
  sqw = q.w * q.w;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
  // q0 * q3 - q1 * q2 Z-up

  if (sarg <= -0.99999) {
    yaw   = -2 * atan2(q.y, q.x); // q3, q1
  } else if (sarg >= 0.99999) {
    yaw   = 2 * atan2(q.y, q.x);
  } else {
    yaw   = atan2(2 * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz);
  }
  return yaw;
}


}; // namespace ZiYan_IO


#endif // ZIYAN_IO__ZIYAN_IO_HPP_