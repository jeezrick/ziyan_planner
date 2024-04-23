#ifndef ZIYAN_IO__OCC_GRID_VALUES_HPP_
#define ZIYAN_IO__OCC_GRID_VALUES_HPP_

#include <cstdint>

namespace ZiYan_IO
{

/**
  * @brief OccupancyGrid data constants
  */
static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

}  // namespace ZiYan_IO

#endif  // ZIYAN_IO__OCC_GRID_VALUES_HPP_
