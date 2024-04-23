#ifndef ZIYAN_PLANNER__NODE_BASIC_HPP_
#define ZIYAN_PLANNER__NODE_BASIC_HPP_

#include <math.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <functional>
#include <queue>
#include <memory>
#include <utility>
#include <limits>

#include "ompl/base/StateSpace.h"

#include "pathplanner/constants.hpp"
#include "pathplanner/node_hybrid.hpp"
#include "pathplanner/node_2d.hpp"
#include "pathplanner/types.hpp"

namespace ziyan_planner
{

/**
 * @class nav2_smac_planner::NodeBasic
 * @brief NodeBasic implementation for priority queue insertion
 */
template<typename NodeT>
class NodeBasic
{
public:
  /**
   * @brief A constructor for nav2_smac_planner::NodeBasic
   * @param index The index of this node for self-reference
   */
  explicit NodeBasic(const unsigned int index)
  : index(index),
    graph_node_ptr(nullptr)
  {
  }

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeT.
   * @param node NodeT ptr to populate metadata into NodeBasic
   */
  void populateSearchNode(NodeT * & node);

  /**
   * @brief Take a NodeBasic and populate it with any necessary state
   * cached in the queue for NodeTs.
   * @param node Search node (basic) object to initialize internal node
   * with state
   */
  void processSearchNode();

  typename NodeT::Coordinates pose;  // Used by NodeHybrid and NodeLattice
  NodeT * graph_node_ptr;
  MotionPrimitive * prim_ptr;  // Used by NodeLattice
  unsigned int index, motion_index;
  bool backward;
  TurnDirection turn_dir;
};

}

#endif
