#ifndef ZIYAN_PLANNER__ANALYTIC_EXPANSION_HPP_
#define ZIYAN_PLANNER__ANALYTIC_EXPANSION_HPP_

#include <string>
#include <vector>
#include <list>
#include <memory>
#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/DubinsStateSpace.h"

#include "pathplanner/node_2d.hpp"
#include "pathplanner/node_hybrid.hpp"
#include "pathplanner/types.hpp"
#include "pathplanner/constants.hpp"

namespace ziyan_planner 
{

template<typename NodeT>
class AnalyticExpansion
{
public:
  typedef NodeT * NodePtr;
  typedef typename NodeT::Coordinates Coordinates;
  typedef std::function<bool (const unsigned int &, NodeT * &)> NodeGetter;

  /**
   * @struct nav2_smac_planner::AnalyticExpansion::AnalyticExpansionNodes
   * @brief Analytic expansion nodes and associated metadata
   */
  struct AnalyticExpansionNode
  {
    AnalyticExpansionNode(
      NodePtr & node_in,
      Coordinates & initial_coords_in,
      Coordinates & proposed_coords_in)
    : node(node_in),
      initial_coords(initial_coords_in),
      proposed_coords(proposed_coords_in)
    {
    }

    NodePtr node;
    Coordinates initial_coords;
    Coordinates proposed_coords;
  };

  typedef std::vector<AnalyticExpansionNode> AnalyticExpansionNodes;

  /**
   * @brief Constructor for analytic expansion object
   */
  AnalyticExpansion(
    const MotionModel & motion_model,
    const SearchInfo & search_info,
    const bool & traverse_unknown,
    const unsigned int & dim_3_size);

  /**
   * @brief Sets the collision checker and costmap to use in expansion validation
   * @param collision_checker Collision checker to use
   */
  void setCollisionChecker(GridCollisionChecker * collision_checker);

  /**
   * @brief Attempt an analytic path completion
   * @param node The node to start the analytic path from
   * @param goal The goal node to plan to
   * @param getter Gets a node at a set of coordinates
   * @param iterations Iterations to run over
   * @param best_cost Best heuristic cost to propertionally expand more closer to the goal
   * @return Node pointer reference to goal node if successful, else
   * return nullptr
   */
  NodePtr tryAnalyticExpansion(
    const NodePtr & current_node,
    const NodePtr & goal_node,
    const NodeGetter & getter, int & iterations, int & best_cost);

  /**
   * @brief Perform an analytic path expansion to the goal
   * @param node The node to start the analytic path from
   * @param goal The goal node to plan to
   * @param getter The function object that gets valid nodes from the graph
   * @param state_space State space to use for computing analytic expansions
   * @return A set of analytically expanded nodes to the goal from current node, if possible
   */
  AnalyticExpansionNodes getAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const NodeGetter & getter, const ompl::base::StateSpacePtr & state_space);

  /**
   * @brief Takes final analytic expansion and appends to current expanded node
   * @param node The node to start the analytic path from
   * @param goal The goal node to plan to
   * @param expanded_nodes Expanded nodes to append to end of current search path
   * @return Node pointer to goal node if successful, else return nullptr
   */
  NodePtr setAnalyticPath(
    const NodePtr & node, const NodePtr & goal,
    const AnalyticExpansionNodes & expanded_nodes);

  /**
   * @brief Takes an expanded nodes to clean up, if necessary, of any state
   * information that may be poluting it from a prior search iteration
   * @param expanded_nodes Expanded node to clean up from search
   */
  void cleanNode(const NodePtr & nodes);

protected:
  MotionModel _motion_model;
  SearchInfo _search_info;
  bool _traverse_unknown;
  unsigned int _dim_3_size;
  GridCollisionChecker * _collision_checker;
  std::list<std::unique_ptr<NodeT>> _detached_nodes;
};

}  

#endif  
