#ifndef PATHFINDER_ASTAR_PLANNER_H
#define PATHFINDER_ASTAR_PLANNER_H

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

namespace pathfinder
{

enum TreeValue
{
  UNKNOWN  = -1,
  OCCUPIED = 0,
  FREE     = 1
};

struct Node
{
  octomap::OcTreeKey key;
  double             total_cost;
  double             cum_dist;
  double             goal_dist;

  bool operator==(const Node &other) const;
  bool operator!=(const Node &other) const;
  bool operator<(const Node &other) const;
};

struct CostComparator
{
  bool operator()(const Node &n1, const Node &n2) const;
};

struct HashFunction
{
  bool operator()(const Node &n) const;
};

class AstarPlanner {

public:
  AstarPlanner(std::shared_ptr<octomap::OcTree> tree, const double &euclidean_distance_cutoff, const bool &unknown_is_occupied);

private:
  std::shared_ptr<octomap::OcTree>             tree;
  /* DynamicEDTOctomap                            euclidean_field; */
  std::set<Node, CostComparator>               open;
  std::unordered_set<Node, HashFunction>       closed;
  std::unordered_map<Node, Node, HashFunction> parent_map;  // first = child, second = parent

public:
  /**
   * @brief Find the path from @start point to @goal point
   *
   * @param start 3D point
   * @param goal 3D point
   *
   * @return vector of 3D points ordered from start to goal. Returns empty vector if a path does not exist
   */
  std::vector<octomap::point3d> findPath(const octomap::point3d &start, const octomap::point3d &goal);

  /**
   * @brief Find the path from @start key to @goal key
   *
   * @param start key
   * @param goal key
   *
   * @return vector of 3D points ordered from start to goal. Returns empty vector if a path does not exist
   */
  std::vector<octomap::point3d> findPath(const octomap::OcTreeKey &start, const octomap::OcTreeKey &goal);

private:
  const std::vector<octomap::point3d> EXPANSION_DIRECTIONS = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}};

  /**
   * @brief Compute the depth of a tree node at given key
   *
   * @param key of the node to be evaluated
   *
   * @return depth of the node (by default, the tree depth goes up to 16 for the finest resolution)
   */
  double geNodeDepth(const octomap::OcTreeKey &key);

  /**
   * @brief Return the 6 neighboring cells of the given key. If the neighbor is NULL, it will not be returned
   *
   * @param key of the node to be evaluated
   *
   * @return vector containing keys of neighboring nodes
   */
  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key);

  /**
   * @brief Search the tree in a given direction
   *
   * @param key of the node to be evaluated
   * @param direction 3D direction vector
   *
   * @return key of the next tree node in this direction (will make larger steps if the tree depth is low at the current key)
   */
  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const octomap::point3d &direction);

  /**
   * @brief Compute the Euclidean distance between two 3D points
   *
   * @param p1
   * @param p2
   *
   * @return Euclidean distance in meters
   */
  double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  /**
   * @brief Compute the Euclidean distance between two tree nodes identified by keys
   *
   * @param k1
   * @param k2
   *
   * @return Euclidean distance in meters
   */
  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2);

  /**
   * @brief Check if two points can be connected by a straight line of free cells
   *
   * @param p1
   * @param p2
   *
   * @return true if the line only contains free cells
   */
  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2);

  /**
   * @brief Trace a path from Node @start to Node @end
   * NOTE: requires the parent map to be consistent (@start is required to have a parent, and the subsequent parentage tracking is required to contain @end)
   *
   * @param start
   * @param end
   *
   * @return vector of keys, ordered from end to start
   */
  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end);

  /**
   * @brief Convert a vector of keys to a vector of 3D points
   *
   * @param keys
   * @param tree
   *
   * @return vector of 3D points
   */
  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys);


  /**
   * @brief Perform an Euclidean distance transform across the entire tree
   *
   * @param euclidean_distance_cutoff min distance from obstacles to stop the transform
   * @param unknown_is_occupied treat unknown cells as occupied
   *
   * @return Euclidean distance transform of the octree
   */
  DynamicEDTOctomap euclideanDistanceTransform(const double euclidean_distance_cutoff, const bool unknown_is_occupied);
};
}  // namespace pathfinder

#endif
