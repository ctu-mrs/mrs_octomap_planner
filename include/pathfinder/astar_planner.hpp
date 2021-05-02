#ifndef PATHFINDER_ASTAR_PLANNER_H
#define PATHFINDER_ASTAR_PLANNER_H

#define VISUALIZE

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#ifdef VISUALIZE
#include <mrs_lib/batch_visualizer.h>
#endif


namespace pathfinder
{

enum TreeValue
{
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
#ifndef VISUALIZE
  AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               bool unknown_is_occupied);
#endif
#ifdef VISUALIZE
  AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               bool unknown_is_occupied, mrs_lib::BatchVisualizer &bv);
#endif

private:
  double safe_obstacle_distance;
  double euclidean_distance_cutoff;
  double planning_tree_resolution;
  double distance_penalty;
  double greedy_penalty;
  bool   unknown_is_occupied;

#ifdef VISUALIZE
  mrs_lib::BatchVisualizer bv;
#endif


public:
  /**
   * @brief Find the path from @start point to @goal point
   *
   * @param start 3D point
   * @param goal 3D point
   * @param mapping_tree current octomap of the world
   *
   * @return vector of 3D points ordered from start to goal. Returns empty vector if a path does not exist
   */
  std::vector<octomap::point3d> findPath(const octomap::point3d &start, const octomap::point3d &goal, std::shared_ptr<octomap::OcTree> mapping_tree);

private:
  const std::vector<octomap::point3d> EXPANSION_DIRECTIONS = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}};

  /**
   * @brief Compute the depth of a tree node at given key
   *
   * @param key of the node to be evaluated
   *
   * @return depth of the node (by default, the tree depth goes up to 16 for the finest resolution)
   */
  double getNodeDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  /**
   * @brief Return the 6 neighboring cells of the given key. If the neighbor is NULL, it will not be returned
   *
   * @param key of the node to be evaluated
   * @param tree
   *
   * @return vector containing keys of neighboring nodes
   */
  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  /**
   * @brief Search the tree in a given direction
   *
   * @param key of the node to be evaluated
   * @param direction 3D direction vector
   *
   * @return key of the next tree node in this direction (will make larger steps if the tree depth is low at the current key)
   */
  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const octomap::point3d &direction, octomap::OcTree &tree);

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
   * @param tree
   *
   * @return Euclidean distance in meters
   */
  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree);

  /**
   * @brief Check if two points can be connected by a straight line of free cells
   *
   * @param p1
   * @param p2
   * @param tree
   *
   * @return true if the line only contains free cells
   */
  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree);

  /**
   * @brief Trace a path from Node @start to Node @end
   * NOTE: requires the parent map to be consistent (@start is required to have a parent, and the subsequent parentage tracking is required to contain @end)
   *
   * @param start
   * @param end
   *
   * @return vector of keys, ordered from end to start
   */
  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end, std::unordered_map<Node, Node, HashFunction> &parent_map);

  /**
   * @brief Convert a vector of keys to a vector of 3D points
   *
   * @param keys
   * @param tree
   *
   * @return vector of 3D points
   */
  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree);


  /**
   * @brief Perform an Euclidean distance transform across the entire tree
   *
   * @param euclidean_distance_cutoff min distance from obstacles to stop the transform
   * @param unknown_is_occupied treat unknown cells as occupied
   *
   * @return Euclidean distance transform of the octree
   */
  DynamicEDTOctomap euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree);

  /**
   * @brief Create a modified tree suitable for collision-free path planning
   * Perform an Euclidean distance transform, clear area around starting position and crop low altitudes
   *
   * @param tree output of mapping node
   * @param start UAV coords
   * @param resolution voxel size of the planning tree
   *
   * @return octree suitable for planning
   */
  octomap::OcTree createPlanningTree(std::shared_ptr<octomap::OcTree> tree, const octomap::point3d &start, double resolution);

#ifdef VISUALIZE
  /**
   * @brief Use the MRS Batch Visualizer to draw the full octree
   *
   * @param tree octree to be visualized
   * @param show_unoccupied true to also draw unoccupied cells
   */
  void visualizeTreeCubes(octomap::OcTree &tree, bool show_unoccupied);

  /**
   * @brief Use the MRS Batch Visualizer to draw the octree in a symbolic manner. Brighter points represent larger depths
   *
   * @param tree
   * @param show_unoccupied
   */
  void visualizeTreePoints(octomap::OcTree &tree, bool show_unoccupied);

  /**
   * @brief Use the MRS Batch Visualizer to draw all open and closed expansions
   *
   * @param open
   * @param closed
   * @param tree
   */
  void visualizeExpansions(std::set<Node, CostComparator> open, std::unordered_set<Node, HashFunction> closed, octomap::OcTree &tree);
#endif
};

}  // namespace pathfinder

#endif
