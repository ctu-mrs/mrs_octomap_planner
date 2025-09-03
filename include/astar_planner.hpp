#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <octomap/octomap.h>
#include <queue>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <mrs_lib/batch_visualizer.h>

namespace mrs_octomap_planner
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
  bool operator<=(const Node &other) const;
};

struct CostComparator
{
  bool operator()(const Node &n1, const Node &n2) const;
};

struct HashFunction
{
  bool operator()(const Node &n) const;
};

struct LeafComparator
{
  bool operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const;
};

class AstarPlanner {

public:
  AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double submap_distance, double planning_tree_resolution,
               double distance_penalty, double greedy_penalty, double timeout_threshold, double max_waypoint_distance, double min_altitude, double max_altitude,
               bool unknown_is_occupied, std::shared_ptr<mrs_lib::BatchVisualizer> bv);

private:
  double safe_obstacle_distance;
  double euclidean_distance_cutoff;
  float  submap_distance;
  double planning_tree_resolution;
  double distance_penalty;
  double greedy_penalty;
  double timeout_threshold;
  double max_waypoint_distance;
  double min_altitude;
  double max_altitude;
  bool   unknown_is_occupied;

  std::shared_ptr<mrs_lib::BatchVisualizer> bv;

public:
  std::pair<std::vector<octomap::point3d>, bool> findPath(const octomap::point3d &start, const octomap::point3d &goal,
                                                          std::shared_ptr<octomap::OcTree> mapping_tree, const double timeout);

private:
  const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                              {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                              {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                              {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};
  double                              getNodeDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const std::vector<int> &direction, octomap::OcTree &tree);

  double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree);

  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end, std::unordered_map<Node, Node, HashFunction> &parent_map);

  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree);

  DynamicEDTOctomap euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree, const octomap::point3d &start_coord);

  std::optional<std::pair<std::shared_ptr<octomap::OcTree>, std::vector<octomap::point3d>>> createPlanningTree(std::shared_ptr<octomap::OcTree> tree,
                                                                                                               const octomap::point3d &start, double resolution,
                                                                                                               const octomap::point3d &start_coord,
                                                                                                               double                  radius);

  octomap::point3d nearestFreeCoord(const octomap::point3d &p, const octomap::point3d &uav_pos, octomap::OcTree &tree);

  octomap::point3d generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, octomap::OcTree &tree);

  std::vector<octomap::point3d> postprocessPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree);

  std::vector<octomap::point3d> prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree);

  void visualizeTreeCubes(octomap::OcTree &tree, bool show_unoccupied);

  void visualizeGoal(const octomap::point3d &goal);

  void visualizeTreePoints(octomap::OcTree &tree, bool show_unoccupied);

  void visualizeExpansions(const std::unordered_set<Node, HashFunction> &open, const std::unordered_set<Node, HashFunction> &closed, octomap::OcTree &tree);

  octomap::OcTreeNode *touchNode(std::shared_ptr<octomap::OcTree> &octree, const octomap::OcTreeKey &key, unsigned int target_depth = 0);

  octomap::OcTreeNode *touchNodeRecurs(std::shared_ptr<octomap::OcTree> &octree, octomap::OcTreeNode *node, const octomap::OcTreeKey &key, unsigned int depth,
                                       unsigned int max_depth = 0);
};

}  // namespace mrs_octomap_planner

#endif
