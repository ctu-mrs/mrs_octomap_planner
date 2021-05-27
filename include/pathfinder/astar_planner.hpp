#ifndef PATHFINDER_ASTAR_PLANNER_H
#define PATHFINDER_ASTAR_PLANNER_H

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <mrs_lib/batch_visualizer.h>

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

struct LeafComparator
{
  bool operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const;
};

class AstarPlanner {

public:
  AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               double timeout_threshold, double max_waypoint_distance, double min_altitude, bool unknown_is_occupied, mrs_lib::BatchVisualizer &bv);

private:
  double safe_obstacle_distance;
  double euclidean_distance_cutoff;
  double planning_tree_resolution;
  double distance_penalty;
  double greedy_penalty;
  double timeout_threshold;
  double max_waypoint_distance;
  double min_altitude;
  bool   unknown_is_occupied;

  mrs_lib::BatchVisualizer bv;


public:
  std::vector<octomap::point3d> findPath(const octomap::point3d &start, const octomap::point3d &goal, std::shared_ptr<octomap::OcTree> mapping_tree);

private:
  const std::vector<octomap::point3d> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                              {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                              {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                              {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};
  double                              getNodeDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const octomap::point3d &direction, octomap::OcTree &tree);

  double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree);

  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end, std::unordered_map<Node, Node, HashFunction> &parent_map);

  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree);

  DynamicEDTOctomap euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree);

  std::pair<octomap::OcTree, std::vector<octomap::point3d>> createPlanningTree(std::shared_ptr<octomap::OcTree> tree, const octomap::point3d &start,
                                                                               double resolution);

  octomap::point3d nearestFreeCoord(const octomap::point3d &p, const octomap::point3d &uav_pos, octomap::OcTree &tree);

  octomap::point3d generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, octomap::OcTree &tree);

  std::vector<octomap::point3d> postprocessPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree);

  std::vector<octomap::point3d> prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree);

  void visualizeTreeCubes(octomap::OcTree &tree, bool show_unoccupied);

  void visualizeTreePoints(octomap::OcTree &tree, bool show_unoccupied);

  void visualizeExpansions(std::set<Node, CostComparator> open, std::unordered_set<Node, HashFunction> closed, octomap::OcTree &tree);
};

}  // namespace pathfinder

#endif
