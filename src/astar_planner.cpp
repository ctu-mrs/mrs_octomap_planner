#include "octomap/OcTree.h"
#include "octomap/OcTreeKey.h"
#include <pathfinder/astar_planner.hpp>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <queue>

AstarPlanner::AstarPlanner() {
}

/* findPath //{ */
std::vector<p3d> AstarPlanner::findPath(const p3d &start, const p3d &goal, octomap::OcTree &tree) {
  std::vector<p3d> waypoints;

  auto start_key = tree.coordToKey(start);
  auto goal_key  = tree.coordToKey(goal);

  std::unordered_map<octomap::OcTreeKey, Cost, octomap::OcTreeKey::KeyHash>               open_map;    // first = key, second = cost
  std::map<Cost, octomap::OcTreeKey, decltype(&costCompare)>                              open_queue;  // first = cost, second = key
  std::unordered_map<octomap::OcTreeKey, Cost, octomap::OcTreeKey::KeyHash>               closed_map;  // first = key, second = cost
  std::unordered_map<octomap::OcTreeKey, octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> parent_map;  // first = child, second = parent


  Cost cost;
  cost.goal_dist      = distEuclidean(start_key, goal_key, tree);
  cost.cum_dist       = 0;
  cost.total_cost     = cost.cum_dist + cost.goal_dist;
  open_map[start_key] = cost;
  open_queue[cost]    = start_key;


  return waypoints;
}
//}

/* geNodeDepth //{ */
double AstarPlanner::geNodeDepth(octomap::OcTreeKey &key, octomap::OcTree &tree) {
  for (auto it = tree.begin(); it != tree.end(); it++) {
    if (it.getKey() == key) {
      return it.getDepth();
    }
  }
  return tree.getTreeDepth();
}
//}

/* getNeighborKeys //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getFreeNeighborKeys(const octomap::OcTreeKey &key, octomap::OcTree &tree) {
  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {
    auto newkey    = expand(key, d, tree);
    auto tree_node = tree.search(newkey);
    if (tree_node != NULL && std::abs(tree_node->getValue()) < 0.3) {
      // free cell
      neighbors.push_back(newkey);
    }
  }

  return neighbors;
}
//}

/* expand //{ */
octomap::OcTreeKey AstarPlanner::expand(const octomap::OcTreeKey &key, const p3d &direction, octomap::OcTree &tree) {
  auto prev_node = tree.search(key);

  octomap::OcTreeKey k;
  int                i = 1;

  k.k[0] = key.k[0] + direction.x();
  k.k[1] = key.k[1] + direction.y();
  k.k[2] = key.k[2] + direction.z();

  // expand outward until a new octree node is reached
  while (tree.search(k) == prev_node) {
    ++i;
    k.k[0] = key.k[0] + i * direction.x();
    k.k[1] = key.k[1] + i * direction.y();
    k.k[2] = key.k[2] + i * direction.z();
  }

  return k;
}
//}

/* distEuclidean //{ */
double AstarPlanner::distEuclidean(const p3d &p1, const p3d &p2) {
  return (p1 - p2).norm();
}

double AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, const octomap::OcTree &tree) {
  double voxel_dist = std::sqrt(std::pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));
  return voxel_dist * tree.getResolution();
}
//}
