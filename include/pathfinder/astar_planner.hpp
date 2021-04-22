#ifndef PATHFINDER_ASTAR_PLANNER_H
#define PATHFINDER_ASTAR_PLANNER_H

#include <vector>
#include <octomap/octomap.h>

/* Cost //{ */
struct Cost
{
  double total_cost;
  double cum_dist;
  double goal_dist;
};
//}

bool costCompare(Cost c1, Cost c2) {
  if (c1.total_cost == c2.total_cost) {
    return c1.goal_dist < c2.goal_dist;
  }
  return c1.total_cost < c2.total_cost;
}

typedef octomap::point3d p3d;

class AstarPlanner {

public:
  AstarPlanner();

public:
  std::vector<p3d> findPath(const p3d &start, const p3d &goal, octomap::OcTree &tree);


private:
  const std::vector<p3d> EXPANSION_DIRECTIONS = {{0, 0, 1}, {0, 0, -1}, {0, 1, 0}, {0, -1, 0}, {1, 0, 0}, {-1, 0, 0}};
  double                 geNodeDepth(octomap::OcTreeKey &key, octomap::OcTree &tree);

  std::vector<octomap::OcTreeKey> getFreeNeighborKeys(const octomap::OcTreeKey &key, octomap::OcTree &tree);

  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const p3d &direction, octomap::OcTree &tree);

  double distEuclidean(const p3d &p1, const p3d &p2);
  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, const octomap::OcTree &tree);
};


#endif
