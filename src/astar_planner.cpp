#include "ros/duration.h"
#include <pathfinder/astar_planner.hpp>

namespace pathfinder
{

bool Node::operator==(const Node &other) const {
  return key == other.key;
}
bool Node::operator!=(const Node &other) const {
  return key != other.key;
}
bool Node::operator<(const Node &other) const {
  if (total_cost == other.total_cost) {
    return goal_dist < other.goal_dist;
  }
  return total_cost < other.total_cost;
}

bool CostComparator::operator()(const Node &n1, const Node &n2) const {
  if (n1.total_cost == n2.total_cost) {
    return n1.goal_dist < n2.goal_dist;
  }
  return n1.total_cost < n2.total_cost;
}

bool HashFunction::operator()(const Node &n) const {
  using std::hash;
  return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
}

/* AstarPlanner constructor //{ */
#ifndef VISUALIZE
AstarPlanner::AstarPlanner(double safe_obstacle_distance, double clearing_radius, double euclidean_distance_cutoff, bool unknown_is_occupied) {
  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->clearing_radius           = clearing_radius;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->unknown_is_occupied       = unknown_is_occupied;
}
#endif

#ifdef VISUALIZE
AstarPlanner::AstarPlanner(double safe_obstacle_distance, double clearing_radius, double euclidean_distance_cutoff, bool unknown_is_occupied,
                           mrs_lib::BatchVisualizer &bv) {
  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->clearing_radius           = clearing_radius;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->unknown_is_occupied       = unknown_is_occupied;
  this->bv                        = bv;
}
#endif
//}

/* findPath main //{ */
std::vector<octomap::point3d> AstarPlanner::findPath(const octomap::OcTreeKey &start, const octomap::OcTreeKey &goal,
                                                     std::shared_ptr<octomap::OcTree> mapping_tree) {

  octomap::OcTree tree = createPlanningTree(mapping_tree, start);

#ifdef VISUALIZE
  bv.clearBuffers();
  bv.clearVisuals();
  /* visualizeTreeCubes(tree, false); */
  bv.publish();
#endif

  std::set<Node, CostComparator>               open;
  std::unordered_set<Node, HashFunction>       closed;
  std::unordered_map<Node, Node, HashFunction> parent_map;  // first = child, second = parent

  auto start_coord = tree.keyToCoord(start);
  auto goal_coord  = tree.keyToCoord(goal);

  std::cout << "Start planning from: " << start_coord.x() << ", " << start_coord.y() << ", " << start_coord.z() << std::endl;

  Node first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distEuclidean(start, goal, tree);
  first.total_cost = first.cum_dist + first.goal_dist;
  open.insert(first);

  int iteration = 0;
  while (!open.empty() && iteration < 100000) {
#ifdef VISUALIZE
    bv.clearBuffers();
    visualizeExpansions(open, closed, tree);
    bv.publish();
#endif
    iteration++;

    auto current = *open.begin();
    open.erase(current);
    closed.insert(current);

    auto current_coord = tree.keyToCoord(current.key);
    std::cout << "Current coord: " << current_coord.x() << ", " << current_coord.y() << ", " << current_coord.z() << std::endl;

    if (freeStraightPath(current_coord, goal_coord, tree)) {
      auto path_keys = backtrackPathKeys(current, first, parent_map);
      std::cout << "PATH FOUND! Length: " << path_keys.size() << std::endl;
      return keysToCoords(path_keys, tree);
    }

    // expand
    auto neighbors = getNeighborhood(current.key, tree);
    for (auto &nkey : neighbors) {

      if (tree.search(nkey)->getValue() == TreeValue::OCCUPIED) {
        continue;
      }

      auto ncoord = tree.keyToCoord(nkey);
      Node n;
      n.key        = nkey;
      n.goal_dist  = distEuclidean(nkey, goal, tree);
      n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, tree);
      n.total_cost = n.goal_dist + n.cum_dist;

      // check if open
      auto open_query = open.find(n);
      if (open_query != open.end()) {
        // in open map
        if (n < current) {
          // but new path is better -> update
          Node open_node = *open_query;
          open.erase(open_node);
          parent_map.erase(open_node);

          open.insert(n);
          parent_map[n] = current;
          continue;
        }
      }

      // check if closed
      auto closed_query = closed.find(n);
      if (closed_query == closed.end()) {
        // not in closed map -> open
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }
  std::cout << "PATH DOES NOT EXIST!" << std::endl;
  return std::vector<octomap::point3d>();
}
//}

/* findPath overload //{ */
std::vector<octomap::point3d> AstarPlanner::findPath(const octomap::point3d &start, const octomap::point3d &goal,
                                                     std::shared_ptr<octomap::OcTree> mapping_tree) {
  auto start_key = mapping_tree->coordToKey(start);
  auto goal_key  = mapping_tree->coordToKey(goal);
  return findPath(start_key, goal_key, mapping_tree);
}
//}

/* getNodeDepth //{ */
double AstarPlanner::getNodeDepth(const octomap::OcTreeKey &key, octomap::OcTree &tree) {
  for (auto it = tree.begin(); it != tree.end(); it++) {
    if (it.getKey() == key) {
      return it.getDepth();
    }
  }
  return tree.getTreeDepth();
}
//}

/* getNeighborKeys //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree) {
  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {
    auto newkey    = expand(key, d, tree);
    auto tree_node = tree.search(newkey);
    if (tree_node != NULL) {
      // free cell
      neighbors.push_back(newkey);
    }
  }

  return neighbors;
}
//}

/* expand //{ */
octomap::OcTreeKey AstarPlanner::expand(const octomap::OcTreeKey &key, const octomap::point3d &direction, octomap::OcTree &tree) {
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
double AstarPlanner::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {
  return (p1 - p2).norm();
}

double AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree) {
  double voxel_dist = std::sqrt(std::pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));
  return voxel_dist * tree.getResolution();
}
//}

/* freeStraightPath //{ */
bool AstarPlanner::freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree) {

  octomap::KeyRay ray;
  tree.computeRayKeys(p1, p2, ray);
  for (auto &k : ray) {
    auto tree_node = tree.search(k);
    if (tree_node == NULL) {
      // Path may exist, but goes through unknown cells
      return false;
    } else if (tree_node->getValue() == TreeValue::OCCUPIED) {
      // Path goes through occupied cells
      return false;
    }
  }
  return true;
}
//}

/* backtrackPathKeys //{ */
std::vector<octomap::OcTreeKey> AstarPlanner::backtrackPathKeys(const Node &from, const Node &to, std::unordered_map<Node, Node, HashFunction> &parent_map) {
  std::vector<octomap::OcTreeKey> keys;

  Node current = from;
  while (current != to) {
    keys.push_back(current.key);
    current = parent_map.find(current)->second;  // get parent
  }
  // reverse order
  std::reverse(keys.begin(), keys.end());
  return keys;
}
//}

/* keysToCoords //{ */
std::vector<octomap::point3d> AstarPlanner::keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree) {
  std::vector<octomap::point3d> coords;
  for (auto &k : keys) {
    coords.push_back(tree.keyToCoord(k));
  }
  return coords;
}
//}

/* euclideanDistanceTransform //{ */
DynamicEDTOctomap AstarPlanner::euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree) {
  double x, y, z;
  tree->getMetricMin(x, y, z);
  octomap::point3d metric_min(x, y, z);
  tree->getMetricMax(x, y, z);
  octomap::point3d  metric_max(x, y, z);
  DynamicEDTOctomap edf(euclidean_distance_cutoff, tree.get(), metric_min, metric_max, unknown_is_occupied);
  edf.update();
  return edf;
}
//}

/* createPlanningTree //{ */
octomap::OcTree AstarPlanner::createPlanningTree(std::shared_ptr<octomap::OcTree> tree, const octomap::OcTreeKey &start) {

  auto            edf         = euclideanDistanceTransform(tree);
  octomap::OcTree binary_tree = octomap::OcTree(*tree);
  for (auto it = binary_tree.begin(); it != binary_tree.end(); it++) {
    if (edf.getDistance(it.getCoordinate()) <= safe_obstacle_distance) {
      if (distEuclidean(it.getKey(), start, *tree) <= clearing_radius) {
        it->setValue(TreeValue::FREE);  // set cells around the start to free (prevent getting stuck near walls)
      } else {
        it->setValue(TreeValue::OCCUPIED);  // obstacle or close to obstacle
      }
    } else {
      it->setValue(TreeValue::FREE);  // free and safe
    }
  }
  binary_tree.prune();
  return binary_tree;
}
//}

#ifdef VISUALIZE
/* visualizeTreeCubes //{ */
void AstarPlanner::visualizeTreeCubes(octomap::OcTree &tree, bool show_unoccupied) {

  for (auto it = tree.begin(); it != tree.end(); it++) {
    Eigen::Vector3d           center(it.getX(), it.getY(), it.getZ());
    double                    cube_scale  = tree.getResolution() * std::pow(2, tree.getTreeDepth() - it.getDepth());
    Eigen::Vector3d           size        = Eigen::Vector3d(1, 1, 1) * cube_scale;
    Eigen::Quaterniond        orientation = Eigen::Quaterniond::Identity();
    mrs_lib::geometry::Cuboid c(center, size, orientation);
    if (it->getValue() == TreeValue::OCCUPIED) {
      bv.addCuboid(c, 0.1, 0.1, 0.1, 0.8, true);
      /* bv.addCuboid(c, 0.1, 0.1, 0.1, 1.0, false); */
      continue;
    }
    if (show_unoccupied && it->getValue() == TreeValue::FREE) {
      bv.addCuboid(c, 0.9, 0.9, 0.9, 0.8, true);
      /* bv.addCuboid(c, 0.8, 0.8, 0.8, 1.0, false); */
    }
  }
}
//}

/* visualizeExpansions //{ */
void AstarPlanner::visualizeExpansions(std::set<Node, CostComparator> open, std::unordered_set<Node, HashFunction> closed, octomap::OcTree &tree) {
  for (auto &n : open) {
    auto            coord = tree.keyToCoord(n.key);
    Eigen::Vector3d p(coord.x(), coord.y(), coord.z());
    bv.addPoint(p, 0.2, 1.0, 0.2, 1.0);
  }
  /* for (auto &n : closed) { */
  /*   auto                      coord = tree.keyToCoord(n.key); */
  /*   Eigen::Vector3d           center(coord.x(), coord.y(), coord.z()); */
  /*   double                    cube_scale  = tree.getResolution() * std::pow(2, tree.getTreeDepth() - getNodeDepth(n.key, tree)); */
  /*   Eigen::Vector3d           size        = Eigen::Vector3d(1, 1, 1) * cube_scale; */
  /*   Eigen::Quaterniond        orientation = Eigen::Quaterniond::Identity(); */
  /*   mrs_lib::geometry::Cuboid c(center, size, orientation); */
  /*   bv.addCuboid(c, 0.6, 0.0, 0.0, 1.0, true); */
  /* } */
}
//}

#endif

}  // namespace pathfinder
