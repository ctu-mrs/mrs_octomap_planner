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

bool LeafComparator::operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const {
  return l1.second < l2.second;
}

/* AstarPlanner constructor //{ */
#ifndef VISUALIZE
AstarPlanner::AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty,
                           double greedy_penalty, double timeout_threshold, double max_waypoint_distance, bool unknown_is_occupied) {
  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->planning_tree_resolution  = planning_tree_resolution;
  this->distance_penalty          = distance_penalty;
  this->greedy_penalty            = greedy_penalty;
  this->timeout_threshold         = timeout_threshold;
  this->max_waypoint_distance     = max_waypoint_distance;
  this->unknown_is_occupied       = unknown_is_occupied;
}
#endif

#ifdef VISUALIZE
AstarPlanner::AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty,
                           double greedy_penalty, double timeout_threshold, double max_waypoint_distance, bool unknown_is_occupied,
                           mrs_lib::BatchVisualizer &bv) {
  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->planning_tree_resolution  = planning_tree_resolution;
  this->distance_penalty          = distance_penalty;
  this->greedy_penalty            = greedy_penalty;
  this->timeout_threshold         = timeout_threshold;
  this->max_waypoint_distance     = max_waypoint_distance;
  this->unknown_is_occupied       = unknown_is_occupied;
  this->bv                        = bv;
}
#endif
//}

/* findPath main //{ */
std::vector<octomap::point3d> AstarPlanner::findPath(const octomap::point3d &start_coord, const octomap::point3d &goal_coord,
                                                     std::shared_ptr<octomap::OcTree> mapping_tree) {

  auto time_start = ros::Time::now();

  std::pair<octomap::OcTree, std::vector<octomap::point3d>> tree_with_tunnel = createPlanningTree(mapping_tree, start_coord, planning_tree_resolution);

  auto tree   = tree_with_tunnel.first;
  auto tunnel = tree_with_tunnel.second;

  std::vector<octomap::OcTreeKey> leaf_keys;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); it++) {
    leaf_keys.push_back(it.getKey());
  }

  auto map_goal  = goal_coord;
  auto map_query = mapping_tree->search(goal_coord);
  if (map_query == NULL) {
    ROS_INFO("[%s]: Goal is outside of map", ros::this_node::getName().c_str());
    map_goal = generateTemporaryGoal(start_coord, goal_coord, tree);
    ROS_INFO("[%s]: Generated a temporary goal: [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), map_goal.x(), map_goal.y(), map_goal.z());
  } else if (map_query->getValue() == TreeValue::OCCUPIED) {
    ROS_INFO("[%s]: Goal is inside an inflated obstacle", ros::this_node::getName().c_str());
    map_goal = nearestFreeCoord(goal_coord, start_coord, tree);
    ROS_INFO("[%s]: Generated a replacement goal: [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), map_goal.x(), map_goal.y(), map_goal.z());
  }

#ifdef VISUALIZE
  /* bv.clearBuffers(); */
  /* visualizeTreeCubes(tree, true); */
  /* /1* /2* visualizeTreePoints(tree, true); *2/ *1/ */
  /* bv.publish(); */
  /* /1* return std::vector<octomap::point3d>(); *1/ */
#endif

  std::set<Node, CostComparator>               open;
  std::unordered_set<Node, HashFunction>       closed;
  std::unordered_map<Node, Node, HashFunction> parent_map;  // first = child, second = parent

  octomap::OcTreeKey start;
  if (tunnel.empty()) {
    start = tree.coordToKey(start_coord);
  } else {
    start = tree.coordToKey(tunnel.back());
  }

  auto planning_start = tree.keyToCoord(start);
  auto goal           = tree.coordToKey(map_goal);

  std::cout << "Start planning from: " << planning_start.x() << ", " << planning_start.y() << ", " << planning_start.z() << std::endl;

  Node first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distEuclidean(start, goal, tree);
  first.total_cost = first.cum_dist + first.goal_dist;
  open.insert(first);

  Node best_node = first;

  while (!open.empty() && ros::ok()) {

    auto time_now = ros::Time::now();
    if (time_now.toSec() - time_start.toSec() > timeout_threshold) {
      ROS_WARN("[%s]: Planning timeout! Using current best node as goal.", ros::this_node::getName().c_str());
      auto path_keys = backtrackPathKeys(best_node, first, parent_map);
      ROS_INFO("[%s]: Path found. Length: %ld", ros::this_node::getName().c_str(), path_keys.size());
      bv.clearBuffers();
      visualizeTreeCubes(tree, false);
      bv.publish();
      return postprocessPath(keysToCoords(path_keys, tree), tree);
      /* return keysToCoords(path_keys, tree); */
    }

#ifdef VISUALIZE
    /* bv.clearBuffers(); */
    /* visualizeExpansions(open, closed, tree); */
    /* bv.publish(); */
#endif

    auto current = *open.begin();
    open.erase(current);
    closed.insert(current);

    auto current_coord = tree.keyToCoord(current.key);
    /* std::cout << "Current coord: " << current_coord.x() << ", " << current_coord.y() << ", " << current_coord.z() << std::endl; */

    if (distEuclidean(current_coord, map_goal) < 0.2) {
      auto path_keys = backtrackPathKeys(current, first, parent_map);
      path_keys.push_back(tree.coordToKey(map_goal));
      ROS_INFO("[%s]: Path found. Length: %ld", ros::this_node::getName().c_str(), path_keys.size());
      bv.clearBuffers();
      visualizeTreeCubes(tree, false);
      bv.publish();
      return postprocessPath(keysToCoords(path_keys, tree), tree);
      /* return keysToCoords(path_keys, tree); */
    }

    // expand
    auto neighbors = getNeighborhood(current.key, tree);


    for (auto &nkey : neighbors) {

      if (std::find(leaf_keys.begin(), leaf_keys.end(), nkey) != leaf_keys.end()) {
        // skip leafs
        continue;
      }

      auto ncoord = tree.keyToCoord(nkey);
      Node n;
      n.key = nkey;
      // check if open
      auto open_query = open.find(n);
      if (open_query != open.end()) {
        // in open map

        n.goal_dist  = distEuclidean(nkey, goal, tree);
        n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, tree);
        n.total_cost = n.goal_dist + n.cum_dist;
        if (n < best_node) {
          best_node = n;
        }

        if (n < current) {
          // new path is better -> update
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

        n.goal_dist  = distEuclidean(nkey, goal, tree);
        n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, tree);
        n.total_cost = distance_penalty * n.cum_dist + greedy_penalty * n.goal_dist;
        if (n < best_node) {
          best_node = n;
        }
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }
  std::cout << "PATH DOES NOT EXIST!" << std::endl;
  return std::vector<octomap::point3d>();
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
      // free cell?
      if (tree_node->getValue() == TreeValue::FREE) {
        neighbors.push_back(newkey);
      }
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
  auto tree_query = tree.search(k);
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
  keys.push_back(current.key);

  while (current != to) {
    current = parent_map.find(current)->second;  // get parent
    keys.push_back(current.key);
  };
  keys.push_back(to.key);

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
std::pair<octomap::OcTree, std::vector<octomap::point3d>> AstarPlanner::createPlanningTree(std::shared_ptr<octomap::OcTree> tree, const octomap::point3d &start,
                                                                                           double resolution) {

  auto            edf         = euclideanDistanceTransform(tree);
  octomap::OcTree binary_tree = octomap::OcTree(resolution);
  tree->expand();
  for (auto it = tree->begin(); it != tree->end(); it++) {
    if (edf.getDistance(it.getCoordinate()) <= safe_obstacle_distance) {
      binary_tree.setNodeValue(it.getCoordinate(), TreeValue::OCCUPIED);  // obstacle or close to obstacle
    } else {
      binary_tree.setNodeValue(it.getCoordinate(), TreeValue::FREE);  // free and safe
    }
  }

  std::vector<octomap::point3d> tunnel;

  octomap::point3d current_coords    = start;
  auto             binary_tree_query = binary_tree.search(current_coords);
  if (binary_tree_query != NULL && binary_tree_query->getValue() != TreeValue::FREE) {
    std::cout << "Start is inside an inflated obstacle. Tunneling out..." << std::endl;
    // tunnel out of expanded walls
    while (ros::ok() && binary_tree_query != NULL) {
      tunnel.push_back(current_coords);
      binary_tree.setNodeValue(current_coords, TreeValue::FREE);
      float            obstacle_dist;
      octomap::point3d closest_obstacle;
      edf.getDistanceAndClosestObstacle(current_coords, obstacle_dist, closest_obstacle);
      octomap::point3d dir_away_from_obstacle = current_coords - closest_obstacle;
      dir_away_from_obstacle.z()              = 0.0f;
      if (obstacle_dist >= safe_obstacle_distance) {
        std::cout << "Tunnel created with " << tunnel.size() << " points" << std::endl;
        break;
      }
      current_coords += dir_away_from_obstacle.normalized() * binary_tree.getResolution();
      while (binary_tree.search(current_coords) == binary_tree_query) {
        current_coords += dir_away_from_obstacle.normalized() * binary_tree.getResolution();
      }
      binary_tree_query = binary_tree.search(current_coords);
    }
  }
  return {binary_tree, tunnel};
}
//}

/* nearestFreeCoord //{ */
octomap::point3d AstarPlanner::nearestFreeCoord(const octomap::point3d &p, const octomap::point3d &uav_pos, octomap::OcTree &tree) {
  auto query = tree.search(p);
  if (query != NULL && query->getValue() == TreeValue::FREE) {
    return p;
  }
  auto neighbors = getNeighborhood(tree.coordToKey(p), tree);
  for (auto &n : neighbors) {
    auto query = tree.search(n);
    if (query != NULL && query->getValue() == TreeValue::FREE) {
      return tree.keyToCoord(n);
    }
  }
  // no free neighbor -> try a point closer to UAV
  octomap::point3d dir_to_uav;
  dir_to_uav = (uav_pos - p).normalized() * tree.getResolution();
  auto new_p = p + dir_to_uav;
  return nearestFreeCoord(p + dir_to_uav, uav_pos, tree);
}
//}

/* postprocessPath //{ */
std::vector<octomap::point3d> AstarPlanner::postprocessPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree) {

  if (waypoints.size() < 2) {
    std::cout << "Not enough points for postprocessing!" << std::endl;
    return waypoints;
  }


  std::vector<octomap::point3d> padded         = waypoints;
  size_t                        waypoints_size = waypoints.size();

  /* padding with additional points if the distances exceed threshold //{ */
  for (int i = 1; i < waypoints_size; i++) {
    if (distEuclidean(padded[i], padded[i - 1]) > max_waypoint_distance) {
      auto direction = (padded[i] - padded[i - 1]).normalized() * max_waypoint_distance;
      padded.insert(padded.begin() + i, padded[i - 1] + direction);
      waypoints_size++;
    }
  }
  //}

  if (padded.size() < 3) {
    std::cout << "Not enough points for postprocessing!" << std::endl;
    return padded;
  }

  std::vector<octomap::point3d> filtered;

  /* removing obsolete points //{ */
  filtered.push_back(padded.front());
  int i = 2;
  while (i < padded.size()) {
    if (!freeStraightPath(filtered.back(), padded[i], tree)) {
      filtered.push_back(padded[i - 1]);
    }
    i++;
  }
  filtered.push_back(padded.back());
  //}

  return filtered;
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

/* visualizeTreePoints //{ */
void AstarPlanner::visualizeTreePoints(octomap::OcTree &tree, bool show_unoccupied) {
  for (auto it = tree.begin(); it != tree.end(); it++) {
    Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
    if (it->getValue() == TreeValue::OCCUPIED) {
      bv.addPoint(p, 0.1, 0.1, 0.1, 1.0);
      /* bv.addCuboid(c, 0.1, 0.1, 0.1, 1.0, false); */
      continue;
    }
    if (show_unoccupied && it->getValue() == TreeValue::FREE) {
      bv.addPoint(p, 0.9, 0.9, 0.9, 0.8);
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
  for (auto &n : closed) {
    auto            coord = tree.keyToCoord(n.key);
    Eigen::Vector3d p(coord.x(), coord.y(), coord.z());
    bv.addPoint(p, 1.0, 0.2, 0.2, 1.0);
  }
}
//}

/* generateTemporaryGoal //{ */
octomap::point3d AstarPlanner::generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, octomap::OcTree &tree) {

  octomap::point3d temp_goal;

  // try to explore unknown cells
  std::set<std::pair<octomap::OcTree::iterator, double>, LeafComparator> leafs;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); it++) {
    if (it->getValue() == TreeValue::OCCUPIED) {
      continue;
    }
    auto k = it.getKey();
    k.k[2] += 1;
    if (tree.search(k) == NULL) {
      continue;
    }
    k.k[2] -= 2;
    if (tree.search(k) == NULL) {
      continue;
    }

    leafs.insert({it, distEuclidean(it.getCoordinate(), goal)});
  }
  // sort free nodes on the map edge by their distance from goal
  if (!leafs.empty()) {
    // select the closest point
    return leafs.begin()->first.getCoordinate();
  }

  // solution that is only good for smaller obstacles
  octomap::KeyRay ray;
  tree.computeRayKeys(start, goal, ray);
  for (auto &k : ray) {
    auto coords = tree.keyToCoord(k);
    if (tree.search(coords) != NULL && tree.search(coords)->getValue() == TreeValue::FREE) {
      temp_goal = coords;
    }
  }
  return temp_goal;
}
//}

#endif

}  // namespace pathfinder
