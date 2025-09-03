#include <astar_planner.hpp>

namespace mrs_octomap_planner
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

bool Node::operator<=(const Node &other) const {

  if (total_cost == other.total_cost) {
    return goal_dist <= other.goal_dist;
  }

  return total_cost <= other.total_cost;
}

bool CostComparator::operator()(const Node &n1, const Node &n2) const {

  if (n1.total_cost == n2.total_cost) {
    return n1.goal_dist > n2.goal_dist;
  }

  return n1.total_cost > n2.total_cost;
}

bool HashFunction::operator()(const Node &n) const {
  using std::hash;
  return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
}

bool LeafComparator::operator()(const std::pair<octomap::OcTree::iterator, double> &l1, const std::pair<octomap::OcTree::iterator, double> &l2) const {
  return l1.second < l2.second;
}

/* AstarPlanner constructor //{ */
AstarPlanner::AstarPlanner(double safe_obstacle_distance, double euclidean_distance_cutoff, double submap_distance, double planning_tree_resolution,
                           double distance_penalty, double greedy_penalty, double timeout_threshold, double max_waypoint_distance, double min_altitude,
                           double max_altitude, bool unknown_is_occupied, std::shared_ptr<mrs_lib::BatchVisualizer> bv) {

  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->submap_distance           = submap_distance;
  this->planning_tree_resolution  = planning_tree_resolution;
  this->distance_penalty          = distance_penalty;
  this->greedy_penalty            = greedy_penalty;
  this->timeout_threshold         = timeout_threshold;
  this->max_waypoint_distance     = max_waypoint_distance;
  this->min_altitude              = min_altitude;
  this->max_altitude              = max_altitude;
  this->unknown_is_occupied       = unknown_is_occupied;
  this->bv                        = bv;
}
//}

/* findPath main //{ */

std::pair<std::vector<octomap::point3d>, bool> AstarPlanner::findPath(const octomap::point3d &start_coord, const octomap::point3d &goal_coord,
                                                                      std::shared_ptr<octomap::OcTree> mapping_tree, const double timeout) {

  ROS_INFO("[Astar]: Astar: user goal [%.2f, %.2f, %.2f]", goal_coord.x(), goal_coord.y(), goal_coord.z());

  auto time_start = ros::Time::now();

  this->timeout_threshold = timeout;

  ros::Time time_start_planning_tree = ros::Time::now();
  auto      tree_with_tunnel         = createPlanningTree(mapping_tree, start_coord, planning_tree_resolution, start_coord, 10.0);
  ROS_INFO_THROTTLE(1.0, "[Astar]: the planning tree took %.4f s to create", (ros::Time::now() - time_start_planning_tree).toSec());

  if (!tree_with_tunnel) {
    ROS_WARN_THROTTLE(1.0, "[Astar]: could not create a planning tree");
    return {std::vector<octomap::point3d>(), false};
  }

  octomap::OcTree &tree   = *tree_with_tunnel.value().first;
  auto             tunnel = tree_with_tunnel.value().second;

  auto map_goal  = goal_coord;
  auto map_query = mapping_tree->search(goal_coord);

  bool original_goal = true;

  if (map_query == NULL) {
    ROS_INFO("[Astar]: Goal is outside of map");
    map_goal = generateTemporaryGoal(start_coord, goal_coord, tree);
    ROS_INFO("[Astar]: Generated a temporary goal: [%.2f, %.2f, %.2f]", map_goal.x(), map_goal.y(), map_goal.z());
    original_goal = false;
  } else if (map_query->getValue() == TreeValue::OCCUPIED) {
    ROS_INFO("[Astar:] Goal is inside an inflated obstacle");
    map_goal = nearestFreeCoord(goal_coord, start_coord, tree);
    ROS_INFO("[Astar]: Generated a replacement goal: [%.2f, %.2f, %.2f]", map_goal.x(), map_goal.y(), map_goal.z());
    original_goal = false;
  }

  mrs_lib::geometry::Cuboid c_start(Eigen::Vector3d(start_coord.x(), start_coord.y(), start_coord.z()), Eigen::Vector3d(0.3, 0.3, 0.3),
                                    Eigen::Quaterniond::Identity());
  mrs_lib::geometry::Cuboid c_goal(Eigen::Vector3d(map_goal.x(), map_goal.y(), map_goal.z()), Eigen::Vector3d(0.3, 0.3, 0.3), Eigen::Quaterniond::Identity());
  bv->addCuboid(c_start, 0.9, 0.6, 0.1, 1);
  bv->addCuboid(c_goal, 0.1, 0.9, 0.6, 1);

  std::priority_queue<Node, std::vector<Node>, CostComparator> open_heap;
  std::unordered_set<Node, HashFunction>                       open;
  std::unordered_set<Node, HashFunction>                       closed;
  std::unordered_map<Node, Node, HashFunction>                 parent_map;  // first = child, second = parent

  octomap::OcTreeKey start;
  if (tunnel.empty()) {
    start = tree.coordToKey(start_coord);
  } else {
    start = tree.coordToKey(tunnel.back());
  }

  auto planning_start = tree.keyToCoord(start);
  auto goal           = tree.coordToKey(map_goal);

  if (distEuclidean(planning_start, map_goal) <= 2 * planning_tree_resolution) {

    ROS_INFO("[Astar]: Path special case, we are there");

    bv->clearVisuals();
    bv->clearBuffers();
    visualizeTreeCubes(tree, true);
    visualizeGoal(goal_coord);
    visualizeExpansions(open, closed, tree);
    bv->publish();

    return {std::vector<octomap::point3d>(), false};
  }

  ROS_INFO_STREAM("[Astar]: Planning from: " << planning_start.x() << ", " << planning_start.y() << ", " << planning_start.z());
  ROS_INFO_STREAM("[Astar]: Planning to: " << map_goal.x() << ", " << map_goal.y() << ", " << map_goal.z());

  auto time_start_planning = ros::Time::now();

  Node first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distEuclidean(start, goal, tree);
  first.total_cost = first.cum_dist + first.goal_dist;
  open_heap.push(first);
  open.insert(first);

  Node best_node        = first;
  Node best_node_greedy = first;

  int iter = 0;

  Node last_closed;

  while (!open.empty() && ros::ok()) {

    Node current = open_heap.top();
    open_heap.pop();
    open.erase(current);
    closed.insert(current);

    last_closed = current;

    auto time_now = ros::Time::now();

    if (time_now.toSec() - time_start.toSec() > timeout_threshold) {

      ROS_WARN("[Astar]: Planning timeout (%.4f s after start of search)! Using current best node as goal.", (ros::Time::now() - time_start_planning).toSec());
      auto path_keys = backtrackPathKeys(best_node == first ? best_node_greedy : best_node, first, parent_map);
      ROS_INFO("[Astar]: Path found. Length: %ld", path_keys.size());

      bv->clearVisuals();
      bv->clearBuffers();
      visualizeTreeCubes(tree, true);
      visualizeGoal(goal_coord);
      visualizeExpansions(open, closed, tree);
      bv->publish();

      return std::make_pair(prepareOutputPath(path_keys, tree), false);
    }

    auto current_coord = tree.keyToCoord(current.key);

    if (distEuclidean(current_coord, map_goal) <= 2 * planning_tree_resolution) {

      auto path_keys = backtrackPathKeys(current, first, parent_map);
      path_keys.push_back(tree.coordToKey(map_goal));
      ROS_INFO("[Astar]: Path found. Length: %ld. Search time: %.4f", path_keys.size(), (ros::Time::now() - time_start_planning).toSec());

      bv->clearVisuals();
      bv->clearBuffers();
      visualizeTreeCubes(tree, true);
      visualizeGoal(goal_coord);
      visualizeExpansions(open, closed, tree);
      bv->publish();

      return {prepareOutputPath(path_keys, tree), original_goal};
    }

    // expand
    auto neighbors = getNeighborhood(current.key, tree);

    /* ROS_INFO_STREAM("poped " << current.key.k); */
    /* ROS_INFO("[%s]: iter %d, open %d, closed %d, neighbours %d", ros::this_node::getName().c_str(), iter++, open.size(), closed.size(), neighbors.size()); */

    for (auto &nkey : neighbors) {

      /* ROS_INFO_STREAM("key" << nkey.k); */

      auto ncoord = tree.keyToCoord(nkey);
      Node n;
      n.key = nkey;

      auto closed_query = closed.find(n);
      auto open_query   = open.find(n);

      // in open map
      n.goal_dist  = distEuclidean(nkey, goal, tree);
      n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, tree);
      n.total_cost = greedy_penalty * n.goal_dist + distance_penalty * n.cum_dist;

      if (closed_query == closed.end() && open_query == open.end()) {

        if (n <= best_node) {
          best_node = n;
        }

        if (n.goal_dist <= best_node_greedy.goal_dist) {
          best_node_greedy = n;
        }

        open_heap.push(n);
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }

  bv->clearVisuals();
  bv->clearBuffers();
  visualizeTreeCubes(tree, true);
  visualizeGoal(goal_coord);
  visualizeExpansions(open, closed, tree);
  bv->publish();

  if (best_node != first) {

    auto path_keys = backtrackPathKeys(best_node, first, parent_map);

    ROS_INFO("[Astar]: direct path does not exist, going to the 'best_node', search time: %.4f", (ros::Time::now() - time_start_planning).toSec());

    return std::make_pair(prepareOutputPath(path_keys, tree), false);
  }

  if (best_node_greedy != first) {

    auto path_keys = backtrackPathKeys(best_node_greedy, first, parent_map);

    ROS_INFO("[Astar]: direct path does not exist, going to the best_node_greedy', search time: %.4f", (ros::Time::now() - time_start_planning).toSec());

    return std::make_pair(prepareOutputPath(path_keys, tree), false);
  }

  ROS_WARN("[Astar]: PATH DOES NOT EXIST! Search time: %.4f", (ros::Time::now() - time_start_planning).toSec());

  return {std::vector<octomap::point3d>(), false};
}
//}

/* getNeighborhood() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::getNeighborhood(const octomap::OcTreeKey &key, octomap::OcTree &tree) {

  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {

    auto newkey    = expand(key, d, tree);
    auto tree_node = tree.search(newkey);

    if (tree_node != NULL) {
      // free cell?
      if (tree_node->getValue() == TreeValue::FREE && tree.keyToCoord(newkey).z() >= min_altitude && tree.keyToCoord(newkey).z() <= max_altitude) {
        neighbors.push_back(newkey);
      }
    }
  }

  return neighbors;
}

//}

/* expand() //{ */

octomap::OcTreeKey AstarPlanner::expand(const octomap::OcTreeKey &key, const std::vector<int> &direction, octomap::OcTree &tree) {

  auto prev_node = tree.search(key);

  octomap::OcTreeKey k;

  k.k[0] = key.k[0] + direction[0];
  k.k[1] = key.k[1] + direction[1];
  k.k[2] = key.k[2] + direction[2];

  return k;
}

//}

/* distEuclidean() //{ */

double AstarPlanner::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {

  return (p1 - p2).norm();
}

double AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::OcTree &tree) {

  double voxel_dist = sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));

  return voxel_dist * tree.getResolution();
}

//}

/* freeStraightPath() //{ */

bool AstarPlanner::freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::OcTree &tree) {

  octomap::KeyRay ray;
  tree.computeRayKeys(p1, p2, ray);

  for (auto &k : ray) {

    auto tree_node = tree.search(k);

    if (tree_node == NULL) {
      // Path may exist, but goes through unknown cells
      return false;
    }

    if (tree_node->getValue() == TreeValue::OCCUPIED) {
      // Path goes through occupied cells
      return false;
    }

    /* if (max_waypoint_distance > 0 && (p1 - p2).norm() > max_waypoint_distance) { */
    /*   return false; */
    /* } */
  }

  return true;
}

//}

/* backtrackPathKeys() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::backtrackPathKeys(const Node &from, const Node &to, std::unordered_map<Node, Node, HashFunction> &parent_map) {

  std::vector<octomap::OcTreeKey> keys;

  Node current = from;
  keys.push_back(current.key);

  while (current.key != to.key) {
    current = parent_map.find(current)->second;
    keys.push_back(current.key);
  };

  keys.push_back(to.key);

  // reverse order
  std::reverse(keys.begin(), keys.end());
  return keys;
}

//}

/* keysToCoords() //{ */

std::vector<octomap::point3d> AstarPlanner::keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::OcTree &tree) {

  std::vector<octomap::point3d> coords;

  for (auto &k : keys) {
    coords.push_back(tree.keyToCoord(k));
  }

  return coords;
}

//}

/* euclideanDistanceTransform() //{ */

DynamicEDTOctomap AstarPlanner::euclideanDistanceTransform(std::shared_ptr<octomap::OcTree> tree, const octomap::point3d &orig_coord) {

  double x, y, z;

  tree->getMetricMin(x, y, z);
  octomap::point3d metric_min(x, y, z);

  tree->getMetricMax(x, y, z);
  octomap::point3d metric_max(x, y, z);

  DynamicEDTOctomap edf(float(euclidean_distance_cutoff), tree.get(), metric_min, metric_max, unknown_is_occupied);
  edf.update();

  return edf;
}

//}

/* createPlanningTree() //{ */

std::optional<std::pair<std::shared_ptr<octomap::OcTree>, std::vector<octomap::point3d>>> AstarPlanner::createPlanningTree(
    std::shared_ptr<octomap::OcTree> tree, const octomap::point3d &start, double resolution, const octomap::point3d &orig_coord, double radius) {

  /* resample the incoming map to the desired resolution //{ */

  auto time_start = ros::Time::now();

  std::shared_ptr<octomap::OcTree> resampled_tree = std::make_shared<octomap::OcTree>(resolution);
  resampled_tree->setOccupancyThres(tree->getOccupancyThres());
  resampled_tree->setProbHit(tree->getProbHit());
  resampled_tree->setProbMiss(tree->getProbMiss());
  resampled_tree->setClampingThresMax(tree->getClampingThresMax());
  resampled_tree->setClampingThresMin(tree->getClampingThresMin());

  octomap::OcTreeKey key = resampled_tree->coordToKey(0, 0, 0, resampled_tree->getTreeDepth());
  resampled_tree->setNodeValue(key, 0.0);

  octomap::point3d p_min(start.x() - submap_distance, start.y() - submap_distance, start.z() - submap_distance);
  octomap::point3d p_max(start.x() + submap_distance, start.y() + submap_distance, start.z() + submap_distance);

  for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(p_min, p_max, tree->getTreeDepth()), end = tree->end_leafs_bbx(); it != end; ++it) {

    auto orig_key = it.getKey();

    const unsigned int old_depth = it.getDepth();
    const unsigned int new_depth = old_depth;

    auto new_key = resampled_tree->coordToKey(it.getX(), it.getY(), it.getZ());

    octomap::OcTreeNode *new_node = touchNode(resampled_tree, new_key, new_depth);

    if (tree->isNodeOccupied(*it)) {
      new_node->setLogOdds(1.0);
    } else {
      new_node->setLogOdds(-1.0);
    }
  }

  resampled_tree->expand();

  ROS_INFO("[%s]: planning tree resampling took %.4f", ros::this_node::getName().c_str(), (ros::Time::now() - time_start).toSec());
  time_start = ros::Time::now();

  /* ROS_ERROR("[%s]: Resampled tree size after expand = %lu, free = %d, occupied = %d.", ros::this_node::getName().c_str(), resampled_tree->size(),
   * counter_free, counter_occ); */

  /* ROS_ERROR("[%s]: Orig coord = [%.2f, %.2f, %.2f].", ros::this_node::getName().c_str(), orig_coord.x(), orig_coord.y(), orig_coord.z()); */
  auto edf = euclideanDistanceTransform(resampled_tree, orig_coord);

  ROS_INFO("[%s]: edf over planning tree took %.4f", ros::this_node::getName().c_str(), (ros::Time::now() - time_start).toSec());
  time_start = ros::Time::now();

  ROS_INFO("[%s]: edf over planning tree took %.4f", ros::this_node::getName().c_str(), (ros::Time::now() - time_start).toSec());
  time_start = ros::Time::now();

  //}

  std::shared_ptr<octomap::OcTree> binary_tree = std::make_shared<octomap::OcTree>(resolution);

  for (auto it = resampled_tree->begin(); it != resampled_tree->end(); it++) {
    if (edf.getDistance(it.getCoordinate()) <= safe_obstacle_distance) {
      binary_tree->setNodeValue(it.getCoordinate(), TreeValue::OCCUPIED);  // occupied
    } else {
      binary_tree->setNodeValue(it.getCoordinate(), TreeValue::FREE);  // free and safe
    }
  }

  ROS_INFO("[%s]: setting node values in planning tree took %.4f", ros::this_node::getName().c_str(), (ros::Time::now() - time_start).toSec());
  time_start = ros::Time::now();
  /* ROS_ERROR("[%s]: Number of set node values based on edf = %d, free = %d, occupied = %d", ros::this_node::getName().c_str(), counter, counter_free,
   * counter_occ); */

  std::vector<octomap::point3d> tunnel;

  octomap::point3d current_coords    = start;
  auto             binary_tree_query = binary_tree->search(current_coords);

  if (binary_tree_query != NULL && binary_tree_query->getValue() != TreeValue::FREE) {

    ROS_WARN("[%s]: start is inside of an inflated obstacle, tunneling out", ros::this_node::getName().c_str());

    // tunnel out of expanded walls

    int iter1 = 0;

    while (ros::ok() && binary_tree_query != NULL && iter1++ <= 100) {

      if (iter1++ > 100) {
        return {};
      }

      tunnel.push_back(current_coords);
      binary_tree->setNodeValue(current_coords, TreeValue::FREE);

      float            obstacle_dist;
      octomap::point3d closest_obstacle;

      edf.getDistanceAndClosestObstacle(current_coords, obstacle_dist, closest_obstacle);
      octomap::point3d dir_away_from_obstacle = current_coords - closest_obstacle;

      if (obstacle_dist >= float(safe_obstacle_distance)) {
        ROS_INFO("[%s]: tunnel create with %d, final obstacle dist = %.4f", ros::this_node::getName().c_str(), int(tunnel.size()), obstacle_dist);
        break;
      }

      ROS_INFO("[%s]: binary tree resolution = %.2f", ros::this_node::getName().c_str(), float(binary_tree->getResolution()));
      current_coords += dir_away_from_obstacle.normalized() * float(binary_tree->getResolution());

      int iter2 = 0;

      while (binary_tree->search(current_coords) == binary_tree_query) {

        if (iter2++ > 100) {
          return {};
        }

        current_coords += dir_away_from_obstacle.normalized() * float(binary_tree->getResolution());

      }

      binary_tree_query = binary_tree->search(current_coords);
    }
  }

  ROS_INFO("[%s]: tunneling in planning tree took %.4f", ros::this_node::getName().c_str(), (ros::Time::now() - time_start).toSec());
  time_start = ros::Time::now();

  std::pair<std::shared_ptr<octomap::OcTree>, std::vector<octomap::point3d>> result = {binary_tree, tunnel};

  return result;
}

//}

/* nearestFreeCoord() //{ */

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
  dir_to_uav = (uav_pos - p).normalized() * float(tree.getResolution());
  auto new_p = p + dir_to_uav;

  return nearestFreeCoord(p + dir_to_uav, uav_pos, tree);
}
//}

/* postprocessPath() //{ */

std::vector<octomap::point3d> AstarPlanner::postprocessPath(const std::vector<octomap::point3d> &waypoints, octomap::OcTree &tree) {

  if (waypoints.size() < 2) {
    ROS_WARN("[Astar]: Not enough points for postprocessing!");
    return waypoints;
  }

  std::vector<octomap::point3d> padded         = waypoints;
  size_t                        waypoints_size = waypoints.size();

  /* padding with additional points if the distances exceed threshold //{ */

  for (int i = 1; i < waypoints_size; i++) {
    if (max_waypoint_distance > 0 && distEuclidean(padded[i], padded[i - 1]) > max_waypoint_distance) {
      auto direction = (padded[i] - padded[i - 1]).normalized() * float(max_waypoint_distance);
      padded.insert(padded.begin() + i, padded[i - 1] + direction);
      waypoints_size++;
    }
  }

  //}

  if (padded.size() < 3) {
    ROS_WARN("[Astar]: Not enough points for postprocessing!");
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

/* prepareOutputPath() //{ */

std::vector<octomap::point3d> AstarPlanner::prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::OcTree &tree) {

  auto waypoints = keysToCoords(keys, tree);

  // visualize raw planner output
  for (auto &w : waypoints) {
    Eigen::Vector3d wp(w.x(), w.y(), w.z());
    bv->addPoint(wp, 0.7, 0.2, 0.7, 1);
  }

  auto processed = postprocessPath(waypoints, tree);

  // visualize postprocessing output
  for (size_t i = 1; i < processed.size(); i++) {
    Eigen::Vector3d w0(processed[i - 1].x(), processed[i - 1].y(), processed[i - 1].z());
    Eigen::Vector3d w1(processed[i].x(), processed[i].y(), processed[i].z());
    bv->addRay(mrs_lib::geometry::Ray::twopointCast(w0, w1), 0.4, 0.4, 1.0, 1);
  }

  return processed;
}
//}

/* visualizeTreeCubes() //{ */

void AstarPlanner::visualizeTreeCubes(octomap::OcTree &tree, bool show_unoccupied) {

  for (auto it = tree.begin(); it != tree.end(); it++) {

    Eigen::Vector3d           center(it.getX(), it.getY(), it.getZ());
    double                    cube_scale  = tree.getResolution() * std::pow(2, tree.getTreeDepth() - it.getDepth());
    Eigen::Vector3d           size        = Eigen::Vector3d(1, 1, 1) * cube_scale;
    Eigen::Quaterniond        orientation = Eigen::Quaterniond::Identity();
    mrs_lib::geometry::Cuboid c(center, size, orientation);

    if (it->getValue() == TreeValue::OCCUPIED) {
      /* bv->addCuboid(c, 0.1, 0.5, 0.1, 0.2, true); */
      /* bv->addCuboid(c, 0, 0, 0, 0.3, false); */
      /* bv->addCuboid(c, 0.1, 0.1, 0.1, 0.8, true); */
      /* bv->addCuboid(c, 0.1, 0.1, 0.1, 1.0, false); */
    }

    if (show_unoccupied && it->getValue() == TreeValue::FREE) {
      /* bv->addCuboid(c, 0.5, 0.5, 0.5, 0.5, true); */
    }
  }
}

//}

/* visualizeGoal() //{ */

void AstarPlanner::visualizeGoal(const octomap::point3d &goal) {

  Eigen::Vector3d           center(goal.x(), goal.y(), goal.z());
  double                    cube_scale  = 0.5;
  Eigen::Vector3d           size        = Eigen::Vector3d(1, 1, 1) * cube_scale;
  Eigen::Quaterniond        orientation = Eigen::Quaterniond::Identity();
  mrs_lib::geometry::Cuboid c(center, size, orientation);

  bv->addCuboid(c, 1.0, 0.0, 1.0, 1.0, true);
}

//}

/* visualizeTreePoints() //{ */

void AstarPlanner::visualizeTreePoints(octomap::OcTree &tree, bool show_unoccupied) {

  for (auto it = tree.begin(); it != tree.end(); it++) {

    Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());

    if (it->getValue() == TreeValue::OCCUPIED) {
      bv->addPoint(p, 0.1, 0.1, 0.1, 1.0);
      /* bv->addCuboid(c, 0.1, 0.1, 0.1, 1.0, false); */
    }

    if (show_unoccupied && it->getValue() == TreeValue::FREE) {
      bv->addPoint(p, 0.9, 0.9, 0.9, 0.8);
      /* bv->addCuboid(c, 0.8, 0.8, 0.8, 1.0, false); */
    }
  }
}
//}

/* visualizeExpansions() //{ */

void AstarPlanner::visualizeExpansions(const std::unordered_set<Node, HashFunction> &open, const std::unordered_set<Node, HashFunction> &closed,
                                       octomap::OcTree &tree) {

  for (auto &n : open) {
    auto            coord = tree.keyToCoord(n.key);
    Eigen::Vector3d p(coord.x(), coord.y(), coord.z());
    bv->addPoint(p, 0.2, 1.0, 0.2, 0.3);
  }

  for (auto &n : closed) {
    auto            coord = tree.keyToCoord(n.key);
    Eigen::Vector3d p(coord.x(), coord.y(), coord.z());
    bv->addPoint(p, 1.0, 0.2, 0.2, 0.3);
  }
}

//}

/* generateTemporaryGoal() //{ */

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

/* touchNode() //{ */

octomap::OcTreeNode *AstarPlanner::touchNode(std::shared_ptr<octomap::OcTree> &octree, const octomap::OcTreeKey &key, unsigned int target_depth) {

  // initialize the tree by insertin a first node
  octomap::OcTreeKey temp_key = octree->coordToKey(0, 0, 0, octree->getTreeDepth());
  octree->setNodeValue(temp_key, 0.0);

  return touchNodeRecurs(octree, octree->getRoot(), key, 0, target_depth);
}

//}

/* touchNodeRecurs() //{ */

octomap::OcTreeNode *AstarPlanner::touchNodeRecurs(std::shared_ptr<octomap::OcTree> &octree, octomap::OcTreeNode *node, const octomap::OcTreeKey &key,
                                                   unsigned int depth, unsigned int max_depth) {

  if (depth < octree->getTreeDepth() && (max_depth == 0 || depth < max_depth)) {

    unsigned int pos = octomap::computeChildIdx(key, int(octree->getTreeDepth() - depth - 1));

    if (!octree->nodeChildExists(node, pos)) {

      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);

  } else {
    return node;
  }
}

//}

}  // namespace mrs_octomap_planner
