// some ros includes
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <nav_msgs/Odometry.h>

// some std includes
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/param_loader.h>

#include <mrs_msgs/Vec4.h>

#include <pathfinder/astar_planner.hpp>

// params
double euclidean_distance_cutoff;
double clearing_radius;
double safe_obstacle_distance;
double navigation_tolerance;
double explore_unknown_threshold;
double distance_penalty;
double greedy_penalty;
bool   unknown_is_occupied;

std::shared_ptr<octomap::OcTree> octree_;

// visualizer params
double points_scale;
double lines_scale;

// uav odometry
Eigen::Vector4d    uav_pos;  // [m]
Eigen::Quaterniond uav_ori;
double             uav_heading;

mrs_lib::BatchVisualizer bv;

// subscribers
ros::Subscriber    odom_subscriber;
ros::Subscriber    octomap_subscriber;
ros::ServiceServer goto_server;

// publishers
ros::Publisher grid_pub;

// timers
ros::Timer map_timer;

/* timerCallback //{ */
void timerCallback([[maybe_unused]] const ros::TimerEvent& evt) {
  bv.clearBuffers();
  bv.clearVisuals();

  // do stuff

  bv.publish();
}
//}

/* odomCallback //{ */
void odomCallback(const nav_msgs::Odometry& odom_in) {
  ROS_INFO_ONCE("[%s]: Getting odom", ros::this_node::getName().c_str());
  uav_pos.x() = odom_in.pose.pose.position.x;
  uav_pos.y() = odom_in.pose.pose.position.y;
  uav_pos.z() = odom_in.pose.pose.position.z;
  uav_ori.w() = odom_in.pose.pose.orientation.w;
  uav_ori.x() = odom_in.pose.pose.orientation.x;
  uav_ori.y() = odom_in.pose.pose.orientation.y;
  uav_ori.z() = odom_in.pose.pose.orientation.z;
  mrs_lib::AttitudeConverter ac(odom_in.pose.pose.orientation);
  uav_pos.w() = ac.getHeading();
}
//}

/* octomapCallback //{ */
void octomapCallback(const octomap_msgs::OctomapPtr octomap_in) {
  ROS_INFO_ONCE("[%s]: Getting octomap", ros::this_node::getName().c_str());
  auto tree_ptr = octomap_msgs::fullMsgToMap(*octomap_in);
  if (!tree_ptr) {
    ROS_WARN("[%s]: Octomap message is empty!", ros::this_node::getName().c_str());
  } else {
    octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));
  }
}
//}

/* gotoCallback //{ */
bool gotoCallback([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  pathfinder::AstarPlanner planner = pathfinder::AstarPlanner(safe_obstacle_distance, clearing_radius, euclidean_distance_cutoff, unknown_is_occupied, bv);
  octomap::point3d         start;
  start.x() = uav_pos.x();
  start.y() = uav_pos.y();
  start.z() = uav_pos.z();

  octomap::point3d goal;
  goal.x() = req.goal[0];
  goal.y() = req.goal[1];
  goal.z() = req.goal[2];

  auto path = planner.findPath(start, goal, octree_);

  for (auto& p : path) {
    bv.addPoint(Eigen::Vector3d(p.x(), p.y(), p.z()));
  }

  res.success = true;
  res.message = "Success";
  return true;
}
//}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "debug_planner_executable");
  ros::NodeHandle      nh_ = ros::NodeHandle("~");
  mrs_lib::ParamLoader param_loader(nh_, "debug_planner_executable");

  param_loader.loadParam("euclidean_distance_cutoff", euclidean_distance_cutoff);
  param_loader.loadParam("clearing_radius", clearing_radius);
  param_loader.loadParam("safe_obstacle_distance", safe_obstacle_distance);
  param_loader.loadParam("navigation_tolerance", navigation_tolerance);
  param_loader.loadParam("explore_unknown_threshold", explore_unknown_threshold);
  param_loader.loadParam("distance_penalty", distance_penalty);
  param_loader.loadParam("greedy_penalty", greedy_penalty);
  param_loader.loadParam("unknown_is_occupied", unknown_is_occupied);
  param_loader.loadParam("points_scale", points_scale);
  param_loader.loadParam("lines_scale", lines_scale);

  /* map_timer = nh_.createTimer(ros::Duration(2), &timerCallback); */

  odom_subscriber    = nh_.subscribe("odom_in", 10, &odomCallback, ros::TransportHints().tcpNoDelay());
  octomap_subscriber = nh_.subscribe("octomap_in", 1, &octomapCallback, ros::TransportHints().tcpNoDelay());

  // BatchVisualizer setup
  bv = mrs_lib::BatchVisualizer(nh_, "visualize", "uav1/gps_origin");
  bv.setPointsScale(points_scale);
  bv.setLinesScale(lines_scale);


  goto_server = nh_.advertiseService("goto_in", &gotoCallback);


  ROS_INFO("[%s]: Initialized!", ros::this_node::getName().c_str());
  ros::spin();

  return 0;
}
