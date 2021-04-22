// some ros includes
#include "octomap/OcTree.h"
#include <ros/ros.h>

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
double clearing_distance;
double safe_obstacle_distance;
double navigation_tolerance;
double explore_unknown_threshold;
double min_altitude;
double distance_penalty;
double greedy_penalty;
bool   unknown_is_occupied;

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

/* gotoCallback //{ */
bool gotoCallback([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {
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
  param_loader.loadParam("clearing_distance", clearing_distance);
  param_loader.loadParam("safe_obstacle_distance", safe_obstacle_distance);
  param_loader.loadParam("navigation_tolerance", navigation_tolerance);
  param_loader.loadParam("explore_unknown_threshold", explore_unknown_threshold);
  param_loader.loadParam("min_altitude", min_altitude);
  param_loader.loadParam("distance_penalty", distance_penalty);
  param_loader.loadParam("greedy_penalty", greedy_penalty);
  param_loader.loadParam("unknown_is_occupied", unknown_is_occupied);
  param_loader.loadParam("points_scale", points_scale);
  param_loader.loadParam("lines_scale", lines_scale);

  /* map_timer = nh_.createTimer(ros::Duration(2), &timerCallback); */

  odom_subscriber = nh_.subscribe("odom_in", 10, &odomCallback, ros::TransportHints().tcpNoDelay());

  // BatchVisualizer setup
  bv = mrs_lib::BatchVisualizer(nh_, "visualize", "uav1/gps_origin");
  bv.setPointsScale(points_scale);
  bv.setLinesScale(lines_scale);


  goto_server = nh_.advertiseService("goto", &gotoCallback);

  auto             planner = AstarPlanner();
  octomap::point3d p1{0, 0, 0};
  octomap::point3d p2{1, 0, 0};
  octomap::OcTree* tree;
  planner.findPath(p1, p2, *tree);

  ROS_INFO("[%s]: Initialized!", ros::this_node::getName().c_str());
  ros::spin();

  return 0;
}
