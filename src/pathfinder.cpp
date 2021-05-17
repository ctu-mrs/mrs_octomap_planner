// some ros includes
#include "ros/duration.h"
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
#include <mrs_msgs/Path.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <pathfinder/astar_planner.hpp>

enum state_t
{
  IDLE = 0,
  PLANNING,
  MOVING
};

std::string state_str[] = {"IDLE", "PLANNING", "MOVING"};

// params
double euclidean_distance_cutoff;
double safe_obstacle_distance;
double distance_penalty;
double greedy_penalty;
double planning_tree_resolution;
double timeout_threshold;
double max_waypoint_distance;
double endpoint_tolerance;
bool   unknown_is_occupied;

std::shared_ptr<octomap::OcTree> octree_;
std::string                      map_frame;

// visualizer params
double points_scale;
double lines_scale;

// uav odometry
octomap::point3d   uav_pos;  // [m]
Eigen::Quaterniond uav_ori;
double             uav_heading;

mrs_lib::BatchVisualizer command_bv;
mrs_lib::BatchVisualizer planner_bv;

// subscribers
ros::Subscriber    odom_subscriber;
ros::Subscriber    octomap_subscriber;
ros::Subscriber    trajectory_subscriber;
ros::ServiceServer goto_server;

// publishers
ros::Publisher path_publisher;

// timers
double     timer_rate = 20.0;  // [Hz]
ros::Timer action_timer;

// planning
int                           replanning_without_motion = 0;
bool                          initialized               = false;
std::atomic<state_t>          state                     = IDLE;
octomap::point3d              current_goal;
octomap::point3d              final_goal;
octomap::point3d              plan_from;
std::vector<octomap::point3d> waypoint_buffer;
bool                          got_trajectory_generator_output = false;
ros::Time                     replanning_timepoint;


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
  uav_heading = ac.getHeading();
}
//}

/* octomapCallback //{ */
void octomapCallback(const octomap_msgs::OctomapPtr octomap_in) {
  ROS_INFO_ONCE("[%s]: Getting octomap", ros::this_node::getName().c_str());
  map_frame     = octomap_in->header.frame_id;
  auto tree_ptr = octomap_msgs::fullMsgToMap(*octomap_in);
  if (!tree_ptr) {
    ROS_WARN("[%s]: Octomap message is empty!", ros::this_node::getName().c_str());
  } else {
    octree_ = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));
  }
}
//}

/* trajectoryCallback //{ */
void trajectoryCallback(const mrs_msgs::TrajectoryReference& trajectory) {
  got_trajectory_generator_output = true;
  plan_from.x()                   = trajectory.points.back().position.x;
  plan_from.y()                   = trajectory.points.back().position.y;
  plan_from.z()                   = trajectory.points.back().position.z;
  replanning_timepoint            = ros::Time::now() + ros::Duration(trajectory.dt * trajectory.points.size()) - ros::Duration(timeout_threshold);
  ROS_INFO("[%s]: Trajectory processed. Replanning in %.2f seconds", ros::this_node::getName().c_str(),
           replanning_timepoint.toSec() - ros::Time::now().toSec());
}
//}

/* gotoCallback //{ */
bool gotoCallback([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  command_bv.clearBuffers();

  final_goal.x() = req.goal[0];
  final_goal.y() = req.goal[1];
  final_goal.z() = req.goal[2];

  plan_from = uav_pos;

  state = PLANNING;
  command_bv.addPoint(Eigen::Vector3d(final_goal.x(), final_goal.y(), final_goal.z()));
  command_bv.publish();

  res.success = true;
  res.message = "Goal set";
  return true;
}
//}

/* actionRoutine //{ */
void actionRoutine([[maybe_unused]] const ros::TimerEvent& evt) {
  if (!initialized) {
    return;
  }

  switch (state) {

    case IDLE: {
      break;
    }

    case PLANNING: {
      got_trajectory_generator_output = false;
      planner_bv.clearBuffers();

      pathfinder::AstarPlanner planner = pathfinder::AstarPlanner(safe_obstacle_distance, euclidean_distance_cutoff, planning_tree_resolution, distance_penalty,
                                                                  greedy_penalty, timeout_threshold, max_waypoint_distance, unknown_is_occupied, planner_bv);

      waypoint_buffer = planner.findPath(plan_from, final_goal, octree_);
      if (waypoint_buffer.size() < 2) {
        ROS_ERROR("[%s]: Path not found", ros::this_node::getName().c_str());
        replanning_without_motion++;
        if (replanning_without_motion > 10) {
          ROS_ERROR("[%s]: Planning failed, UAV is stuck!", ros::this_node::getName().c_str());
          state = IDLE;
          break;
        }
        state = PLANNING;
        break;
      }
      current_goal = waypoint_buffer.back();

      mrs_msgs::Path path_msg;
      path_msg.header.frame_id = map_frame;
      path_msg.header.stamp    = ros::Time::now();
      path_msg.fly_now         = true;

      for (auto& w : waypoint_buffer) {
        mrs_msgs::Reference ref;
        ref.position.x = w.x();
        ref.position.y = w.y();
        ref.position.z = w.z();
        ref.heading    = 0;
        path_msg.points.push_back(ref);
      }
      /* planner_bv.addPoint(Eigen::Vector3d(current_goal.x(), current_goal.y(), current_goal.z())); */
      /* planner_bv.addTrajectory(path_msg); */
      /* planner_bv.publish(); */

      path_publisher.publish(path_msg);
      state = MOVING;
      break;
    }

    case MOVING: {
      replanning_without_motion = 0;
      if ((uav_pos - final_goal).norm() < endpoint_tolerance) {
        ROS_INFO("[%s]: Final goal reached!", ros::this_node::getName().c_str());
        state = IDLE;
      }
      if (got_trajectory_generator_output && replanning_timepoint.toSec() - ros::Time::now().toSec() < 0) {
        state = PLANNING;
      }
      break;
    }
  }
}
//}

int main(int argc, char** argv) {

  // initialize node and create no handle
  ros::init(argc, argv, "debug_planner_executable");
  ros::NodeHandle      nh_ = ros::NodeHandle("~");
  mrs_lib::ParamLoader param_loader(nh_, "debug_planner_executable");

  param_loader.loadParam("euclidean_distance_cutoff", euclidean_distance_cutoff);
  param_loader.loadParam("safe_obstacle_distance", safe_obstacle_distance);
  param_loader.loadParam("distance_penalty", distance_penalty);
  param_loader.loadParam("greedy_penalty", greedy_penalty);
  param_loader.loadParam("planning_tree_resolution", planning_tree_resolution);
  param_loader.loadParam("unknown_is_occupied", unknown_is_occupied);
  param_loader.loadParam("points_scale", points_scale);
  param_loader.loadParam("endpoint_tolerance", endpoint_tolerance);
  param_loader.loadParam("lines_scale", lines_scale);
  param_loader.loadParam("max_waypoint_distance", max_waypoint_distance);
  param_loader.loadParam("timeout_threshold", timeout_threshold);

  action_timer = nh_.createTimer(ros::Duration(1.0 / timer_rate), &actionRoutine);

  odom_subscriber       = nh_.subscribe("odom_in", 10, &odomCallback, ros::TransportHints().tcpNoDelay());
  octomap_subscriber    = nh_.subscribe("octomap_in", 1, &octomapCallback, ros::TransportHints().tcpNoDelay());
  trajectory_subscriber = nh_.subscribe("trajectory_in", 1, &trajectoryCallback, ros::TransportHints().tcpNoDelay());
  path_publisher        = nh_.advertise<mrs_msgs::Path>("path_out", 1);

  // BatchVisualizer setup
  command_bv = mrs_lib::BatchVisualizer(nh_, "visualize_command", "uav1/gps_origin");
  command_bv.setPointsScale(points_scale);
  command_bv.setLinesScale(lines_scale);

  planner_bv = mrs_lib::BatchVisualizer(nh_, "visualize_planner", "uav1/gps_origin");
  planner_bv.setPointsScale(points_scale);
  planner_bv.setLinesScale(lines_scale);

  goto_server = nh_.advertiseService("goto_in", &gotoCallback);


  ROS_INFO("[%s]: Initialized!", ros::this_node::getName().c_str());
  initialized = true;
  ros::spin();

  return 0;
}
