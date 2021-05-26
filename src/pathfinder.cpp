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
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>

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
double time_for_trajectory_generator;
double max_waypoint_distance;
double endpoint_tolerance;
double min_altitude;
double replan_after;
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

mrs_lib::BatchVisualizer input_bv;
mrs_lib::BatchVisualizer planner_bv;
mrs_lib::BatchVisualizer processed_bv;

// subscribers
ros::Subscriber    odom_subscriber;
ros::Subscriber    octomap_subscriber;
ros::Subscriber    mpc_prediction_subscriber;
ros::ServiceServer goto_server;

// service client
ros::ServiceClient trajectory_generation_client;
ros::ServiceClient trajectory_reference_client;

// timers
double     timer_rate = 20.0;  // [Hz]
ros::Timer action_timer;

// planning
int                  replanning_without_motion = 0;
bool                 initialized               = false;
std::atomic<state_t> state                     = IDLE;
octomap::point3d     user_defined_goal;
octomap::point3d     internal_goal;
octomap::point3d     plan_from;

bool             set_timepoints = false;
ros::Time        replanning_start_timepoint;
ros::Time        replanning_end_timepoint;
octomap::point3d replanning_point;

/* setReplanningPoint //{ */
void setReplanningPoint(mrs_msgs::TrajectoryReference traj) {
  replanning_point.x() = traj.points.back().position.x;
  replanning_point.y() = traj.points.back().position.y;
  replanning_point.z() = traj.points.back().position.z;
  /* double dist          = 0.0; */
  /* for (size_t i = 1; i < traj.points.size(); i++) { */
  /*   Eigen::Vector3d p0(traj.points[i - 1].position.x, traj.points[i - 1].position.y, traj.points[i - 1].position.z); */
  /*   Eigen::Vector3d p1(traj.points[i].position.x, traj.points[i].position.y, traj.points[i].position.z); */
  /*   dist += (p0 - p1).norm(); */
  /*   if (dist >= replan_after) { */
  /*     break; */
  /*   } */
  /*   replanning_point.x() = traj.points[i].position.x; */
  /*   replanning_point.y() = traj.points[i].position.y; */
  /*   replanning_point.z() = traj.points[i].position.z; */
  /* } */
  mrs_lib::geometry::Cuboid c(Eigen::Vector3d(replanning_point.x(), replanning_point.y(), replanning_point.z()), Eigen::Vector3d(0.4, 0.4, 0.4),
                              Eigen::Quaterniond::Identity());
  input_bv.clearBuffers();
  input_bv.addPoint(Eigen::Vector3d(user_defined_goal.x(), user_defined_goal.y(), user_defined_goal.z()), 0, 1, 0, 1);
  input_bv.addCuboid(c, 0.5, 1.0, 1.0, 0.8, true);
  input_bv.publish();
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

/* mpcPredictionCallback //{ */
void mpcPredictionCallback(const mrs_msgs::MpcPredictionFullState& prediction) {
  ROS_INFO_ONCE("[%s]: Getting MPC Tracker output", ros::this_node::getName().c_str());
  if (set_timepoints) {

    for (size_t i = 0; i < prediction.position.size(); i++) {
      octomap::point3d p;
      p.x() = prediction.position[i].x;
      p.y() = prediction.position[i].y;
      p.z() = prediction.position[i].z;

      if ((p - replanning_point).norm() < 0.08) {
        replanning_end_timepoint   = prediction.stamps[i];
        replanning_start_timepoint = replanning_end_timepoint - ros::Duration(timeout_threshold + time_for_trajectory_generator);
        ROS_INFO("[%s]: Replanning will start in %.3f seconds!", ros::this_node::getName().c_str(), (replanning_start_timepoint - ros::Time::now()).toSec());
        set_timepoints = false;
        break;
      }
    }
  }
}
//}

/* gotoCallback //{ */
bool gotoCallback([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  input_bv.clearBuffers();

  user_defined_goal.x() = req.goal[0];
  user_defined_goal.y() = req.goal[1];
  user_defined_goal.z() = req.goal[2];

  plan_from = uav_pos;

  state = PLANNING;
  input_bv.addPoint(Eigen::Vector3d(user_defined_goal.x(), user_defined_goal.y(), user_defined_goal.z()));
  input_bv.publish();

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
      pathfinder::AstarPlanner planner =
          pathfinder::AstarPlanner(safe_obstacle_distance, euclidean_distance_cutoff, planning_tree_resolution, distance_penalty, greedy_penalty,
                                   timeout_threshold, max_waypoint_distance, min_altitude, unknown_is_occupied, planner_bv);

      auto waypoints = planner.findPath(plan_from, user_defined_goal, octree_);
      if (waypoints.size() < 2) {
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

      mrs_msgs::GetPathSrv path_srv;
      path_srv.request.path.header.frame_id = map_frame;
      path_srv.request.path.header.stamp    = ros::Time::now();
      path_srv.request.path.fly_now         = false;

      for (auto& w : waypoints) {
        mrs_msgs::Reference ref;
        ref.position.x = w.x();
        ref.position.y = w.y();
        ref.position.z = w.z();
        ref.heading    = 0;
        path_srv.request.path.points.push_back(ref);
      }

      ROS_INFO("[%s]: Calling trajectory generation", ros::this_node::getName().c_str());

      trajectory_generation_client.call(path_srv);

      ROS_INFO("[%s]: Generator response: %s", ros::this_node::getName().c_str(), path_srv.response.message.c_str());

      if (!path_srv.response.success) {
        break;
      }

      processed_bv.clearBuffers();
      for (auto& p : path_srv.response.trajectory.points) {
        auto v = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        processed_bv.addPoint(v, 0, 1, 0, 1);
      }
      processed_bv.publish();

      auto trajectory = path_srv.response.trajectory;
      ROS_INFO("[%s]: Setting replanning point", ros::this_node::getName().c_str());
      setReplanningPoint(trajectory);
      set_timepoints = true;

      ROS_INFO("[%s]: Publishing reference", ros::this_node::getName().c_str());

      mrs_msgs::TrajectoryReferenceSrv reference_out;
      reference_out.request.trajectory         = path_srv.response.trajectory;
      reference_out.request.trajectory.fly_now = true;
      trajectory_reference_client.call(reference_out);


      ROS_INFO("[%s]: Control manager response: %s", ros::this_node::getName().c_str(), reference_out.response.message.c_str());

      if (!reference_out.response.success) {
        break;
      }

      state = MOVING;
      break;
    }

    case MOVING: {
      replanning_without_motion = 0;
      if ((uav_pos - user_defined_goal).norm() < endpoint_tolerance) {
        ROS_INFO("[%s]: User defined goal reached!", ros::this_node::getName().c_str());
        state = IDLE;
        break;
      }

      if ((replanning_start_timepoint - ros::Time::now()).toSec() <= 0) {
        ROS_INFO("[%s]: Trigger replanning!", ros::this_node::getName().c_str());
        plan_from = replanning_point;
        state     = PLANNING;
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
  param_loader.loadParam("min_altitude", min_altitude);
  param_loader.loadParam("timeout_threshold", timeout_threshold);
  param_loader.loadParam("time_for_trajectory_generator", time_for_trajectory_generator);
  param_loader.loadParam("replan_after", replan_after);

  action_timer = nh_.createTimer(ros::Duration(1.0 / timer_rate), &actionRoutine);

  odom_subscriber           = nh_.subscribe("odom_in", 10, &odomCallback, ros::TransportHints().tcpNoDelay());
  octomap_subscriber        = nh_.subscribe("octomap_in", 1, &octomapCallback, ros::TransportHints().tcpNoDelay());
  mpc_prediction_subscriber = nh_.subscribe("mpc_prediction_in", 1, &mpcPredictionCallback, ros::TransportHints().tcpNoDelay());

  trajectory_generation_client = nh_.serviceClient<mrs_msgs::GetPathSrv>("trajectory_generation_out");
  trajectory_reference_client  = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");

  // BatchVisualizer setup
  input_bv = mrs_lib::BatchVisualizer(nh_, "visualize_input", "uav1/gps_origin");
  input_bv.setPointsScale(points_scale);
  input_bv.setLinesScale(lines_scale);

  planner_bv = mrs_lib::BatchVisualizer(nh_, "visualize_planner", "uav1/gps_origin");
  planner_bv.setPointsScale(points_scale);
  planner_bv.setLinesScale(lines_scale);

  processed_bv = mrs_lib::BatchVisualizer(nh_, "visualize_processed", "uav1/gps_origin");
  processed_bv.setPointsScale(points_scale);
  processed_bv.setLinesScale(lines_scale);

  goto_server = nh_.advertiseService("goto_in", &gotoCallback);


  ROS_INFO("[%s]: Initialized!", ros::this_node::getName().c_str());
  initialized = true;
  ros::spin();

  return 0;
}
