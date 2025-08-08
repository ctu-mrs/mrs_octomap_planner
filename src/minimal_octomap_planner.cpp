/* includes //{ */

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_modules_msgs/OctomapPlannerDiagnostics.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/ValidateReferenceArray.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_octomap_tools/octomap_methods.h>
#include <mrs_subt_planning_lib/astar_planner.h>
#include <nodelet/nodelet.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <astar_planner.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>

//}

namespace mrs_octomap_planner
{

using OcTree_t          = octomap::OcTree;
using OcTreeSharedPtr_t = std::shared_ptr<octomap::OcTree>;
using OctomapConstPtr_t = octomap_msgs::Octomap::ConstPtr;

class MinimalOcotomapPlanner : public nodelet::Nodelet
{

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool        is_initialized_ = false;
  std::string _uav_name_;

  mrs_octomap_planner::AstarPlanner planner                  = mrs_octomap_planner::AstarPlanner(safe_obstacle_distance,
                                                                                _euclidean_distance_cutoff_,
                                                                                _distance_transform_distance_,
                                                                                planning_tree_resolution_,
                                                                                _distance_penalty_,
                                                                                _greedy_penalty_,
                                                                                _timeout_threshold_,
                                                                                _max_waypoint_distance_,
                                                                                _min_altitude_,
                                                                                max_altitude,
                                                                                _unknown_is_occupied_,
                                                                                bv_planner_);
  double                            _safe_obstacle_distance_ = 0.0;
  double                            _distance_transform_distance_ = 0.0;
  double                            _planning_tree_resolution_    = 0.0;
  double                            _distance_penalty_            = 0.0;
  double                            _greedy_penalty_              = 0.0;
  double                            _timeout_threshold_           = 0.0;
  double                            _time_for_trajectory_generator_;
  double                            _max_waypoint_distance_ = 0.0;
  double                            _min_altitude_;
  double                            _rate_main_timer_;
  double                            _rate_diagnostics_timer_;
  double                            _rate_future_check_timer_;
  double                            _replan_after_;
  double                            _min_path_length_;
  double                            _min_path_heading_change_;
  bool                              _unknown_is_occupied_;
  bool                              _use_subt_planner_;
  bool                              _subt_make_path_straight_;
  bool                              _subt_apply_postprocessing_;
  bool                              _subt_apply_pruning_;
  bool                              _subt_debug_info_;
  double                            _subt_clearing_dist_;
  double                            _subt_pruning_dist_;
  double                            _subt_bbx_horizontal_;
  double                            _subt_bbx_vertical_;
  double                            _subt_processing_safe_dist_;
  double                            _subt_admissibility_;
  int                               _subt_processing_max_iterations_;
  bool                              _subt_processing_horizontal_neighbors_only_;
  double                            _subt_processing_z_diff_tolerance_;
  bool                              _subt_processing_fix_goal_point_;
  double                            _subt_processing_path_length_;
  double                            _subt_processing_timeout_;
  int                               _subt_shortening_window_size_;
  int                               _subt_shortening_distance_;
  bool                              _subt_remove_obsolete_points_;
  double                            _subt_obsolete_points_tolerance_;
  double                            _distance_transform_distance_;
  double                            _trajectory_generation_input_length_;
  bool                              _trajectory_generation_relax_heading_;
  bool                              _trajectory_generation_use_heading_;
  int                               _collision_check_point_count_;
  int                               _min_allowed_trajectory_points_after_crop_;
  bool                              _scope_timer_enabled_;
  double                            _scope_timer_duration_;
  double                            _goal_reached_dist_;
  int                               _max_attempts_to_replan_;
  double _min_dist_to_goal_improvement_;  // minimum improvement of distance of last waypoint to goal in two consecutive
                                          // replanning steps
  double _max_goal_dist_to_disable_replanning_;  // maximum distance for which the replanning can be disabled when there
                                                 // is a risk of oscillations
  bool _use_user_heading_for_final_point_;

  double     _max_altitude_;
  std::mutex mutex_max_altitude_;

  double     _safe_obstacle_distance_;
  double     _safe_obstacle_distance_min_;
  double     _safe_obstacle_distance_max_;
  std::mutex mutex_safety_distance_;

  bool   _turn_in_flight_direction_;
  double _heading_offset_;
  double _max_segment_length_for_heading_sampling_;  // TODO: fix variable name

  double planning_tree_resolution_;

  std::string octree_frame_;

  std::shared_ptr<OcTree_t> octree_;
  std::mutex                mutex_octree_;

  ros::Time  planner_time_flag_;
  std::mutex mutex_planner_time_flag_;
  bool       _restart_planner_on_deadlock_;
  double     planner_deadlock_timeout_;

  // virtual obstacles params
  std::mutex                     _mutex_virtual_obstacles;
  std::vector<VirtualObstacle_t> _virtual_obstacles;
  void                           addVirtualObstaclesToOctree(const std::shared_ptr<OcTree_t> octree);

  // visualizer params
  double _points_scale_;
  double _lines_scale_;

  // initial condition
  octomap::point3d  initial_pos_;
  double            initial_heading_;
  std::mutex        mutex_initial_condition_;
  std::atomic<bool> got_initial_pos_;

  std::shared_ptr<mrs_lib::BatchVisualizer> bv_input_;
  std::mutex                                mutex_bv_input_;

  std::shared_ptr<mrs_lib::BatchVisualizer> bv_planner_;
  bool                                      bv_planner_frame_set_ = false;

  mrs_lib::BatchVisualizer bv_processed_;
  std::mutex               mutex_bv_processed_;

  // subscribers
  mrs_lib::SubscribeHandler<octomap_msgs::Octomap>         sh_octomap_;
  mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints> sh_constraints_;

  // subscriber callbacks
  void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);

  // service servers
  ros::ServiceServer service_server_get_plan_

      // service server callbacks
      bool
      callbackGetPath([[maybe_unused]] mrs_msgs::Vec4::Request& req,
                      mrs_msgs::Vec4::Response&                 res);

  // timers
  ros::Timer timer_main_;
  void       timerMain([[maybe_unused]] const ros::TimerEvent& evt);

  // diagnostics
  mrs_modules_msgs::OctomapPlannerDiagnostics diagnostics_;
  std::mutex                                  mutex_diagnostics_;

  // timeouts
  void timeoutOctomap(const std::string& topic,
                      const ros::Time&   last_msg);

  // transformer
  std::unique_ptr<mrs_lib::Transformer> transformer_;

  std::string current_control_frame_;
  std::mutex  mutex_current_control_frame_;

  // planning
  std::atomic<int> replanning_counter_ = 0;
  ros::Time        time_last_plan_;
  int              path_id_                         = 0;
  bool             new_user_goal_received_          = false;
  bool             first_planning_for_current_goal_ = false;
  bool             detected_collision_              = false;

  ros::Time replanning_start_timepoint_;
  ros::Time replanning_end_timepoint_;

  bool copyLocalMap(std::shared_ptr<OcTree_t>& from,
                    std::shared_ptr<OcTree_t>& to);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree,
                                 const octomap::OcTreeKey&  key,
                                 unsigned int               target_depth);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree,
                                       octomap::OcTreeNode*       node,
                                       const octomap::OcTreeKey&  key,
                                       unsigned int               depth,
                                       unsigned int               max_depth);
};


void MinimalOcotomapPlanner::onInit()
{

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  planner_time_flag_ = ros::Time(0);

  ROS_INFO("[MrsMinimalOctomapPlanner]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "MrsMinimalOctomapPlanner");

  double _planner_deadlock_timeout_factor;

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("main_timer/rate", _rate_main_timer_);
  param_loader.loadParam("diagnostics_timer/rate", _rate_diagnostics_timer_);
  param_loader.loadParam("future_check_timer/rate", _rate_future_check_timer_);

  param_loader.loadParam("euclidean_distance_cutoff", _euclidean_distance_cutoff_);
  param_loader.loadParam("safe_obstacle_distance/default", _safe_obstacle_distance_);
  param_loader.loadParam("safe_obstacle_distance/min", _safe_obstacle_distance_min_);
  param_loader.loadParam("safe_obstacle_distance/max", _safe_obstacle_distance_max_);
  param_loader.loadParam("distance_penalty", _distance_penalty_);
  param_loader.loadParam("greedy_penalty", _greedy_penalty_);
  param_loader.loadParam("planning_tree/resolution", planning_tree_resolution_);
  param_loader.loadParam("unknown_is_occupied", _unknown_is_occupied_);
  param_loader.loadParam("distance_transform/submap_distance", _distance_transform_distance_);
  param_loader.loadParam("points_scale", _points_scale_);
  param_loader.loadParam("lines_scale", _lines_scale_);
  param_loader.loadParam("max_waypoint_distance", _max_waypoint_distance_);
  param_loader.loadParam("min_altitude", _min_altitude_);
  param_loader.loadParam("max_altitude", _max_altitude_);
  param_loader.loadParam("timeout_threshold", _timeout_threshold_);
  param_loader.loadParam("time_for_trajectory_generator", _time_for_trajectory_generator_);
  param_loader.loadParam("replan_after", _replan_after_);
  param_loader.loadParam("min_path_length", _min_path_length_);
  param_loader.loadParam("min_path_heading_change", _min_path_heading_change_);
  param_loader.loadParam("goal_reached_dist", _goal_reached_dist_);
  param_loader.loadParam("max_attempts_to_replan", _max_attempts_to_replan_);
  param_loader.loadParam("min_dist_to_goal_improvement", _min_dist_to_goal_improvement_);
  param_loader.loadParam("max_goal_dist_to_disable_replanning", _max_goal_dist_to_disable_replanning_);
  param_loader.loadParam("use_user_heading_for_final_point", _use_user_heading_for_final_point_);
  param_loader.loadParam("trajectory_generator/input_trajectory_length", _trajectory_generation_input_length_);
  param_loader.loadParam("trajectory_generator/use_heading", _trajectory_generation_use_heading_);
  param_loader.loadParam("trajectory_generator/relax_heading", _trajectory_generation_relax_heading_);
  param_loader.loadParam("trajectory_generator/turn_in_flight_direction", _turn_in_flight_direction_);
  param_loader.loadParam("trajectory_generator/heading_offset", _heading_offset_);
  param_loader.loadParam("trajectory_generator/max_segment_length_heading", _max_segment_length_for_heading_sampling_);
  param_loader.loadParam("subt_planner/use", _use_subt_planner_);
  param_loader.loadParam("subt_planner/make_path_straight", _subt_make_path_straight_);
  param_loader.loadParam("subt_planner/apply_postprocessing", _subt_apply_postprocessing_);
  param_loader.loadParam("subt_planner/apply_pruning", _subt_apply_pruning_);
  param_loader.loadParam("subt_planner/debug_info", _subt_debug_info_);
  param_loader.loadParam("subt_planner/clearing_dist", _subt_clearing_dist_);
  param_loader.loadParam("subt_planner/pruning_dist", _subt_pruning_dist_);
  param_loader.loadParam("subt_planner/admissibility", _subt_admissibility_);
  param_loader.loadParam("subt_planner/planning_tree/bounding_box/horizontal", _subt_bbx_horizontal_);
  param_loader.loadParam("subt_planner/planning_tree/bounding_box/vertical", _subt_bbx_vertical_);
  param_loader.loadParam("subt_planner/postprocessing/safe_dist", _subt_processing_safe_dist_);
  param_loader.loadParam("subt_planner/postprocessing/max_iteration", _subt_processing_max_iterations_);
  param_loader.loadParam("subt_planner/postprocessing/horizontal_neighbors_only",
                         _subt_processing_horizontal_neighbors_only_);
  param_loader.loadParam("subt_planner/postprocessing/z_diff_tolerance", _subt_processing_z_diff_tolerance_);
  param_loader.loadParam("subt_planner/postprocessing/fix_goal_point", _subt_processing_fix_goal_point_);
  param_loader.loadParam("subt_planner/postprocessing/path_length", _subt_processing_path_length_);
  param_loader.loadParam("subt_planner/postprocessing/timeout", _subt_processing_timeout_);
  param_loader.loadParam("subt_planner/shortening/window_size", _subt_shortening_window_size_);
  param_loader.loadParam("subt_planner/shortening/distance", _subt_shortening_distance_);
  param_loader.loadParam("subt_planner/remove_obsolete_points", _subt_remove_obsolete_points_);
  param_loader.loadParam("subt_planner/obsolete_points_tolerance", _subt_obsolete_points_tolerance_);
  param_loader.loadParam("collision_check_point_count", _collision_check_point_count_);
  param_loader.loadParam("min_allowed_trajectory_points_after_crop", _min_allowed_trajectory_points_after_crop_);
  param_loader.loadParam("scope_timer/enable", _scope_timer_enabled_);
  param_loader.loadParam("scope_timer/duration", _scope_timer_duration_);
  param_loader.loadParam("restart_planner_on_deadlock", _restart_planner_on_deadlock_);
  param_loader.loadParam("planner_deadlock_timeout_factor", _planner_deadlock_timeout_factor);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MrsMinimalOctomapPlanner]: could not load all parameters");
    ros::shutdown();
  }

  if (_goal_reached_dist_ < 2 * planning_tree_resolution_) {
    ROS_WARN(
        "[MinimalOcotomapPlanner]: Cannot set %.2f as goal reached dist for planning tree resolution %.2f. Setting "
        "goal reached distance to %.2f to prevent deadlocks.",
        _goal_reached_dist_,
        planning_tree_resolution_,
        2 * planning_tree_resolution_);
    _goal_reached_dist_ = 2 * planning_tree_resolution_;
  }

  if (_euclidean_distance_cutoff_ <= _safe_obstacle_distance_) {
    ROS_WARN(
        "[MinimalOcotomapPlanner]: Setting Euclidean distance cutoff to %.2f for safety distance %.2f leads to "
        "unfeasible planning. Setting Euclidean distance cutoff to %.2f "
        "to prevent UAV being stuck.",
        _euclidean_distance_cutoff_,
        _safe_obstacle_distance_,
        _safe_obstacle_distance_ + 0.01);
    _euclidean_distance_cutoff_ = _safe_obstacle_distance_ + 0.01;
  }


  // set planner deadlock timeout
  if (_restart_planner_on_deadlock_) {

    if (_planner_deadlock_timeout_factor < 3.0) {
      ROS_WARN(
          "[MrsMinimalOctomapPlanner]: Timeout factor for planner deadlock detection was set too low (< 3.0). Setting "
          "factor to 3.0 to prevent premature killing of "
          "the planner.");
      _planner_deadlock_timeout_factor = 3.0;
    }

    planner_deadlock_timeout_ = _planner_deadlock_timeout_factor * _timeout_threshold_;
    ROS_INFO("[MrsMinimalOctomapPlanner]: Planner deadlock timeout set to %.2f s.", planner_deadlock_timeout_);
  }

  octree_ = nullptr;

  // | ---------------------- state machine --------------------- |

  state_ = STATE_IDLE;

  // | ----------------------- publishers ----------------------- |

  pub_diagnostics_       = nh_.advertise<mrs_modules_msgs::OctomapPlannerDiagnostics>("diagnostics_out", 1);
  pub_virtual_obstacles_ = nh_.advertise<visualization_msgs::MarkerArray>("virtual_obstacles_out", 1);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Pathfinder";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts,
                                                                 "octomap_in",
                                                                 ros::Duration(5.0),
                                                                 &MinimalOcotomapPlanner::timeoutOctomap,
                                                                 this,
                                                                 &MinimalOcotomapPlanner::callbackOctomap,
                                                                 this);
  // | --------------------- service clients -------------------- |

  sc_get_trajectory_ = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
  sc_trajectory_reference_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
  sc_hover_ = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "hover_out");

  // | ----------------------- transformer ---------------------- |

  transformer_ = std::make_unique<mrs_lib::Transformer>("Pathfinder");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | -------------------- batch visualiuzer ------------------- |

  bv_input_ = mrs_lib::BatchVisualizer(nh_, "visualize_input", "");
  bv_input_.setPointsScale(_points_scale_);
  bv_input_.setLinesScale(_lines_scale_);

  bv_planner_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_planner", "");
  bv_planner_->setPointsScale(_points_scale_);
  bv_planner_->setLinesScale(_lines_scale_);

  bv_processed_ = mrs_lib::BatchVisualizer(nh_, "visualize_processed", "");
  bv_processed_.setPointsScale(_points_scale_);
  bv_processed_.setLinesScale(_lines_scale_);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_rate_main_timer_), &MinimalOcotomapPlanner::timerMain, this);
  timer_future_check_ =
      nh_.createTimer(ros::Rate(_rate_future_check_timer_), &MinimalOcotomapPlanner::timerFutureCheck, this);
  timer_diagnostics_ =
      nh_.createTimer(ros::Rate(_rate_diagnostics_timer_), &MinimalOcotomapPlanner::timerDiagnostics, this);
  timer_publish_virtual_obstacles_ =
      nh_.createTimer(ros::Rate(1.0 / 2.0), &MinimalOcotomapPlanner::timerPublishVirtualObstacles, this);

  // | --------------------- service servers -------------------- |

  service_server_goto_      = nh_.advertiseService("goto_in", &MinimalOcotomapPlanner::callbackGoto, this);
  service_server_stop_      = nh_.advertiseService("stop_in", &MinimalOcotomapPlanner::callbackStop, this);
  service_server_reference_ = nh_.advertiseService("reference_in", &MinimalOcotomapPlanner::callbackReference, this);
  service_server_set_planner_ =
      nh_.advertiseService("planner_type_in", &MinimalOcotomapPlanner::callbackSetPlanner, this);
  service_server_set_safety_distance_ =
      nh_.advertiseService("set_safety_distance_in", &MinimalOcotomapPlanner::callbackSetSafetyDistance, this);
  service_server_set_max_altitude_ =
      nh_.advertiseService("set_max_altitude_in", &MinimalOcotomapPlanner::callbackSetMaxAltitude, this);
  service_server_add_virtual_obstacle_ =
      nh_.advertiseService("add_virtual_obstacle_in", &MinimalOcotomapPlanner::callbackAddVirtualObstacle, this);

  // | --------------------- finish the init -------------------- |


  is_initialized_ = true;

  ROS_INFO("[MrsMinimalOctomapPlanner]: initialized");
}


void MinimalOcotomapPlanner::callbackOctomap(OctomapConstPtr_t msg)
{

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsMinimalOctomapPlanner]: getting octomap");

  std::optional<OcTreeSharedPtr_t> octree_local = msgToMap(msg);

  if (!octree_local) {
    ROS_WARN_THROTTLE(1.0, "[MrsMinimalOctomapPlanner]: received map is empty!");
    return;
  }

  {
    std::scoped_lock lock(mutex_octree_);
    octree_       = octree_local.value();
    octree_frame_ = msg->header.frame_id;
  }

  bv_planner_->setParentFrame(msg->header.frame_id);
}

std::optional<OcTreeSharedPtr_t> OctomapPlanner::msgToMap(OctomapConstPtr_t octomap)
{

  octomap::AbstractOcTree* abstract_tree;

  if (octomap->binary) {
    abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);
  }
  else {
    abstract_tree = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!abstract_tree) {

    ROS_WARN("[MrsMinimalOctomapPlanner]: Octomap message is empty! can not convert to OcTree");
    return {};
  }
  else {

    OcTreePtr_t octree_out = OcTreePtr_t(dynamic_cast<OcTree_t*>(abstract_tree));
    return { octree_out };
  }
}

void MinimalOcotomapPlanner::timeoutOctomap(const std::string& topic,
                                            const ros::Time&   last_msg)
{

  if (!is_initialized_) {
    return;
  }

  if (!sh_octomap_.hasMsg()) {
    return;
  }

  ROS_WARN_THROTTLE(1.0, "[MrsMinimalOctomapPlanner]: octomap timeout!");
}

bool MinimalOcotomapPlanner::callbackGetPath(mrs_octomap_planner::Path::Request&  req,
                                             mrs_octomap_planner::Path::Response& res)
{
  if (!is_initialized_) {
    return false;
  }

  const bool got_octomap = sh_octomap_.hasMsg() && (ros::Time::now() - sh_octomap_.lastMsgTime()).toSec() < 2.0;

  if (!got_octomap) {
    ROS_INFO_THROTTLE(1.0,
                      "[MrsMinimalOctomapPlanner]: waiting for data: octomap = %s",
                      got_octomap ? "TRUE" : "FALSE");
    return false;
  }

  mrs_octomap_planner::AstarPlanner planner = mrs_octomap_planner::AstarPlanner(safe_obstacle_distance,
                                                                                _euclidean_distance_cutoff_,
                                                                                _distance_transform_distance_,
                                                                                planning_tree_resolution_,
                                                                                _distance_penalty_,
                                                                                _greedy_penalty_,
                                                                                _timeout_threshold_,
                                                                                _max_waypoint_distance_,
                                                                                _min_altitude_,
                                                                                max_altitude,
                                                                                _unknown_is_occupied_,
                                                                                bv_planner_);

  octomap::point3d plan_from, plan_to;
  plan_from.x() = req.start[0];
  plan_from.y() = req.start[1];
  plan_from.z() = req.start[2];

  plan_to.x() = req.end[0];
  plan_to.y() = req.end[1];
  plan_to.z() = req.end[2];

  OcTreeSharedPtr_t octree = mrs_lib::get_mutexed(mutex_octree_, octree_);

  auto path = planner.findPath(plan_from, plan_to, octree, _time_for_planning_);

  // path is complete
  if (path.second) {
    path.first.push_back(plan_to);
    ROS_INFO("[MrsMinimalOctomapPlanner]: Found complete path of length = %lu", path.first.size());
  }
  else {
    if (path.first.size() < 2) {
      ROS_WARN("[MrsMinimalOctomapPlanner]: No path found");
      res.success = false;
      res.message = "No path found";
      res.path    = std::vector<octomap::point3d>();
      return true;
    }
    ROS_INFO("[MrsMinimalOctomapPlanner]: Incomplete path found");

    double front_x = path.first.front().x();
    double front_y = path.first.front().y();
    double front_z = path.first.front().z();

    double back_x = path.first.back().x();
    double back_y = path.first.back().y();
    double back_z = path.first.back().z();

    double dist_path_start_to_end =
        sqrt(pow(front_x - back_x, 2) + pow(front_y - back_y, 2) + pow(front_z - back_z, 2));

    if (dist_path_start_to_end < _min_path_length_) {
      ROS_WARN("[MrsMinimalOctomapPlanner]: Path too short, length: %.3f", dist_path_start_to_end);
      res.success = false;
      res.message = "Path too short, length: " << dist_path_start_to_end;
      res.path    = std::vector<octomap::point3d>();
      return true;
    }
  }
  res.success = true;
  res.message = "Found complete path of length = " << path.first.size();
  res.path    = path.first;
  return true;
}
}

}  // namespace mrs_octomap_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_planner::MinimalOcotomapPlanner,
                       nodelet::Nodelet)
