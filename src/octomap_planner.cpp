/* includes //{ */

#include <memory>
#include <ros/init.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTree.h>
#include <mrs_octomap_tools/octomap_methods.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>

#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/Vec1.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/ValidateReferenceArray.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/String.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/TrajectoryReference.h>

#include <mrs_modules_msgs/OctomapPlannerDiagnostics.h>

#include <std_srvs/Trigger.h>

#include <astar_planner.hpp>
#include <mrs_subt_planning_lib/astar_planner.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//}

namespace mrs_octomap_planner
{

/* defines //{ */

typedef enum
{
  STATE_IDLE,
  STATE_PLANNING,
  STATE_MOVING,
} State_t;

struct VirtualObstacle_t
{
  std::string                  frame_id;
  float                        height;
  std::vector<Eigen::Vector3f> vertices;
  std::vector<Eigen::Vector3f> uvw;
  visualization_msgs::Marker   vis_marker;
};

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING"};

using OcTree_t            = octomap::OcTree;
using OcTreePtr_t         = std::shared_ptr<octomap::OcTree>;
using OcTreeMsgConstPtr_t = octomap_msgs::OctomapConstPtr;

//}

/* class OctomapPlanner //{ */

class OctomapPlanner : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool              is_initialized_ = false;
  std::atomic<bool> ready_to_plan_  = false;
  std::string       _uav_name_;

  // params
  double _euclidean_distance_cutoff_;
  double _distance_penalty_;
  double _greedy_penalty_;
  double _timeout_threshold_;
  double _time_for_trajectory_generator_;
  double _max_waypoint_distance_;
  double _min_altitude_;
  double _rate_main_timer_;
  double _rate_diagnostics_timer_;
  double _rate_future_check_timer_;
  double _replan_after_;
  double _min_path_length_;
  double _min_path_heading_change_;
  bool   _unknown_is_occupied_;
  bool   _use_subt_planner_;
  bool   _subt_make_path_straight_;
  bool   _subt_apply_postprocessing_;
  bool   _subt_apply_pruning_;
  bool   _subt_debug_info_;
  double _subt_clearing_dist_;
  double _subt_pruning_dist_;
  double _subt_bbx_horizontal_;
  double _subt_bbx_vertical_;
  double _subt_processing_safe_dist_;
  double _subt_admissibility_;
  int    _subt_processing_max_iterations_;
  bool   _subt_processing_horizontal_neighbors_only_;
  double _subt_processing_z_diff_tolerance_;
  bool   _subt_processing_fix_goal_point_;
  double _subt_processing_path_length_;
  double _subt_processing_timeout_;
  int    _subt_shortening_window_size_;
  int    _subt_shortening_distance_;
  bool   _subt_remove_obsolete_points_;
  double _subt_obsolete_points_tolerance_;
  double _distance_transform_distance_;
  double _trajectory_generation_input_length_;
  bool   _trajectory_generation_relax_heading_;
  bool   _trajectory_generation_use_heading_;
  int    _collision_check_point_count_;
  int    _min_allowed_trajectory_points_after_crop_;
  bool   _scope_timer_enabled_;
  double _scope_timer_duration_;
  double _goal_reached_dist_;
  int    _max_attempts_to_replan_;
  double _min_dist_to_goal_improvement_;         // minimum improvement of distance of last waypoint to goal in two consecutive replanning steps
  double _max_goal_dist_to_disable_replanning_;  // maximum distance for which the replanning can be disabled when there is a risk of oscillations
  bool   _use_user_heading_for_final_point_;

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

  mrs_lib::BatchVisualizer bv_input_;
  std::mutex               mutex_bv_input_;

  std::shared_ptr<mrs_lib::BatchVisualizer> bv_planner_;
  bool                                      bv_planner_frame_set_ = false;

  mrs_lib::BatchVisualizer bv_processed_;
  std::mutex               mutex_bv_processed_;

  // subscribers
  mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>            sh_tracker_cmd_;
  mrs_lib::SubscribeHandler<octomap_msgs::Octomap>               sh_octomap_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>       sh_constraints_;

  // publishers
  ros::Publisher pub_diagnostics_;
  ros::Publisher pub_virtual_obstacles_;

  // subscriber callbacks
  void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg);
  void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);

  // service servers
  ros::ServiceServer service_server_goto_;
  ros::ServiceServer service_server_stop_;
  ros::ServiceServer service_server_reference_;
  ros::ServiceServer service_server_set_planner_;
  ros::ServiceServer service_server_set_safety_distance_;
  ros::ServiceServer service_server_set_max_altitude_;
  ros::ServiceServer service_server_add_virtual_obstacle_;

  // service server callbacks
  bool callbackGoto([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackStop([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool callbackReference([[maybe_unused]] mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);
  bool callbackSetPlanner([[maybe_unused]] mrs_msgs::String::Request& req, mrs_msgs::String::Response& res);
  bool callbackSetSafetyDistance(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackSetMaxAltitude(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res);
  bool callbackAddVirtualObstacle(mrs_msgs::ValidateReferenceArray::Request& req, mrs_msgs::ValidateReferenceArray::Response& res);

  // service clients
  mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>             sc_get_trajectory_;
  mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>                sc_hover_;

  // timers
  ros::Timer timer_main_;
  void       timerMain([[maybe_unused]] const ros::TimerEvent& evt);

  ros::Timer timer_diagnostics_;
  void       timerDiagnostics([[maybe_unused]] const ros::TimerEvent& evt);

  ros::Timer timer_future_check_;
  void       timerFutureCheck([[maybe_unused]] const ros::TimerEvent& evt);

  ros::Timer timer_publish_virtual_obstacles_;
  void       timerPublishVirtualObstacles([[maybe_unused]] const ros::TimerEvent& evt);

  // diagnostics
  mrs_modules_msgs::OctomapPlannerDiagnostics diagnostics_;
  std::mutex                                  mutex_diagnostics_;

  // timeouts
  void timeoutOctomap(const std::string& topic, const ros::Time& last_msg);
  void timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg);
  void timeoutControlManagerDiag(const std::string& topic, const ros::Time& last_msg);

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

  // state machine
  std::atomic<State_t> state_;
  void                 changeState(const State_t new_state);
  std::atomic<bool>    interrupted_ = false;

  mrs_msgs::Reference user_goal_;
  std::mutex          mutex_user_goal_;

  octomap::point3d internal_goal_;

  std::atomic<bool> set_timepoints_ = false;

  ros::Time replanning_start_timepoint_;
  ros::Time replanning_end_timepoint_;

  // unused
  octomap::point3d replanning_point_;
  std::mutex       mutex_replanning_point_;

  // routines
  void setReplanningPoint(const mrs_msgs::TrajectoryReference& traj);

  std::vector<double> estimateSegmentTimes(const std::vector<Eigen::Vector4d>& vertices, const bool use_heading);

  std::optional<OcTreePtr_t> msgToMap(const octomap_msgs::OctomapConstPtr octomap);

  /**
   * @brief returns planning initial condition for a given future time based on the MPC prediction horizon
   *
   * @param time
   *
   * @return x, y, z, heading reference
   */
  std::optional<mrs_msgs::ReferenceStamped> getInitialCondition(const ros::Time time);

  bool copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth);

  void hover(void);
};

//}

/* onInit() //{ */

void OctomapPlanner::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  planner_time_flag_ = ros::Time(0);

  ROS_INFO("[MrsOctomapPlanner]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "MrsOctomapPlanner");

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
  param_loader.loadParam("subt_planner/postprocessing/horizontal_neighbors_only", _subt_processing_horizontal_neighbors_only_);
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
    ROS_ERROR("[MrsOctomapPlanner]: could not load all parameters");
    ros::shutdown();
  }

  if (_goal_reached_dist_ < 2 * planning_tree_resolution_) {
    ROS_WARN(
        "[OctomapPlanner]: Cannot set %.2f as goal reached dist for planning tree resolution %.2f. Setting goal reached distance to %.2f to prevent deadlocks.",
        _goal_reached_dist_, planning_tree_resolution_, 2 * planning_tree_resolution_);
    _goal_reached_dist_ = 2 * planning_tree_resolution_;
  }
  // set planner deadlock timeout
  if (_restart_planner_on_deadlock_) {

    if (_planner_deadlock_timeout_factor < 3.0) {
      ROS_WARN(
          "[MrsOctomapPlanner]: Timeout factor for planner deadlock detection was set too low (< 3.0). Setting factor to 3.0 to prevent premature killing of "
          "the planner.");
      _planner_deadlock_timeout_factor = 3.0;
    }

    planner_deadlock_timeout_ = _planner_deadlock_timeout_factor * _timeout_threshold_;
    ROS_INFO("[MrsOctomapPlanner]: Planner deadlock timeout set to %.2f s.", planner_deadlock_timeout_);
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

  sh_tracker_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", ros::Duration(3.0), &OctomapPlanner::timeoutTrackerCmd, this,
                                                                        &OctomapPlanner::callbackTrackerCmd, this);
  sh_octomap_     = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", ros::Duration(5.0), &OctomapPlanner::timeoutOctomap, this,
                                                                 &OctomapPlanner::callbackOctomap, this);

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", ros::Duration(3.0),
                                                                                            &OctomapPlanner::timeoutControlManagerDiag, this);

  sh_constraints_ = mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>(shopts, "constraints_in");

  // | --------------------- service clients -------------------- |

  sc_get_trajectory_       = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
  sc_trajectory_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
  sc_hover_                = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "hover_out");

  // | --------------------- service servers -------------------- |

  service_server_goto_                 = nh_.advertiseService("goto_in", &OctomapPlanner::callbackGoto, this);
  service_server_stop_                 = nh_.advertiseService("stop_in", &OctomapPlanner::callbackStop, this);
  service_server_reference_            = nh_.advertiseService("reference_in", &OctomapPlanner::callbackReference, this);
  service_server_set_planner_          = nh_.advertiseService("planner_type_in", &OctomapPlanner::callbackSetPlanner, this);
  service_server_set_safety_distance_  = nh_.advertiseService("set_safety_distance_in", &OctomapPlanner::callbackSetSafetyDistance, this);
  service_server_set_max_altitude_     = nh_.advertiseService("set_max_altitude_in", &OctomapPlanner::callbackSetMaxAltitude, this);
  service_server_add_virtual_obstacle_ = nh_.advertiseService("add_virtual_obstacle_in", &OctomapPlanner::callbackAddVirtualObstacle, this);

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

  timer_main_                      = nh_.createTimer(ros::Rate(_rate_main_timer_), &OctomapPlanner::timerMain, this);
  timer_future_check_              = nh_.createTimer(ros::Rate(_rate_future_check_timer_), &OctomapPlanner::timerFutureCheck, this);
  timer_diagnostics_               = nh_.createTimer(ros::Rate(_rate_diagnostics_timer_), &OctomapPlanner::timerDiagnostics, this);
  timer_publish_virtual_obstacles_ = nh_.createTimer(ros::Rate(1.0 / 2.0), &OctomapPlanner::timerPublishVirtualObstacles, this);


  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[MrsOctomapPlanner]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackPositionCmd() //{ */

void OctomapPlanner::callbackTrackerCmd([[maybe_unused]] const mrs_msgs::TrackerCommand::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  // TODO: really needed?

  ROS_INFO_ONCE("[MrsOctomapPlanner]: getting tracker cmd");
}

//}

/* timeoutTrackerCmd() //{ */

void OctomapPlanner::timeoutTrackerCmd(const std::string& topic, const ros::Time& last_msg) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_tracker_cmd_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    ROS_WARN_THROTTLE(1.0, "[MrsOctomapPlanner]: position cmd timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* callbackOctomap() //{ */

void OctomapPlanner::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[MrsOctomapPlanner]: getting octomap");

  std::optional<OcTreePtr_t> octree_local = msgToMap(msg);

  if (!octree_local) {
    ROS_WARN_THROTTLE(1.0, "[MrsOctomapPlanner]: received map is empty!");
    return;
  }

  {
    std::scoped_lock lock(mutex_octree_);

    /* copyLocalMap(*octree_local, octree_global_); */

    octree_ = octree_local.value();
    {
      std::scoped_lock lock(_mutex_virtual_obstacles);
      addVirtualObstaclesToOctree(octree_);
    }

    octree_frame_ = msg->header.frame_id;
  }

  if (!bv_planner_frame_set_) {
    bv_planner_->setParentFrame(msg->header.frame_id);
    bv_planner_frame_set_ = true;
  }

  {
    std::scoped_lock lock(mutex_bv_input_);

    bv_input_.setParentFrame(msg->header.frame_id);
  }

  {
    std::scoped_lock lock(mutex_bv_processed_);

    bv_processed_.setParentFrame(msg->header.frame_id);
  }
}

//}

/* timeoutOctomap() //{ */

void OctomapPlanner::timeoutOctomap(const std::string& topic, const ros::Time& last_msg) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_octomap_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    ROS_WARN_THROTTLE(1.0, "[MrsOctomapPlanner]: octomap timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* timeoutControlManagerDiag() //{ */

void OctomapPlanner::timeoutControlManagerDiag(const std::string& topic, const ros::Time& last_msg) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    ROS_WARN_THROTTLE(1.0, "[MrsOctomapPlanner]: Control manager diag timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* callbackStop() //{ */

bool OctomapPlanner::callbackStop([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    ROS_ERROR_STREAM_THROTTLE(0.5, "[MrsOctomapPlanner]: " << ss.str());

    res.success = false;
    res.message = ss.str();
    return true;
  }
  changeState(STATE_IDLE);
  hover();

  std::stringstream ss;
  ss << "Stopping by request";

  ROS_ERROR_STREAM_THROTTLE(0.5, "[MrsOctomapPlanner]: " << ss.str());
  res.success = true;
  res.message = ss.str();
  return true;
}

//}

/* callbackGoto() //{ */

bool OctomapPlanner::callbackGoto([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  /* prerequisities //{ */

  if (!is_initialized_) {
    return false;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    ROS_ERROR_STREAM_THROTTLE(0.5, "[MrsOctomapPlanner]: " << ss.str());

    res.success = false;
    res.message = ss.str();
    return true;
  }

  //}

  // | -------- transform the reference to the map frame -------- |

  /* { */
  /*   mrs_msgs::TrackerCommandConstPtr tracker_cmd = sh_tracker_cmd_.getMsg(); */

  /*   mrs_msgs::ReferenceStamped reference; */
  /*   reference.header.frame_id = tracker_cmd->header.frame_id; */

  /*   reference.reference.position.x = req.goal[0]; */
  /*   reference.reference.position.y = req.goal[1]; */
  /*   reference.reference.position.z = req.goal[2]; */
  /*   reference.reference.heading    = req.goal[3]; */

  /*   auto result = transformer_->transformSingle(reference, octree_frame_); */

  /*   if (result) { */

  /*     std::scoped_lock lock(mutex_user_goal_); */

  /*     user_goal_              = result.value().reference; */
  /*     new_user_goal_received_ = true; */

  /*   } else { */
  /*     std::stringstream ss; */
  /*     ss << "could not transform the reference from " << tracker_cmd->header.frame_id << " to " << octree_frame_; */

  /*     ROS_ERROR_STREAM("[MrsOctomapPlanner]: " << ss.str()); */

  /*     res.success = false; */
  /*     res.message = ss.str(); */
  /*     return true; */
  /*   } */
  /* } */

  {
    std::scoped_lock lock(mutex_user_goal_);
    user_goal_.position.x   = req.goal[0];
    user_goal_.position.y   = req.goal[1];
    user_goal_.position.z   = req.goal[2];
    user_goal_.heading      = req.goal[3];
    new_user_goal_received_ = true;
  }

  interrupted_ = false;
  changeState(STATE_PLANNING);

  {
    std::scoped_lock lock(mutex_bv_input_, mutex_user_goal_);

    bv_input_.clearBuffers();
    bv_input_.addPoint(Eigen::Vector3d(user_goal_.position.x, user_goal_.position.y, user_goal_.position.z));
    bv_input_.publish();
  }

  res.success = true;
  res.message = "goal set";
  return true;
}

//}

/* callbackReference() //{ */

bool OctomapPlanner::callbackReference([[maybe_unused]] mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  /* prerequisities //{ */

  if (!is_initialized_) {
    return false;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    ROS_ERROR_STREAM_THROTTLE(0.5, "[MrsOctomapPlanner]: " << ss.str());

    res.success = false;
    res.message = ss.str();
    return true;
  }

  //}

  // | -------- transform the reference to the map frame -------- |

  {
    mrs_msgs::TrackerCommandConstPtr tracker_cmd = sh_tracker_cmd_.getMsg();

    mrs_msgs::ReferenceStamped reference;
    reference.header    = req.header;
    reference.reference = req.reference;

    auto result = transformer_->transformSingle(reference, octree_frame_);

    if (result) {

      std::scoped_lock lock(mutex_user_goal_);

      user_goal_              = result.value().reference;
      new_user_goal_received_ = true;

    } else {
      std::stringstream ss;
      ss << "could not transform the reference from " << req.header.frame_id << " to " << octree_frame_;

      ROS_ERROR_STREAM("[MrsOctomapPlanner]: " << ss.str());

      res.success = false;
      res.message = ss.str();
      return true;
    }
  }

  interrupted_ = false;
  changeState(STATE_PLANNING);

  {
    std::scoped_lock lock(mutex_bv_input_, mutex_user_goal_);

    bv_input_.clearBuffers();
    bv_input_.addPoint(Eigen::Vector3d(user_goal_.position.x, user_goal_.position.y, user_goal_.position.z));
    bv_input_.publish();
  }

  res.success = true;
  res.message = "reference set";
  return true;
}

//}

/* callbackSetPlanner() //{ */

bool OctomapPlanner::callbackSetPlanner([[maybe_unused]] mrs_msgs::String::Request& req, mrs_msgs::String::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  ROS_INFO("[MrsOctomapPlanner]: Setting planner to %s requested.", req.value.c_str());
  res.success = true;

  if (req.value == "mrs") {
    _use_subt_planner_ = false;
  } else if (req.value == "subt") {
    _use_subt_planner_ = true;
  } else {
    res.success = false;
  }

  res.message = res.success ? "Planner set successfully." : "Invalid type of planner requested.";
  ROS_INFO("[MrsOctomapPlanner]: %s", res.message.c_str());
  return res.success;
}

//}

/* callbackSetSafetyDistance() //{ */

bool OctomapPlanner::callbackSetSafetyDistance(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (req.goal >= _safe_obstacle_distance_min_ && req.goal <= _safe_obstacle_distance_max_) {

    {
      std::scoped_lock lock(mutex_safety_distance_);

      _safe_obstacle_distance_ = req.goal;
    }

    ROS_INFO("[MrsOctomapPlanner]: setting safety distance to %.2f m.", _safe_obstacle_distance_);
    res.success = true;

  } else {

    ROS_WARN("[MrsOctomapPlanner]: failed to set safety distance %.2f m (outside the allowed range [%.2f, %.2f])", req.goal, _safe_obstacle_distance_min_,
             _safe_obstacle_distance_max_);
    res.success = false;
  }

  res.message = res.success ? "safety distance set" : "safety distance not set";

  ROS_INFO("[MrsOctomapPlanner]: %s", res.message.c_str());

  return true;
}

//}

/* callbackSetMaxAltitude() //{ */

bool OctomapPlanner::callbackSetMaxAltitude(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  {
    std::scoped_lock lock(mutex_max_altitude_);

    _max_altitude_ = req.goal;
  }

  ROS_INFO("[MrsOctomapPlanner]: setting max altitude to %.2f m.", _max_altitude_);
  res.success = true;

  res.message = "max altitude set";

  ROS_INFO("[MrsOctomapPlanner]: %s", res.message.c_str());

  return true;
}

//}
//
/* callbackAddVirtualObstacle() //{ */

bool OctomapPlanner::callbackAddVirtualObstacle(mrs_msgs::ValidateReferenceArray::Request& req, mrs_msgs::ValidateReferenceArray::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  if (req.array.header.frame_id != octree_frame_) {
    res.success = {false};
    res.message = "frame_id doesn't match octree frame -> cannot insert virtual obstacle";
    return false;
  }

  if (req.array.array.size() != 3) {
    res.success = {false};
    res.message = "array does not contain exactly 3 points -> cannot insert virtual obstacle";
    return false;
  }

  // Define the 12 edges by point pairs
  const std::vector<std::pair<int, int>> edge_indices = {
      {0, 1}, {0, 2}, {1, 3}, {2, 3},  // bottom face
      {4, 5}, {5, 7}, {7, 6}, {6, 4},  // top face
      {0, 4}, {1, 5}, {2, 6}, {3, 7}   // vertical edges
  };

  VirtualObstacle_t obst;
  obst.frame_id = req.array.header.frame_id;
  obst.vertices.resize(8);
  obst.uvw.resize(3);

  auto& p0 = obst.vertices[0];
  auto& p1 = obst.vertices[1];
  auto& p2 = obst.vertices[2];
  auto& p3 = obst.vertices[3];
  auto& p4 = obst.vertices[4];
  auto& p5 = obst.vertices[5];
  auto& p6 = obst.vertices[6];
  auto& p7 = obst.vertices[7];

  auto& u = obst.uvw[0];
  auto& v = obst.uvw[1];
  auto& w = obst.uvw[2];

  // | -------------------- Compute vertices -------------------- |
  const auto& pts   = req.array.array;
  const float min_z = float(std::fmin(pts.at(0).position.z, std::fmin(pts.at(1).position.z, pts.at(2).position.z)));
  const float max_z = float(std::fmax(pts.at(0).position.z, std::fmax(pts.at(1).position.z, pts.at(2).position.z)));

  p0 = Eigen::Vector3f(pts.at(0).position.x, pts.at(0).position.y, min_z);
  p1 = Eigen::Vector3f(pts.at(1).position.x, pts.at(1).position.y, min_z);
  p2 = Eigen::Vector3f(pts.at(2).position.x, pts.at(2).position.y, min_z);

  obst.height = max_z - min_z;

  u = p1 - p0;  // base edge 1
  v = p2 - p0;  // base edge 2

  Eigen::Vector3f normal = u.cross(v).normalized();  // height direction
  normal[2]              = std::fabs(normal.z());    // always point up in the given frame
  w                      = normal * obst.height;     // height vector

  // Bottom face corner (last one)
  p3 = p2 + u;

  // Top face
  p4 = p0 + w;
  p5 = p1 + w;
  p6 = p2 + w;
  p7 = p3 + w;

  // | -------------------- Setup vis marker -------------------- |
  visualization_msgs::Marker edges;
  auto&                      marker = obst.vis_marker;
  marker.header.frame_id            = octree_frame_;
  marker.header.stamp               = ros::Time::now();
  marker.ns                         = "edges";
  marker.id                         = 0;
  marker.type                       = visualization_msgs::Marker::LINE_LIST;
  marker.action                     = visualization_msgs::Marker::ADD;
  marker.scale.x                    = 0.02;  // line width

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.points.reserve(edge_indices.size());
  for (const auto& e : edge_indices) {
    geometry_msgs::Point p_start;
    geometry_msgs::Point p_end;

    p_start.x = obst.vertices[e.first].x();
    p_start.y = obst.vertices[e.first].y();
    p_start.z = obst.vertices[e.first].z();
    p_end.x   = obst.vertices[e.second].x();
    p_end.y   = obst.vertices[e.second].y();
    p_end.z   = obst.vertices[e.second].z();
    marker.points.push_back(p_start);
    marker.points.push_back(p_end);
  }

  {
    std::scoped_lock lock(_mutex_virtual_obstacles);

    _virtual_obstacles.push_back(obst);
  }

  ROS_INFO("[MrsOctomapPlanner]: adding virtual obstacle");
  ROS_INFO("                        p1: %.1f %.1f %.1f", p1.x(), p1.y(), p1.z());
  ROS_INFO("                        p2: %.1f %.1f %.1f", p2.x(), p2.y(), p2.z());
  ROS_INFO("                        p3: %.1f %.1f %.1f", p3.x(), p3.y(), p3.z());

  res.success = {true};
  res.message = "obstacle added";

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void OctomapPlanner::timerMain([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  /* prerequsities //{ */

  const bool got_octomap              = sh_octomap_.hasMsg() && (ros::Time::now() - sh_octomap_.lastMsgTime()).toSec() < 2.0;
  const bool got_tracker_cmd          = sh_tracker_cmd_.hasMsg() && (ros::Time::now() - sh_tracker_cmd_.lastMsgTime()).toSec() < 2.0;
  const bool got_control_manager_diag = sh_control_manager_diag_.hasMsg() && (ros::Time::now() - sh_control_manager_diag_.lastMsgTime()).toSec() < 2.0;
  const bool got_constraints          = sh_constraints_.hasMsg() && (ros::Time::now() - sh_constraints_.lastMsgTime()).toSec() < 2.0;

  if (!got_octomap || !got_tracker_cmd || !got_control_manager_diag || !got_constraints) {
    ROS_INFO_THROTTLE(1.0, "[MrsOctomapPlanner]: waiting for data: octomap = %s, position cmd = %s, ControlManager diag = %s, constraints = %s",
                      got_octomap ? "TRUE" : "FALSE", got_tracker_cmd ? "TRUE" : "FALSE", got_control_manager_diag ? "TRUE" : "FALSE",
                      got_constraints ? "TRUE" : "FALSE");
    return;
  } else {
    ready_to_plan_ = true;
  }

  //}

  ROS_INFO_ONCE("[MrsOctomapPlanner]: main timer spinning");

  const auto user_goal = mrs_lib::get_mutexed(mutex_user_goal_, user_goal_);
  if (new_user_goal_received_) {
    first_planning_for_current_goal_ = true;
    new_user_goal_received_          = false;
  }

  const mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();
  const mrs_msgs::TrackerCommandConstPtr            tracker_cmd          = sh_tracker_cmd_.getMsg();

  std::shared_ptr<OcTree_t> octree;

  {
    std::scoped_lock lock(mutex_octree_);

    octree = std::make_shared<OcTree_t>(*octree_);
  }

  octomap::point3d user_goal_octpoint;
  user_goal_octpoint.x() = user_goal.position.x;
  user_goal_octpoint.y() = user_goal.position.y;
  user_goal_octpoint.z() = user_goal.position.z;

  {
    std::scoped_lock lock(mutex_diagnostics_);

    diagnostics_.header.stamp    = ros::Time::now();
    diagnostics_.header.frame_id = octree_frame_;
    diagnostics_.idle            = false;

    diagnostics_.desired_reference.x = user_goal.position.x;
    diagnostics_.desired_reference.y = user_goal.position.y;
    diagnostics_.desired_reference.z = user_goal.position.z;
  }

  switch (state_) {

      /* STATE_IDLE //{ */

    case STATE_IDLE: {

      {
        std::scoped_lock lock(mutex_diagnostics_);

        diagnostics_.idle = true;
      }

      break;
    }

      //}

      /* STATE_PLANNING //{ */

    case STATE_PLANNING: {

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerMain - STATE_PLANNING", ros::Duration(_scope_timer_duration_), _scope_timer_enabled_);

      if (!octree->getRoot()) {

        ROS_ERROR("[MrsOctomapPlanner]: don't have a map");

        changeState(STATE_IDLE);

        break;
      }

      {
        std::scoped_lock lock(mutex_diagnostics_);

        diagnostics_.idle = false;
      }

      if (replanning_counter_ >= _max_attempts_to_replan_) {

        ROS_ERROR("[MrsOctomapPlanner]: planning failed, the uav is stuck");

        changeState(STATE_IDLE);

        break;
      }

      /* get the initial condition (predicted pose of UAV at a specific time) */ /*//{*/
      double time_for_planning;

      if (control_manager_diag->tracker_status.have_goal) {
        time_for_planning = _timeout_threshold_;
      } else {
        time_for_planning = _timeout_threshold_ + pow(1.5, float(replanning_counter_));
      }

      ROS_INFO("[MrsOctomapPlanner]: planning timeout %.2f s", time_for_planning);

      ros::Time init_cond_time = ros::Time::now() + ros::Duration(time_for_planning + _time_for_trajectory_generator_);

      ROS_INFO("[MrsOctomapPlanner]: init cond time %.2f s", init_cond_time.toSec());

      timer.checkpoint("before getInitialCondition");

      int  iter              = 0;
      auto initial_condition = getInitialCondition(init_cond_time);

      while (!initial_condition && iter < 20) {

        initial_condition = getInitialCondition(init_cond_time);
        ROS_WARN("[MrsOctomapPlanner]: Trying to get initial condition updated with the last sent path.");
        ros::Duration(0.005).sleep();
        iter++;
      }

      timer.checkpoint("after getInitialCondition()");

      if (!initial_condition) {

        ROS_ERROR_THROTTLE(1.0, "[MrsOctomapPlanner]: could not obtain initial condition for planning");
        hover();
        changeState(STATE_IDLE);

        break;
      }

      ROS_INFO("[MrsOctomapPlanner]: init cond time stamp %.2f", initial_condition.value().header.stamp.toSec());

      octomap::point3d plan_from;
      plan_from.x() = initial_condition.value().reference.position.x;
      plan_from.y() = initial_condition.value().reference.position.y;
      plan_from.z() = initial_condition.value().reference.position.z;
      /*//}*/

      /* check if goal was reached */ /*//{*/
      if ((plan_from - user_goal_octpoint).norm() <= _min_path_length_ &&
          mrs_lib::geometry::radians::dist(initial_condition.value().reference.heading, user_goal_.heading) < 0.1) {

        ROS_INFO_THROTTLE(1.0, "[MrsOctomapPlanner]: we reached the target");
        changeState(STATE_IDLE);
        break;
      }
      /*//}*/

      /* plan the path to goal */ /*//{*/
      std::pair<std::vector<octomap::point3d>, bool> waypoints;
      /* ros::Time                                      mct_start = ros::Time::now(); */

      auto safe_obstacle_distance = mrs_lib::get_mutexed(mutex_safety_distance_, _safe_obstacle_distance_);
      auto max_altitude           = mrs_lib::get_mutexed(mutex_max_altitude_, _max_altitude_);


      if ((plan_from - user_goal_octpoint).norm() <=
          2 * sqrt(3) * planning_tree_resolution_) {  // enable planning on a short range and change of the heading without translation

        octomap::KeyRay key_ray;
        bool            is_direct_path_collision_free = true;
        if (octree->computeRayKeys(plan_from, user_goal_octpoint, key_ray)) {
          for (octomap::KeyRay::iterator it1 = key_ray.begin(), end = key_ray.end(); it1 != end; ++it1) {
            auto node = octree->search(*it1);
            if (node && octree->isNodeOccupied(node)) {
              is_direct_path_collision_free = false;
              break;
            }
          }
        } else {
          ROS_ERROR_THROTTLE(0.1, "[MrsOctomapPlanner]: collision check of short direct path failed, could not raytrace!");
          is_direct_path_collision_free = false;
          break;
        }

        if (is_direct_path_collision_free) {
          ROS_INFO("[OctomapPlanner]: Using direct_collision_free_path.");
          waypoints.second = true;
          waypoints.first.push_back(plan_from);

        } else {

          ROS_WARN_THROTTLE(1.0, "[MrsOctomapPlanner]: direct short path does not exist.");
          changeState(STATE_IDLE);
          break;
        }

      } else {

        if (_use_subt_planner_) {

          // | -------------------- MRS SubT planner -------------------- |
          mrs_subt_planning::AstarPlanner subt_planner = mrs_subt_planning::AstarPlanner();

          subt_planner.initialize(true, time_for_planning - _subt_processing_timeout_, _subt_processing_timeout_, safe_obstacle_distance, _subt_clearing_dist_,
                                  _min_altitude_, max_altitude, _subt_debug_info_, bv_planner_, false);
          subt_planner.setAstarAdmissibility(_subt_admissibility_);

          ROS_INFO("[MrsOctomapPlanner]: Calling find path method.");

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = ros::Time::now();
          }

          waypoints = subt_planner.findPath(plan_from, user_goal_octpoint, octree, _subt_make_path_straight_, _subt_apply_postprocessing_,
                                            _subt_bbx_horizontal_, _subt_bbx_vertical_, _subt_processing_safe_dist_, _subt_processing_max_iterations_,
                                            _subt_processing_horizontal_neighbors_only_, _subt_processing_z_diff_tolerance_, _subt_processing_path_length_,
                                            _subt_shortening_window_size_, _subt_shortening_distance_, _subt_apply_pruning_, _subt_pruning_dist_, false, 2.0,
                                            _subt_remove_obsolete_points_, _subt_obsolete_points_tolerance_);

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = ros::Time(0);
          }

        } else {

          ROS_INFO("[MrsOctomapPlanner]: Initializing astar planner.");
          mrs_octomap_planner::AstarPlanner planner = mrs_octomap_planner::AstarPlanner(
              safe_obstacle_distance, _euclidean_distance_cutoff_, _distance_transform_distance_, planning_tree_resolution_, _distance_penalty_,
              _greedy_penalty_, _timeout_threshold_, _max_waypoint_distance_, _min_altitude_, max_altitude, _unknown_is_occupied_, bv_planner_);

          ROS_INFO("[MrsOctomapPlanner]: Calling find path method.");

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = ros::Time::now();
          }

          waypoints = planner.findPath(plan_from, user_goal_octpoint, octree, time_for_planning);

          ROS_INFO("[MrsOctomapPlanner]: Path planning finished.");

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = ros::Time(0);
          }

          ROS_INFO("[MrsOctomapPlanner]: after mutex planner time flag");
        }
        ROS_INFO("[MrsOctomapPlanner]: behind else");
      }

      ROS_INFO("[MrsOctomapPlanner]: behind else 2");
      timer.checkpoint("after findPath()");
      ROS_INFO("[MrsOctomapPlanner]: after timer");

      // path is complete
      if (waypoints.second) {

        ROS_INFO("[MrsOctomapPlanner]: Path is complete");
        replanning_counter_ = 0;

        ROS_INFO("[MrsOctomapPlanner]: Replanning counter = 0");
        waypoints.first.push_back(user_goal_octpoint);

        ROS_INFO("[MrsOctomapPlanner]: Path is complete. Path length = %lu", waypoints.first.size());
        ROS_INFO("[MrsOctomapPlanner]: Path is complete last line");

      } else {

        ROS_INFO("[MrsOctomapPlanner]: Path is not complete");
        ROS_INFO("[MrsOctomapPlanner]: path length: %lu", waypoints.first.size());
        ROS_INFO("[MrsOctomapPlanner]: Path is not complete behind print");
        if (waypoints.first.size() < 2) {

          ROS_WARN("[MrsOctomapPlanner]: path not found");

          replanning_counter_++;

          break;
        }

        ROS_INFO("[MrsOctomapPlanner]: Path is not complete  but found");

        double front_x = waypoints.first.front().x();
        double front_y = waypoints.first.front().y();
        double front_z = waypoints.first.front().z();

        double back_x = waypoints.first.back().x();
        double back_y = waypoints.first.back().y();
        double back_z = waypoints.first.back().z();

        double path_start_end_dist = sqrt(pow(front_x - back_x, 2) + pow(front_y - back_y, 2) + pow(front_z - back_z, 2));

        if (path_start_end_dist < _min_path_length_) {

          ROS_WARN("[MrsOctomapPlanner]: path too short, length: %.3f", path_start_end_dist);

          replanning_counter_++;

          changeState(STATE_PLANNING);

          break;
        }

        // Avoiding oscillation when the goal cannot be reached
        octomap::point3d best_goal_current     = octomap::point3d(float(back_x), float(back_y), float(back_z));
        double           path_end_to_goal_dist = (plan_from - user_goal_octpoint).norm();

        if (!detected_collision_ && !first_planning_for_current_goal_ && path_end_to_goal_dist > _goal_reached_dist_ &&
            path_end_to_goal_dist < _max_goal_dist_to_disable_replanning_) {  // goal unreachable

          octomap::point3d best_goal_prev = octomap::point3d(diagnostics_.best_goal.x, diagnostics_.best_goal.y, diagnostics_.best_goal.z);

          double dist_to_goal_prev    = (best_goal_prev - user_goal_octpoint).norm();
          double dist_to_goal_current = (best_goal_current - user_goal_octpoint).norm();

          if (dist_to_goal_prev - dist_to_goal_current < _min_dist_to_goal_improvement_) {

            ROS_INFO_THROTTLE(1.0, "[MrsOctomapPlanner]: Insufficient improvement in dist to goal. Aborting the planning to avoid oscillations.");

            replanning_counter_++;

            changeState(STATE_PLANNING);

            break;
          }
        }
      }

      ROS_INFO("[MrsOctomapPlanner]: After condition 1.");

      time_last_plan_                  = ros::Time::now();
      first_planning_for_current_goal_ = false;
      detected_collision_              = false;

      ROS_INFO("[OctomapPlanner]: Waiting for mutex diagnostics");

      {
        std::scoped_lock lock(mutex_diagnostics_);

        diagnostics_.best_goal.x = waypoints.first.back().x();
        diagnostics_.best_goal.y = waypoints.first.back().y();
        diagnostics_.best_goal.z = waypoints.first.back().z();
      }

      ROS_INFO("[OctomapPlanner]: After mutex diagnostics");

      /*//}*/

      ROS_INFO("[OctomapPlanner]: Waiting for mutex initial condition (unused)");

      {
        std::scoped_lock lock(mutex_initial_condition_);

        mrs_msgs::TrackerCommandConstPtr tracker_cmd = sh_tracker_cmd_.getMsg();

        ROS_INFO("[OctomapPlanner]: Waiting for mutex octree");
        auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
        ROS_INFO("[OctomapPlanner]: After mutex octree");

        // transform the position cmd to the map frame
        mrs_msgs::ReferenceStamped position_cmd_ref;
        position_cmd_ref.header               = tracker_cmd->header;
        position_cmd_ref.reference.position.x = tracker_cmd->position.x;
        position_cmd_ref.reference.position.y = tracker_cmd->position.y;
        position_cmd_ref.reference.position.z = tracker_cmd->position.z;
        position_cmd_ref.reference.heading    = tracker_cmd->heading;

        auto res = transformer_->transformSingle(position_cmd_ref, octree_frame);

        if (!res) {
          ROS_ERROR("[MrsOctomapPlanner]: could not transform position cmd to the map frame");
          return;
        }

        initial_pos_.x() = res.value().reference.position.x;
        initial_pos_.y() = res.value().reference.position.y;
        initial_pos_.z() = res.value().reference.position.z;
        initial_heading_ = res.value().reference.heading;
      }

      ROS_INFO("[OctomapPlanner]: After mutex initial condition (unused)");

      ros::Time path_stamp = initial_condition.value().header.stamp;

      if (ros::Time::now() > path_stamp || !control_manager_diag->tracker_status.have_goal) {
        path_stamp = ros::Time(0);
      }

      ROS_INFO("[MrsOctomapPlanner]: Calling path service with timestamp = %.3f at time %.3f.", path_stamp.toSec(), ros::Time::now().toSec());
      ros::Time tg_start = ros::Time::now();

      mrs_msgs::GetPathSrv srv_get_path;
      srv_get_path.request.path.header.frame_id = octree_frame_;
      srv_get_path.request.path.header.stamp    = path_stamp;
      srv_get_path.request.path.fly_now         = false;
      srv_get_path.request.path.relax_heading   = _trajectory_generation_relax_heading_;
      srv_get_path.request.path.use_heading     = _trajectory_generation_use_heading_;

      std::vector<Eigen::Vector4d> eig_waypoints;

      // create an array of Eigen waypoints
      for (auto& w : waypoints.first) {

        Eigen::Vector4d eig_waypoint;
        eig_waypoint[0] = w.x();
        eig_waypoint[1] = w.y();
        eig_waypoint[2] = w.z();
        eig_waypoint[3] = user_goal.heading;

        eig_waypoints.push_back(eig_waypoint);
      }

      std::vector<double> segment_times = estimateSegmentTimes(eig_waypoints, false);

      double cum_time = 0;
      double cum_dist = 0;
      double dx, dy;
      int    end_idx;

      for (int i = 0; i < waypoints.first.size(); i++) {

        mrs_msgs::Reference ref;
        ref.position.x = waypoints.first[i].x();
        ref.position.y = waypoints.first[i].y();
        ref.position.z = waypoints.first[i].z();

        // sample heading reference in flight direction
        if (_turn_in_flight_direction_) {

          if (i < waypoints.first.size() - 1) {  // heading in the direction of flying with prevention of fast turning due to quick changes of path directions

            cum_dist = 0.0;
            end_idx  = i + 1;
            while (cum_dist < _max_segment_length_for_heading_sampling_) {
              dx = waypoints.first[end_idx].x() - ref.position.x;
              dy = waypoints.first[end_idx].y() - ref.position.y;
              cum_dist += (waypoints.first[end_idx] - waypoints.first[end_idx - 1]).norm();
              end_idx = fmin(++end_idx, waypoints.first.size() - 1);
            }

            if (fabs(dx) > 1e-3 || fabs(dy) > 1e-3) {
              ref.heading = atan2(dy, dx) + _heading_offset_;
            } else {
              if (i > 0) {
                ref.heading = srv_get_path.request.path.points.back().heading;
              } else {
                ref.heading = initial_heading_;
              }
            }
          } else {
            if (waypoints.first.size() > 1) {
              ref.heading = srv_get_path.request.path.points.back().heading;
            } else {
              ref.heading = initial_heading_;
            }
          }
        } else {

          ref.heading = user_goal.heading;
        }

        if (i > 0) {
          double waypoint_dist = (waypoints.first[i] - waypoints.first[i - 1]).norm();
          if (waypoint_dist > _max_segment_length_for_heading_sampling_) {
            mrs_msgs::Reference inter_ref;
            inter_ref.position.x =
                waypoints.first[i - 1].x() + (waypoints.first[i].x() - waypoints.first[i - 1].x()) / waypoint_dist * _max_segment_length_for_heading_sampling_;
            inter_ref.position.y =
                waypoints.first[i - 1].y() + (waypoints.first[i].y() - waypoints.first[i - 1].y()) / waypoint_dist * _max_segment_length_for_heading_sampling_;
            inter_ref.position.z =
                waypoints.first[i - 1].z() + (waypoints.first[i].z() - waypoints.first[i - 1].z()) / waypoint_dist * _max_segment_length_for_heading_sampling_;
            inter_ref.heading = ref.heading;
            srv_get_path.request.path.points.push_back(inter_ref);
            ROS_INFO("[MrsOctomapPlanner]: TG inter input point %02d: [%.2f, %.2f, %.2f, %.2f]", i, inter_ref.position.x, inter_ref.position.y,
                     inter_ref.position.z, inter_ref.heading);
          }
        }

        ROS_INFO("[MrsOctomapPlanner]: TG input point %02d: [%.2f, %.2f, %.2f, %.2f]", i, ref.position.x, ref.position.y, ref.position.z, ref.heading);
        srv_get_path.request.path.points.push_back(ref);

        cum_time += segment_times[i];

        if (i > 1 && cum_time > _trajectory_generation_input_length_) {
          ROS_INFO("[MrsOctomapPlanner]: cutting path in waypoint %d out of %d", i, int(waypoints.first.size()));
          break;
        }
      }

      if (interrupted_) {
        ROS_WARN("[MrsOctomapPlanner]: planner interrupted, breaking main timer");
        break;
      }

      ROS_INFO("[MrsOctomapPlanner]: calling trajectory generation");

      timer.checkpoint("calling trajectory generation");

      {
        bool success = sc_get_trajectory_.call(srv_get_path);

        if (!success) {
          ROS_ERROR("[MrsOctomapPlanner]: service call for trajectory failed");
          break;
        } else {
          if (!srv_get_path.response.success) {
            ROS_ERROR("[MrsOctomapPlanner]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
            break;
          }
        }
      }

      ROS_INFO("[MrsOctomapPlanner]: Trajectory generation took %.2f s", (ros::Time::now() - tg_start).toSec());

      {
        std::scoped_lock lock(mutex_bv_processed_);

        bv_processed_.clearBuffers();
        for (auto& p : srv_get_path.response.trajectory.points) {
          auto v = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
          bv_processed_.addPoint(v, 0, 1, 0, 1);
        }
        bv_processed_.publish();
      }

      auto trajectory = srv_get_path.response.trajectory;

      // check if the trajectory is safe
      bool ray_is_cool = true;
      for (int i = 0; i < trajectory.points.size() - 1; i++) {
        // check for obstacles between the path waypoints
        octomap::point3d point1(trajectory.points[i].position.x, trajectory.points[i].position.y, trajectory.points[i].position.z);
        octomap::point3d point2(trajectory.points[i + 1].position.x, trajectory.points[i + 1].position.y, trajectory.points[i + 1].position.z);

        octomap::KeyRay key_ray;

        if (octree->computeRayKeys(point1, point2, key_ray)) {
          for (octomap::KeyRay::iterator it1 = key_ray.begin(), end = key_ray.end(); it1 != end; ++it1) {
            auto node = octree->search(*it1);
            if (node && octree->isNodeOccupied(node)) {
              ROS_ERROR_THROTTLE(0.1, "[MrsOctomapPlanner]: trajectory check found collision with prediction horizon between %d and %d, replanning!", i, i + 1);
              ray_is_cool = false;
              break;
            }
          }
        } else {
          ROS_ERROR_THROTTLE(0.1, "[MrsOctomapPlanner]: trajectory check failed, could not raytrace!");
          ray_is_cool = false;
          break;
        }
      }
      if (!ray_is_cool) {
        replanning_counter_++;
        detected_collision_ = true;
        changeState(STATE_PLANNING);
        break;
      }

      ROS_INFO("[MrsOctomapPlanner]: Setting replanning point");
      setReplanningPoint(trajectory);
      set_timepoints_ = true;

      // set user goal heading for final point such that it is achieved independently on length of the trajectory
      if (_use_user_heading_for_final_point_ && !srv_get_path.response.trajectory.points.empty()) {
        srv_get_path.response.trajectory.points.push_back(srv_get_path.response.trajectory.points.back());
        srv_get_path.response.trajectory.points.back().heading = user_goal_.heading;
      }

      ROS_INFO("[MrsOctomapPlanner]: publishing trajectory reference");

      mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
      srv_trajectory_reference.request.trajectory         = srv_get_path.response.trajectory;
      srv_trajectory_reference.request.trajectory.fly_now = true;

      // set id of trajectory
      path_id_++;
      srv_trajectory_reference.request.trajectory.input_id = path_id_;

      int cb = 0;
      ROS_INFO("[MrsOctomapPlanner]: Mrs trajectory generation output:");

      for (auto& point : srv_trajectory_reference.request.trajectory.points) {
        ROS_INFO("[MrsOctomapPlanner]: Trajectory point %02d: [%.2f, %.2f, %.2f, %.2f]", cb, point.position.x, point.position.y, point.position.z,
                 point.heading);
        cb++;
      }

      ROS_INFO("[MrsOctomapPlanner]: Calling trajectory service with timestamp = %.3f at time %.3f.",
               srv_trajectory_reference.request.trajectory.header.stamp.toSec(), ros::Time::now().toSec());

      {
        bool success = sc_trajectory_reference_.call(srv_trajectory_reference);

        if (!success) {
          ROS_ERROR("[MrsOctomapPlanner]: service call for trajectory reference failed");
          break;
        } else {
          if (!srv_trajectory_reference.response.success) {
            ROS_ERROR("[MrsOctomapPlanner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
            break;
          }
        }
      }

      changeState(STATE_MOVING);
      break;
    }

      //}

      /* STATE_MOVING //{ */

    case STATE_MOVING: {

      {
        std::scoped_lock lock(mutex_diagnostics_);

        diagnostics_.idle = false;
      }

      /* std::scoped_lock lock(mutex_initial_condition_); */
      mrs_msgs::TrackerCommandConstPtr tracker_cmd  = sh_tracker_cmd_.getMsg();
      auto                             octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);

      // transform the position cmd to the map frame
      mrs_msgs::ReferenceStamped position_cmd_ref;
      position_cmd_ref.header               = tracker_cmd->header;
      position_cmd_ref.reference.position.x = tracker_cmd->position.x;
      position_cmd_ref.reference.position.y = tracker_cmd->position.y;
      position_cmd_ref.reference.position.z = tracker_cmd->position.z;
      position_cmd_ref.reference.heading    = tracker_cmd->heading;

      auto res = transformer_->transformSingle(position_cmd_ref, octree_frame);

      if (!res) {
        ROS_ERROR("[MrsOctomapPlanner]: could not transform position cmd to the map frame");
        return;
      }

      octomap::point3d position_cmd_octomap;
      position_cmd_octomap.x() = res.value().reference.position.x;
      position_cmd_octomap.y() = res.value().reference.position.y;
      position_cmd_octomap.z() = res.value().reference.position.z;

      /* auto initial_pos = mrs_lib::get_mutexed(mutex_initial_condition_, initial_pos_); */

      /* double dist_to_goal = (initial_pos - user_goal_octpoint).norm(); */
      double dist_to_goal = (position_cmd_octomap - user_goal_octpoint).norm();

      ROS_INFO_THROTTLE(1.0, "[MrsOctomapPlanner]: dist to goal: %.2f m", dist_to_goal);

      if (dist_to_goal < _goal_reached_dist_) {
        ROS_INFO("[MrsOctomapPlanner]: user goal reached");
        changeState(STATE_IDLE);
        break;
      }

      if ((ros::Time::now() - (time_last_plan_ + ros::Duration(_replan_after_))).toSec() > 0) {

        ROS_INFO("[MrsOctomapPlanner]: triggering replanning");

        changeState(STATE_PLANNING);
      }

      break;
    }

      //}
  }
}

//}

/* timerFutureCheck() //{ */

void OctomapPlanner::timerFutureCheck([[maybe_unused]] const ros::TimerEvent& evt) {

  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: start");

  if (!is_initialized_) {
    return;
  }

  /* preconditions //{ */

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: missing control manager");
    return;
  }

  if (!sh_octomap_.hasMsg()) {
    ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: missing octomap");
    return;
  }

  //}

  if (state_ == STATE_IDLE) {
    ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: is idle");
    return;
  }

  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: after checks");

  ROS_INFO_ONCE("[MrsOctomapPlanner]: future check timer spinning");

  const mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("timerFutureCheck", ros::Duration(_scope_timer_duration_), _scope_timer_enabled_);

  std::shared_ptr<OcTree_t> octree;

  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: waiting for mutex octree");
  {
    std::scoped_lock lock(mutex_octree_);

    octree = std::make_shared<OcTree_t>(*octree_);
  }

  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: after mutex octree");
  // | ----------- check if the prediction is feasible ---------- |

  if (!octree->getRoot()) {
    ROS_ERROR("[MrsOctomapPlanner]: cannot check for collision, don't have a map");
    return;
  }

  mrs_msgs::MpcPredictionFullState            prediction           = sh_tracker_cmd_.getMsg()->full_state_prediction;
  mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: start checking");
  if (control_manager_diag->flying_normally && control_manager_diag->tracker_status.have_goal) {

    geometry_msgs::TransformStamped tf;

    auto ret = transformer_->getTransform(prediction.header.frame_id, octree_frame_, prediction.header.stamp);

    if (!ret) {
      ROS_ERROR_THROTTLE(1.0, "[MrsOctomapPlanner]: could not transform position cmd to the map frame! can not check for potential collisions!");
      return;
    }

    tf = ret.value();

    // prepare the potential future trajectory

    mrs_msgs::TrajectoryReference trajectory;
    trajectory.header.stamp    = ret.value().header.stamp;
    trajectory.header.frame_id = transformer_->frame_to(ret.value());
    trajectory.fly_now         = true;
    trajectory.use_heading     = _trajectory_generation_use_heading_;
    trajectory.dt              = 0.2;

    for (int i = 1; i < prediction.position.size(); i++) {

      mrs_msgs::ReferenceStamped pose;
      pose.header               = prediction.header;
      pose.reference.position.x = prediction.position[i].x;
      pose.reference.position.y = prediction.position[i].y;
      pose.reference.position.z = prediction.position[i].z;
      pose.reference.heading    = prediction.heading[i];

      auto transformed_pose = transformer_->transform(pose, tf);

      if (!transformed_pose) {
        ROS_ERROR_THROTTLE(1.0, "[MrsOctomapPlanner]: could not transform position cmd to the map frame! can not check for potential collisions!");
        return;
      }

      trajectory.points.push_back(transformed_pose->reference);
    }

    auto safe_obstacle_distance = mrs_lib::get_mutexed(mutex_safety_distance_, _safe_obstacle_distance_);

    // generate a set of points around the waypoint (2D/3D?)
    // for each point, do raycasting from current waypoint to the point
    // check for collisions
    // if something is detected, crop the trajectory
    for (int i = 0; i < trajectory.points.size(); i++) {

      octomap::point3d point1(trajectory.points[i].position.x, trajectory.points[i].position.y, trajectory.points[i].position.z);
      double           angle_step          = 2 * M_PI / _collision_check_point_count_;
      double           raycasting_distance = safe_obstacle_distance - octree->getResolution();
      bool             cropped_trajectory  = false;

      // TODO check in 3D as well??
      for (double phi = -M_PI; phi < M_PI; phi += angle_step) {
        octomap::point3d point_ray_end = point1 + octomap::point3d(raycasting_distance * cos(phi), raycasting_distance * sin(phi), 0);
        octomap::KeyRay  ray;
        if (octree->computeRayKeys(point1, point_ray_end, ray)) {
          for (octomap::KeyRay::iterator it = ray.begin(), end = ray.end(); it != end; ++it) {
            // check if the cell is occupied in the map
            auto node = octree->search(*it);
            if (!node) {
              /* ROS_WARN("[MrsOctomapPlanner]: Detected UNKNOWN space along the planned trajectory!"); */
            } else if (octree->isNodeOccupied(node)) {
              /* ROS_WARN("[MrsOctomapPlanner]: Detected OCCUPIED space along the planned trajectory!"); */
              // shorten the trajectory
              int orig_traj_size = int(trajectory.points.size());
              for (int j = int(trajectory.points.size()) - 1; j >= i - 1 && j > _min_allowed_trajectory_points_after_crop_; j--) {
                trajectory.points.pop_back();
              }

              ROS_WARN("[MrsOctomapPlanner]: Detected OCCUPIED space along the planned trajectory! Cropped the trajectory to %d from %d points.",
                       int(trajectory.points.size()), orig_traj_size);

              mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
              srv_trajectory_reference.request.trajectory = trajectory;

              bool success = sc_trajectory_reference_.call(srv_trajectory_reference);

              cropped_trajectory = true;

              if (!success) {
                ROS_ERROR("[MrsOctomapPlanner]: service call for trajectory reference failed");
                break;
              } else {
                if (!srv_trajectory_reference.response.success) {
                  ROS_ERROR("[MrsOctomapPlanner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
                  break;
                }
              }
              // TODO some mutex for octree_global_?
              break;
            }
            if (cropped_trajectory) {
              break;
            }
          }
        } else {
          ROS_WARN_THROTTLE(1.0, "[MrsOctomapPlanner]: Unable to raycast.");
        }
        if (cropped_trajectory) {
          break;
        }
      }
      if (cropped_trajectory) {
        break;
      }
    }

    // check if the trajectory is safe
    for (int i = 0; i < trajectory.points.size() - 1; i++) {

      octomap::point3d point1(trajectory.points[i].position.x, trajectory.points[i].position.y, trajectory.points[i].position.z);
      octomap::point3d point2(trajectory.points[i + 1].position.x, trajectory.points[i + 1].position.y, trajectory.points[i + 1].position.z);

      octomap::KeyRay key_ray;

      if (octree->computeRayKeys(point1, point2, key_ray)) {

        bool ray_is_cool = true;
        for (octomap::KeyRay::iterator it1 = key_ray.begin(), end = key_ray.end(); it1 != end; ++it1) {

          auto node = octree->search(*it1);
          if (node && octree->isNodeOccupied(node)) {
            ray_is_cool = false;
            break;
          }
        }

        if (!ray_is_cool) {

          ROS_ERROR_THROTTLE(0.1, "[MrsOctomapPlanner]: future check found collision with prediction horizon between %d and %d, hovering!", i, i + 1);

          // the trajectory directly passes through an obstacle, trigger hovering
          changeState(STATE_IDLE);
          hover();

          break;
        }

      } else {

        ROS_ERROR_THROTTLE(0.1, "[MrsOctomapPlanner]: future check failed, could not raytrace!");
        hover();
        break;
      }
    }
  }
  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer future: end checking");
}

//}

/* timerDiagnostics() //{ */

void OctomapPlanner::timerDiagnostics([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer diagnostics: Waiting for mutex diagnostics");

  auto diagnostics = mrs_lib::get_mutexed(mutex_diagnostics_, diagnostics_);

  try {
    pub_diagnostics_.publish(diagnostics);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_diagnostics_.getTopic().c_str());
  }

  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer diagnostics: Waiting for mutex planner time flag");
  auto planner_time_flag = mrs_lib::get_mutexed(mutex_planner_time_flag_, planner_time_flag_);

  if (_restart_planner_on_deadlock_ && planner_time_flag != ros::Time(0)) {
    if ((ros::Time::now() - planner_time_flag).toSec() > planner_deadlock_timeout_) {
      ROS_ERROR("[MrsOctomapPlanner]: Planner is deadlocked, restarting!");
      ros::shutdown();
    }
  }
  ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: Timer diagnostics: end");
}

//}

/* timerPublishVirtualObstacles() //{ */

void OctomapPlanner::timerPublishVirtualObstacles([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  if (pub_virtual_obstacles_.getNumSubscribers() == 0) {
    return;
  }

  visualization_msgs::MarkerArray ma;
  {
    std::scoped_lock lock(_mutex_virtual_obstacles);

    ma.markers.reserve(_virtual_obstacles.size());

    for (int i = 0; i < _virtual_obstacles.size(); i++) {
      auto& obst = _virtual_obstacles.at(i);

      obst.vis_marker.header.stamp = evt.current_real;
      obst.vis_marker.id           = i;
      ma.markers.push_back(obst.vis_marker);
    }
  }

  try {
    pub_virtual_obstacles_.publish(ma);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_virtual_obstacles_.getTopic().c_str());
  }
}

//}

// | ------------------------ routines ------------------------ |

/* setReplanningPoint() //{ */

void OctomapPlanner::setReplanningPoint(const mrs_msgs::TrajectoryReference& traj) {

  const float x = traj.points.back().position.x;
  const float y = traj.points.back().position.y;
  const float z = traj.points.back().position.z;

  {
    std::scoped_lock lock(mutex_replanning_point_);

    replanning_point_.x() = x;
    replanning_point_.y() = y;
    replanning_point_.z() = z;
  }

  mrs_lib::geometry::Cuboid c(Eigen::Vector3d(x, y, z), Eigen::Vector3d(0.4, 0.4, 0.4), Eigen::Quaterniond::Identity());

  {
    std::scoped_lock lock(mutex_bv_input_);

    auto user_goal = mrs_lib::get_mutexed(mutex_user_goal_, user_goal_);

    bv_input_.clearBuffers();
    bv_input_.addPoint(Eigen::Vector3d(user_goal.position.x, user_goal.position.y, user_goal.position.z), 0, 1, 0, 1);
    bv_input_.addCuboid(c, 0.5, 1.0, 1.0, 0.8, true);
    bv_input_.publish();
  }
}

//}

/* changeState() //{ */

void OctomapPlanner::changeState(const State_t new_state) {

  const State_t old_state = state_;

  if (interrupted_ && old_state == STATE_IDLE) {
    ROS_WARN("[MrsOctomapPlanner]: Planning interrupted, not changing state.");
    return;
  }

  switch (new_state) {

    case STATE_PLANNING: {

      if (old_state == STATE_IDLE) {
        replanning_counter_ = 0;
      }
    }

    default: {

      break;
    }
  }

  ROS_INFO("[MrsOctomapPlanner]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

  state_ = new_state;
}

//}

/* getInitialCondition() //{ */

std::optional<mrs_msgs::ReferenceStamped> OctomapPlanner::getInitialCondition(const ros::Time des_time) {

  const mrs_msgs::MpcPredictionFullState prediction_full_state = sh_tracker_cmd_.getMsg()->full_state_prediction;

  if (prediction_full_state.input_id != 0 && prediction_full_state.input_id < path_id_) {
    ROS_ERROR_THROTTLE(1.0, "[MrsOctomapPlanner]: could not obtain initial condition, the input_id (%lu) does not match id of last sent path (%d).",
                       prediction_full_state.input_id, path_id_);
    return {};
  }

  if (prediction_full_state.stamps.size() == 0) {
    ROS_ERROR_THROTTLE(1.0, "[MrsOctomapPlanner]: Could not obtain initial condition, prediction full state is empty.");
    return {};
  }

  if ((des_time - prediction_full_state.stamps.back()).toSec() > 0) {
    ROS_ERROR_THROTTLE(1.0, "[MrsOctomapPlanner]: could not obtain initial condition, the desired time is too far in the future");
    return {};
  }

  mrs_msgs::ReferenceStamped orig_reference;
  orig_reference.header = prediction_full_state.header;

  ros::Time future_time_stamp;

  for (int i = 0; i < prediction_full_state.stamps.size(); i++) {

    if ((prediction_full_state.stamps[i] - des_time).toSec() > 0) {
      orig_reference.reference.position.x = prediction_full_state.position[i].x;
      orig_reference.reference.position.y = prediction_full_state.position[i].y;
      orig_reference.reference.position.z = prediction_full_state.position[i].z;
      orig_reference.reference.heading    = prediction_full_state.heading[i];
      future_time_stamp                   = prediction_full_state.stamps[i];
      break;
    }
  }

  // transform the initial condition to the current map frame

  auto result = transformer_->transformSingle(orig_reference, octree_frame_);

  if (result) {

    mrs_msgs::ReferenceStamped transformed_reference = result.value();
    transformed_reference.header.stamp               = future_time_stamp;

    return transformed_reference;

  } else {

    std::stringstream ss;
    ss << "could not transform initial condition to the map frame";

    ROS_ERROR_STREAM("[MrsOctomapPlanner]: " << ss.str());
    return {};
  }
}

//}

/* hover() //{ */

void OctomapPlanner::hover(void) {

  ROS_INFO("[MrsOctomapPlanner]: triggering hover, interrupting planner");

  interrupted_ = true;

  std_srvs::Trigger srv_out;

  sc_hover_.call(srv_out);
}

//}

/* estimateSegmentTimes() //{ */

std::vector<double> OctomapPlanner::estimateSegmentTimes(const std::vector<Eigen::Vector4d>& vertices, const bool use_heading) {
  ROS_INFO("[OctomapPlanner]: Starting estimate segment time with vertices size = %lu.", vertices.size());

  if (vertices.size() <= 1) {
    return std::vector<double>(0);
  }

  ROS_INFO("[OctomapPlanner]: Continuing estimate segment time.");

  const mrs_msgs::DynamicsConstraintsConstPtr constraints = sh_constraints_.getMsg();

  const double v_max_vertical    = std::min(constraints->vertical_ascending_speed, constraints->vertical_descending_speed);
  const double a_max_vertical    = std::min(constraints->vertical_ascending_acceleration, constraints->vertical_descending_acceleration);
  const double j_max_vertical    = std::min(constraints->vertical_ascending_jerk, constraints->vertical_descending_jerk);
  const double v_max_horizontal  = constraints->horizontal_speed;
  const double a_max_horizontal  = constraints->horizontal_acceleration;
  const double j_max_horizontal  = constraints->horizontal_jerk;
  const double heading_acc_max   = constraints->heading_acceleration;
  const double heading_speed_max = constraints->heading_speed;

  std::vector<double> segment_times;
  segment_times.reserve(vertices.size() - 1);

  size_t check = vertices.size() - 1;
  ROS_INFO("[OctomapPlanner]: Continuing estimate segment time. Check = %lu", check);
  // for each vertex in the path
  for (size_t i = 0; i < vertices.size() - 1; i++) {
    ROS_INFO_THROTTLE(1.0, "[OctomapPlanner]: iterating over vertices i = %lu", vertices.size());

    Eigen::Vector3d start     = vertices[i].head(3);
    Eigen::Vector3d end       = vertices[i + 1].head(3);
    double          start_hdg = vertices[i](3);
    double          end_hdg   = vertices[i + 1](3);

    double acceleration_time_1 = 0;
    double acceleration_time_2 = 0;

    double jerk_time_1 = 0;
    double jerk_time_2 = 0;

    double acc_1_coeff = 0;
    double acc_2_coeff = 0;

    double distance = (end - start).norm();

    double inclinator = atan2(end(2) - start(2), sqrt(pow(end(0) - start(0), 2) + pow(end(1) - start(1), 2)));

    double v_max, a_max, j_max;

    if (inclinator > atan2(v_max_vertical, v_max_horizontal) || inclinator < -atan2(v_max_vertical, v_max_horizontal)) {
      v_max = fabs(v_max_vertical / sin(inclinator));
    } else {
      v_max = fabs(v_max_horizontal / cos(inclinator));
    }

    if (inclinator > atan2(a_max_vertical, a_max_horizontal) || inclinator < -atan2(a_max_vertical, a_max_horizontal)) {
      a_max = fabs(a_max_vertical / sin(inclinator));
    } else {
      a_max = fabs(a_max_horizontal / cos(inclinator));
    }

    if (inclinator > atan2(j_max_vertical, j_max_horizontal) || inclinator < -atan2(j_max_vertical, j_max_horizontal)) {
      j_max = fabs(j_max_vertical / sin(inclinator));
    } else {
      j_max = fabs(j_max_horizontal / cos(inclinator));
    }

    if (i >= 1) {

      Eigen::Vector3d pre = vertices[i - 1].head(3);

      Eigen::Vector3d vec1 = start - pre;
      Eigen::Vector3d vec2 = end - start;

      vec1.normalize();
      vec2.normalize();

      double scalar = vec1.dot(vec2) < 0 ? 0.0 : vec1.dot(vec2);

      acc_1_coeff = (1 - scalar);

      acceleration_time_1 = acc_1_coeff * ((v_max / a_max) + (a_max / j_max));

      jerk_time_1 = acc_1_coeff * (2 * (a_max / j_max));
    }

    // the first vertex
    if (i == 0) {
      acc_1_coeff         = 1.0;
      acceleration_time_1 = (v_max / a_max) + (a_max / j_max);
      jerk_time_1         = (2 * (a_max / j_max));
    }

    // last vertex
    if (i == vertices.size() - 2) {
      acc_2_coeff         = 1.0;
      acceleration_time_2 = (v_max / a_max) + (a_max / j_max);
      jerk_time_2         = (2 * (a_max / j_max));
    }

    // a vertex
    if (i < vertices.size() - 2) {

      Eigen::Vector3d post = vertices[i + 2].head(3);

      Eigen::Vector3d vec1 = end - start;
      Eigen::Vector3d vec2 = post - end;

      vec1.normalize();
      vec2.normalize();

      double scalar = vec1.dot(vec2) < 0 ? 0.0 : vec1.dot(vec2);

      acc_2_coeff = (1 - scalar);

      acceleration_time_2 = acc_2_coeff * ((v_max / a_max) + (a_max / j_max));

      jerk_time_2 = acc_2_coeff * (2 * (a_max / j_max));
    }

    if (acceleration_time_1 > sqrt(2 * distance / a_max)) {
      acceleration_time_1 = sqrt(2 * distance / a_max);
    }

    if (jerk_time_1 > sqrt(2 * v_max / j_max)) {
      jerk_time_1 = sqrt(2 * v_max / j_max);
    }

    if (acceleration_time_2 > sqrt(2 * distance / a_max)) {
      acceleration_time_2 = sqrt(2 * distance / a_max);
    }

    if (jerk_time_2 > sqrt(2 * v_max / j_max)) {
      jerk_time_2 = sqrt(2 * v_max / j_max);
    }

    double max_velocity_time;

    if (((distance - (2 * (v_max * v_max) / a_max)) / v_max) < 0) {
      max_velocity_time = ((distance) / v_max);
    } else {
      max_velocity_time = ((distance - (2 * (v_max * v_max) / a_max)) / v_max);
    }

    /* double t = max_velocity_time + acceleration_time_1 + acceleration_time_2 + jerk_time_1 + jerk_time_2; */
    double t = max_velocity_time + acceleration_time_1 + acceleration_time_2;

    /* printf("segment %d, [%.2f %.2f %.2f] - > [%.2f %.2f %.2f] = %.2f\n", i, start(0), start(1), start(2), end(0), end(1), end(2), distance); */
    /* printf("segment %d time %.2f, distance %.2f, %.2f, %.2f, %.2f, vmax: %.2f, amax: %.2f, jmax: %.2f\n", i, t, distance, max_velocity_time, */
    /*        acceleration_time_1, acceleration_time_2, v_max, a_max, j_max); */

    if (t < 0.01) {
      t = 0.01;
    }

    // | ------------- check the heading rotation time ------------ |

    double angular_distance = fabs(mrs_lib::geometry::radians::dist(start_hdg, end_hdg));

    double hdg_velocity_time     = 0;
    double hdg_acceleration_time = 0;

    if (use_heading) {

      if (heading_speed_max < std::numeric_limits<float>::max() && heading_acc_max < std::numeric_limits<float>::max()) {

        if (((angular_distance - (2 * (heading_speed_max * heading_speed_max) / heading_acc_max)) / heading_speed_max) < 0) {
          hdg_velocity_time = ((angular_distance) / heading_speed_max);
        } else {
          hdg_velocity_time = ((angular_distance - (2 * (heading_speed_max * heading_speed_max) / heading_acc_max)) / heading_speed_max);
        }

        if (angular_distance > M_PI / 4) {
          hdg_acceleration_time = 2 * (heading_speed_max / heading_acc_max);
        }
      }
    }

    // what will take longer? to fix the lateral or the heading
    double heading_fix_time = 1.5 * (hdg_velocity_time + hdg_acceleration_time);

    if (heading_fix_time > t) {
      t = heading_fix_time;
    }

    segment_times.push_back(t);
  }
  ROS_INFO("[OctomapPlanner]: End of segment times");
  return segment_times;
}

//}

/* msgToMap() //{ */

std::optional<OcTreePtr_t> OctomapPlanner::msgToMap(const octomap_msgs::OctomapConstPtr octomap) {

  octomap::AbstractOcTree* abstract_tree;

  if (octomap->binary) {
    abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);
  } else {
    abstract_tree = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!abstract_tree) {

    ROS_WARN("[MrsOctomapPlanner]: octomap message is empty!");
    return {};

  } else {

    OcTreePtr_t octree_out = OcTreePtr_t(dynamic_cast<OcTree_t*>(abstract_tree));
    return {octree_out};
  }
}

//}

/* copyLocalMap() //{ */

bool OctomapPlanner::copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to) {

  octomap::OcTreeKey minKey, maxKey;

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->setNodeValue(key, octomap::logodds(0.0));
  }

  for (OcTree_t::leaf_iterator it = from->begin_leafs(from->getTreeDepth()), end = from->end_leafs(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(it->getValue());
  }

  return true;
}

//}

/* touchNode() //{ */

octomap::OcTreeNode* OctomapPlanner::touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0) {

  return touchNodeRecurs(octree, octree->getRoot(), key, 0, target_depth);
}

//}

/* touchNodeRecurs() //{ */

octomap::OcTreeNode* OctomapPlanner::touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key,
                                                     unsigned int depth, unsigned int max_depth = 0) {

  assert(node);

  // follow down to last level
  if (depth < octree->getTreeDepth() && (max_depth == 0 || depth < max_depth)) {

    unsigned int pos = octomap::computeChildIdx(key, int(octree->getTreeDepth() - depth - 1));

    /* ROS_INFO("pos: %d", pos); */
    if (!octree->nodeChildExists(node, pos)) {

      // not a pruned node, create requested child
      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);
  }

  // at last level, update node, end of recursion
  else {

    return node;
  }
}

//}

/* addVirtualObstaclesToOctree() //{ */

void OctomapPlanner::addVirtualObstaclesToOctree(const std::shared_ptr<OcTree_t> octree) {

  if (!octree) {
    return;
  }

  const float step = 2.0f * float(planning_tree_resolution_);

  for (const auto& obst : _virtual_obstacles) {

    // Helper lambda to mark a rectangular face defined by origin and two edge vectors
    auto mark_face = [&](const Eigen::Vector3f& origin, const Eigen::Vector3f& edge1, const Eigen::Vector3f& edge2) {
      const int steps1 = std::max(1, static_cast<int>(edge1.norm() / step));
      const int steps2 = std::max(1, static_cast<int>(edge2.norm() / step));
      for (int i = 0; i <= steps1; ++i) {
        for (int j = 0; j <= steps2; ++j) {
          const Eigen::Vector3f pt = origin + (float(i) / float(steps1)) * edge1 + (float(j) / float(steps2)) * edge2;
          octree->updateNode(octomap::point3d(pt.x(), pt.y(), pt.z()), true);
        }
      }
    };

    const auto& p0 = obst.vertices[0];
    const auto& p1 = obst.vertices[1];
    const auto& p2 = obst.vertices[2];
    const auto& p3 = obst.vertices[3];
    const auto& p4 = obst.vertices[4];
    const auto& u  = obst.uvw[0];
    const auto& v  = obst.uvw[1];
    const auto& w  = obst.uvw[2];

    // Bottom face
    mark_face(p0, u, v);
    // Top face
    mark_face(p4, u, v);

    // 4 side faces
    mark_face(p0, u, w);
    mark_face(p0, v, w);
    mark_face(p1, v, w);
    mark_face(p2, u, w);
  }
}

//}

}  // namespace mrs_octomap_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_planner::OctomapPlanner, nodelet::Nodelet)
