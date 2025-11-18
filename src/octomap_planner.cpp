/* includes //{ */

#include <memory>

#include <octomap/OcTree.h>

#include <octomap_msgs/msg/octomap.h>
#include <octomap_msgs/conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/misc.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/node.h>

#include <mrs_msgs/srv/vec4.hpp>
#include <mrs_msgs/srv/vec1.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/validate_reference_array.hpp>
#include <mrs_msgs/srv/get_path_srv.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/srv/trajectory_reference_srv.hpp>

#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/mpc_prediction_full_state.hpp>
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/dynamics_constraints.hpp>
#include <mrs_msgs/msg/trajectory_reference.hpp>

#include <mrs_modules_msgs/msg/octomap_planner_diagnostics.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <astar_planner.hpp>
#include <mrs_subt_planning_lib/astar_planner.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//}

namespace mrs_octomap_planner
{

/* defines //{ */

#if USE_ROS_TIMER == 1
    typedef mrs_lib::ROSTimer TimerType;
    #else
    typedef mrs_lib::ThreadTimer TimerType;
    #endif

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
  visualization_msgs::msg::Marker   vis_marker;
};

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING"};

using OcTree_t            = octomap::OcTree;
using OcTreePtr_t         = std::shared_ptr<octomap::OcTree>;
using OcTreeMsgConstPtr_t = octomap_msgs::msg::Octomap::ConstSharedPtr;

//}

/* class OctomapPlanner //{ */

class OctomapPlanner : public mrs_lib::Node {

public:
  OctomapPlanner(const rclcpp::NodeOptions& options);

private:
  void initialize();
  std::function<void()> callback_timer_main_;
  std::function<void()> callback_timer_future_check_;
  std::function<void()> callback_timer_diagnostics_;
  std::function<void()> callback_timer_publish_virtual_obstacles_;


  std::atomic<bool> is_initialized_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

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
  double _rate_virtual_obstacle_pub_timer_;
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
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger_;
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

  rclcpp::Time  planner_time_flag_;
  std::mutex mutex_planner_time_flag_;
  bool       _restart_planner_on_deadlock_;
  double     _planner_deadlock_timeout_factor;
  double     planner_deadlock_timeout_;
  bool       avoiding_oscillations_ = false;

  // virtual obstacles params
  std::mutex                     mutex_virtual_obstacles_;
  std::vector<VirtualObstacle_t> virtual_obstacles_;
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


  // callback groups 

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  // subscribers
  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>            sh_tracker_cmd_;
  mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>               sh_octomap_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::DynamicsConstraints>       sh_constraints_;

  // publishers

  mrs_lib::PublisherHandler<mrs_modules_msgs::msg::OctomapPlannerDiagnostics> pub_diagnostics_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_virtual_obstacles_;

  // subscriber callbacks
  void callbackTrackerCmd(const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);
  void callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

  // service servers
  rclcpp::Service<mrs_msgs::srv::Vec4>::SharedPtr                     service_server_goto_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                  service_server_stop_; 
  rclcpp::Service<mrs_msgs::srv::ReferenceStampedSrv>::SharedPtr      service_server_reference_;
  rclcpp::Service<mrs_msgs::srv::String>::SharedPtr                   service_server_set_planner_;
  rclcpp::Service<mrs_msgs::srv::Vec1>::SharedPtr                     service_server_set_safety_distance_;
  rclcpp::Service<mrs_msgs::srv::Vec1>::SharedPtr                     service_server_set_max_altitude_;
  rclcpp::Service<mrs_msgs::srv::ValidateReferenceArray>::SharedPtr   service_server_add_virtual_obstacle_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                  service_server_remove_virtual_obstacles_;
  
  // service server callbacks
  void callbackGoto(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> req,std::shared_ptr<mrs_msgs::srv::Vec4::Response> res);
  void callbackStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void callbackReference(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> req, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> res);
  void callbackSetPlanner(const std::shared_ptr<mrs_msgs::srv::String::Request> req, std::shared_ptr<mrs_msgs::srv::String::Response> res);
  void callbackSetSafetyDistance(const std::shared_ptr<mrs_msgs::srv::Vec1::Request> req, std::shared_ptr<mrs_msgs::srv::Vec1::Response> res);
  void callbackSetMaxAltitude(const std::shared_ptr<mrs_msgs::srv::Vec1::Request> req, std::shared_ptr<mrs_msgs::srv::Vec1::Response> res);
  void callbackAddVirtualObstacle(const std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Request> req, std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Response> res);
  void callbackRemoveVirtualObstacles(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // service clients
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv>             sc_get_trajectory_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv> sc_trajectory_reference_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>                sc_hover_;

  // timers

  std::shared_ptr<TimerType> timer_main_;
  std::shared_ptr<TimerType> timer_diagnostics_;
  std::shared_ptr<TimerType> timer_future_check_;
  std::shared_ptr<TimerType> timer_publish_virtual_obstacles_;

  void timerMain();
  void timerDiagnostics();
  void timerFutureCheck();
  void timerPublishVirtualObstacles();

  // diagnostics
  mrs_modules_msgs::msg::OctomapPlannerDiagnostics diagnostics_;
  std::mutex                                  mutex_diagnostics_;

  // timeouts
  void timeoutOctomap(const std::string& topic, const rclcpp::Time& last_msg);
  void timeoutTrackerCmd(const std::string& topic, const rclcpp::Time& last_msg);
  void timeoutControlManagerDiag(const std::string& topic, const rclcpp::Time& last_msg);

  // transformer
  std::unique_ptr<mrs_lib::Transformer> transformer_;

  std::string current_control_frame_;
  std::mutex  mutex_current_control_frame_;

  // planning
  std::atomic<int> replanning_counter_ = 0;
  rclcpp::Time     time_last_plan_;
  int              path_id_                         = 0;
  bool             new_user_goal_received_          = false;
  bool             first_planning_for_current_goal_ = false;
  bool             detected_collision_              = false;

  // state machine
  std::atomic<State_t> state_;
  void                 changeState(const State_t new_state);
  std::atomic<bool>    interrupted_ = false;

  mrs_msgs::msg::Reference user_goal_;
  std::mutex          mutex_user_goal_;

  octomap::point3d internal_goal_;

  std::atomic<bool> set_timepoints_ = false;

  rclcpp::Time replanning_start_timepoint_;
  rclcpp::Time replanning_end_timepoint_;

  // unused
  octomap::point3d replanning_point_;
  std::mutex       mutex_replanning_point_;

  // routines
  void setReplanningPoint(const mrs_msgs::msg::TrajectoryReference& traj);

  std::vector<double> estimateSegmentTimes(const std::vector<Eigen::Vector4d>& vertices, const bool use_heading);

  std::optional<OcTreePtr_t> msgToMap(const octomap_msgs::msg::Octomap::ConstSharedPtr octomap);

  /**
   * @brief returns planning initial condition for a given future time based on the MPC prediction horizon
   *
   * @param time
   *
   * @return x, y, z, heading reference
   */
  std::optional<mrs_msgs::msg::ReferenceStamped> getInitialCondition(const rclcpp::Time time);

  bool copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth);

  void hover(void);
};

//}

/* OctomapServer() //{ */

OctomapPlanner::OctomapPlanner(const rclcpp::NodeOptions& options) : mrs_lib::Node("octomap_planner", options) {
  initialize();
}

/* initialize() //{ */

void OctomapPlanner::initialize() {
  node_ = this->this_node_ptr(); 
  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(),"initializing");


  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // load files 

  mrs_lib::ParamLoader param_loader(node_);

  // load custom config
  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    RCLCPP_INFO(node_->get_logger(), "loading custom config '%s", custom_config_path.c_str());
    param_loader.addYamlFile(custom_config_path);
  }

  // load other configs

  std::vector<std::string> config_files;
  param_loader.loadParam("config_files", config_files);

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'", config_file.c_str());
    param_loader.addYamlFile(config_file);
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("main_timer/rate", _rate_main_timer_);
  param_loader.loadParam("diagnostics_timer/rate", _rate_diagnostics_timer_);
  param_loader.loadParam("future_check_timer/rate", _rate_future_check_timer_);
  param_loader.loadParam("virtual_obstacle_publishing_timer/rate", _rate_virtual_obstacle_pub_timer_);
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
    RCLCPP_ERROR(node_->get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
  }

  if (_goal_reached_dist_ < 2 * planning_tree_resolution_) {
    RCLCPP_WARN(node_->get_logger(),
        "Cannot set %.2f as goal reached dist for planning tree resolution %.2f. Setting goal reached distance to %.2f to prevent deadlocks.",
        _goal_reached_dist_, planning_tree_resolution_, 2 * planning_tree_resolution_);
    _goal_reached_dist_ = 2 * planning_tree_resolution_;
  }

  _euclidean_distance_cutoff_ = _safe_obstacle_distance_ + 0.01;  // adaptation to prevent UAV being stuck

  // set planner deadlock timeout
  if (_restart_planner_on_deadlock_) {

    if (_planner_deadlock_timeout_factor < 3.0) {
      RCLCPP_WARN(node_->get_logger(),
          "[MrsOctomapPlanner]: Timeout factor for planner deadlock detection was set too low (< 3.0). Setting factor to 3.0 to prevent premature killing of "
          "the planner.");
      _planner_deadlock_timeout_factor = 3.0;
    }

    planner_deadlock_timeout_ = _planner_deadlock_timeout_factor * _timeout_threshold_;
    RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Planner deadlock timeout set to %.2f s.", planner_deadlock_timeout_);
  }

  octree_ = nullptr;

  // | ---------------------- state machine --------------------- |

  state_ = STATE_IDLE;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandlerOptions phopts;
  phopts.node = node_;  

  
  pub_diagnostics_        = mrs_lib::PublisherHandler<mrs_modules_msgs::msg::OctomapPlannerDiagnostics>(phopts, "~/diagnostics_out");
  pub_virtual_obstacles_  = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(phopts, "~/virtual_obstacles_out");

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                 = node_;
  shopts.node_name            = "Pathfinder";
  shopts.no_message_timeout   = mrs_lib::no_timeout;
  shopts.threadsafe           = true;
  shopts.autostart            = true;



  sh_tracker_cmd_           = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "~/tracker_cmd_in");
  sh_octomap_               = mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>(shopts, "~/octomap_in");
  sh_control_manager_diag_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diag_in");
  sh_constraints_           = mrs_lib::SubscriberHandler<mrs_msgs::msg::DynamicsConstraints>(shopts, "~/constraints_in");

  // | --------------------- service clients -------------------- |

  sc_get_trajectory_       = mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv>(node_, "~/trajectory_generation_out");
  sc_trajectory_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::TrajectoryReferenceSrv>(node_, "~/trajectory_reference_out");
  sc_hover_                = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/hover_out");

  // | ----------------------- transformer ---------------------- |

  transformer_ = std::make_unique<mrs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  // | -------------------- batch visualiuzer ------------------- |

  bv_input_ = mrs_lib::BatchVisualizer(node_, "visualize_input", "");
  bv_input_.setPointsScale(_points_scale_);
  bv_input_.setLinesScale(_lines_scale_);

  bv_planner_ = std::make_shared<mrs_lib::BatchVisualizer>(node_, "visualize_planner", "");
  bv_planner_->setPointsScale(_points_scale_);
  bv_planner_->setLinesScale(_lines_scale_);

  bv_processed_ = mrs_lib::BatchVisualizer(node_, "visualize_processed", "");
  bv_processed_.setPointsScale(_points_scale_);
  bv_processed_.setLinesScale(_lines_scale_);

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.callback_group = cbkgrp_timers_;
  timer_opts_start.autostart      = true;

  
  callback_timer_main_ = std::bind(&OctomapPlanner::timerMain, this);
  timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_rate_main_timer_, clock_), callback_timer_main_);
  timer_main_->start();

  callback_timer_future_check_ = std::bind(&OctomapPlanner::timerFutureCheck, this);
  timer_future_check_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_rate_future_check_timer_, clock_), callback_timer_future_check_);
  timer_future_check_->start(); 

  callback_timer_diagnostics_ = std::bind(&OctomapPlanner::timerDiagnostics, this);
  timer_diagnostics_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_rate_diagnostics_timer_, clock_), callback_timer_diagnostics_);
  timer_diagnostics_->start();

  callback_timer_publish_virtual_obstacles_ = std::bind(&OctomapPlanner::timerPublishVirtualObstacles, this);
  timer_publish_virtual_obstacles_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_rate_virtual_obstacle_pub_timer_, clock_), callback_timer_publish_virtual_obstacles_);
  timer_publish_virtual_obstacles_->start();

  // | --------------------- service servers -------------------- |


  service_server_goto_ = node_->create_service<mrs_msgs::srv::Vec4>(
            "~/goto_in",
            std::bind(&OctomapPlanner::callbackGoto, this,
                      std::placeholders::_1, std::placeholders::_2));
  service_server_stop_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/stop_in",
            std::bind(&OctomapPlanner::callbackStop, this,
                      std::placeholders::_1, std::placeholders::_2));
  service_server_reference_ = node_->create_service<mrs_msgs::srv::ReferenceStampedSrv>(
            "~/reference_in",
            std::bind(&OctomapPlanner::callbackReference, this,
                      std::placeholders::_1, std::placeholders::_2));
  service_server_set_planner_ = node_->create_service<mrs_msgs::srv::String>(
            "~/planner_type_in",
            std::bind(&OctomapPlanner::callbackSetPlanner, this,
                      std::placeholders::_1, std::placeholders::_2));
  service_server_set_safety_distance_ = node_->create_service<mrs_msgs::srv::Vec1>(
            "~/set_safety_distance_in",
            std::bind(&OctomapPlanner::callbackSetSafetyDistance, this,
                      std::placeholders::_1, std::placeholders::_2));
  service_server_set_max_altitude_ = node_->create_service<mrs_msgs::srv::Vec1>(
            "~/set_max_altitude_in",
            std::bind(&OctomapPlanner::callbackSetMaxAltitude, this,
                      std::placeholders::_1, std::placeholders::_2));
  service_server_add_virtual_obstacle_ = node_->create_service<mrs_msgs::srv::ValidateReferenceArray>(
            "~/add_virtual_obstacle_in",
            std::bind(&OctomapPlanner::callbackAddVirtualObstacle, this,
                      std::placeholders::_1, std::placeholders::_2));
  service_server_remove_virtual_obstacles_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/remove_virtual_obstacles_in",
            std::bind(&OctomapPlanner::callbackRemoveVirtualObstacles, this,
                      std::placeholders::_1, std::placeholders::_2));

  // | --------------------- finish the init -------------------- |


  /* scope timer logger //{ */

  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<mrs_lib::ScopeTimerLogger>(node_, scope_timer_log_filename, _scope_timer_enabled_);

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(),"initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* timeoutTrackerCmd() //{ */

void OctomapPlanner::timeoutTrackerCmd(const std::string& topic, const rclcpp::Time& last_msg) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_tracker_cmd_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: position cmd timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* callbackOctomap() //{ */

void OctomapPlanner::callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsOctomapPlanner]: getting octomap");

  std::optional<OcTreePtr_t> octree_local = msgToMap(msg);

  if (!octree_local) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: received map is empty!");
    return;
  }

  {
    std::scoped_lock lock(mutex_octree_);

    /* copyLocalMap(*octree_local, octree_global_); */

    octree_ = octree_local.value();
    {
      std::scoped_lock lock(mutex_virtual_obstacles_);
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
void OctomapPlanner::timeoutOctomap(const std::string& topic, const rclcpp::Time& last_msg) {
  

  if (!is_initialized_) {
    return;
  }

  if (!sh_octomap_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: octomap timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* timeoutControlManagerDiag() //{ */

void OctomapPlanner::timeoutControlManagerDiag(const std::string& topic, const rclcpp::Time& last_msg) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: Control manager diag timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* callbackStop() //{ */
void OctomapPlanner::callbackStop(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  if (!is_initialized_) {
    return;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 500, "[MrsOctomapPlanner]: " << ss.str());

    res->success = false;
    res->message = ss.str();
    return;
  }
  changeState(STATE_IDLE);
  hover();

  std::stringstream ss;
  ss << "Stopping by request";

  RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 500, "[MrsOctomapPlanner]: " << ss.str());
  res->success = true;
  res->message = ss.str();
  return;
}

//}

/* callbackGoto() //{ */
  void OctomapPlanner::callbackGoto(const std::shared_ptr<mrs_msgs::srv::Vec4::Request> req,std::shared_ptr<mrs_msgs::srv::Vec4::Response> res) {

  /* prerequisities //{ */

  if (!is_initialized_) {
    return;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 500, "[MrsOctomapPlanner]: " << ss.str());

    res->success = false;
    res->message = ss.str();
    return;
  }

  //}

  // | -------- transform the reference to the map frame -------- |

  {
    mrs_msgs::msg::TrackerCommand::ConstSharedPtr tracker_cmd = sh_tracker_cmd_.getMsg();

    mrs_msgs::msg::ReferenceStamped reference;
    reference.header.frame_id = tracker_cmd->header.frame_id;

    reference.reference.position.x = req->goal[0];
    reference.reference.position.y = req->goal[1];
    reference.reference.position.z = req->goal[2];
    reference.reference.heading    = req->goal[3];

    auto result = transformer_->transformSingle(reference, octree_frame_);

    if (result) {

      std::scoped_lock lock(mutex_user_goal_);

      user_goal_              = result.value().reference;
      new_user_goal_received_ = true;

    } else {
      std::stringstream ss;
      ss << "could not transform the reference from " << tracker_cmd->header.frame_id << " to " << octree_frame_;

      RCLCPP_ERROR_STREAM(node_->get_logger(),"[MrsOctomapPlanner]: " << ss.str());

      res->success = false;
      res->message = ss.str();
      return;
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

  res->success = true;
  res->message = "goal set";
  return;
}

//}

/* callbackReference() //{ */

void OctomapPlanner::callbackReference(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> req, std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> res) {

  /* prerequisities //{ */

  if (!is_initialized_) {
    return;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 500, "[MrsOctomapPlanner]: " << ss.str());

    res->success = false;
    res->message = ss.str();
    return;
  }

  //}

  // | -------- transform the reference to the map frame -------- |
  {

    mrs_msgs::msg::ReferenceStamped ref_stamped;
    ref_stamped.header    = req->header;
    ref_stamped.reference = req->reference;

    auto result = transformer_->transformSingle(ref_stamped, octree_frame_);

    if (result) {

      std::scoped_lock lock(mutex_user_goal_);

      user_goal_              = result.value().reference;
      new_user_goal_received_ = true;

    } else {
      std::stringstream ss;
      ss << "could not transform the reference from " << ref_stamped.header.frame_id << " to " << octree_frame_;

      RCLCPP_ERROR_STREAM(node_->get_logger(),"[MrsOctomapPlanner]: " << ss.str());

      res->success = false;
      res->message = ss.str();
      return;
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

  res->success = true;
  res->message = "reference set";
  return;
}

//}

/* callbackSetPlanner() //{ */

void OctomapPlanner::callbackSetPlanner(const std::shared_ptr<mrs_msgs::srv::String::Request> req, std::shared_ptr<mrs_msgs::srv::String::Response> res) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Setting planner to %s requested.", req->value.c_str());
  res->success = true;

  if (req->value == "mrs") {
    _use_subt_planner_ = false;
  } else if (req->value == "subt") {
    _use_subt_planner_ = true;
  } else {
    res->success = false;
  }

  res->message = res->success ? "Planner set successfully." : "Invalid type of planner requested.";
  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: %s", res->message.c_str());
  return;
}

//}

/* callbackSetSafetyDistance() //{ */

void OctomapPlanner::callbackSetSafetyDistance(const std::shared_ptr<mrs_msgs::srv::Vec1::Request> req, std::shared_ptr<mrs_msgs::srv::Vec1::Response> res) {

  if (!is_initialized_) {
    return;
  }

  if (req->goal >= _safe_obstacle_distance_min_ && req->goal <= _safe_obstacle_distance_max_) {

    {
      std::scoped_lock lock(mutex_safety_distance_);

      _safe_obstacle_distance_    = req->goal;
      _euclidean_distance_cutoff_ = _safe_obstacle_distance_ + 0.01;  // needed for correct function of MRS planner
    }

    RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: setting safety distance to %.2f m.", _safe_obstacle_distance_);
    res->success = true;

  } else {

    RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: failed to set safety distance %.2f m (outside the allowed range [%.2f, %.2f])", req->goal, _safe_obstacle_distance_min_,
             _safe_obstacle_distance_max_);
    res->success = false;
  }

  res->message = res->success ? "safety distance set" : "safety distance not set";

  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: %s", res->message.c_str());

  return;
}

//}

/* callbackSetMaxAltitude() //{ */

void OctomapPlanner::callbackSetMaxAltitude(const std::shared_ptr<mrs_msgs::srv::Vec1::Request> req, std::shared_ptr<mrs_msgs::srv::Vec1::Response> res) {

  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_max_altitude_);

    _max_altitude_ = req->goal;
  }

  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: setting max altitude to %.2f m.", _max_altitude_);
  res->success = true;

  res->message = "max altitude set";

  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: %s", res->message.c_str());

  return;
}

//}
//
/* callbackAddVirtualObstacle() //{ */

void OctomapPlanner::callbackAddVirtualObstacle(const std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Request> req, std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Response> res) {

  if (!is_initialized_) {
    res->success = {false};
    res->message = "not yet initialized -> cannot insert virtual obstacle";
    return;
  }

  if (req->array.array.size() != 3) {
    res->success = {false};
    res->message = "array does not contain exactly 3 points -> cannot insert virtual obstacle";
    return;
  }

  // Transform points to octree frame
  std::string                      octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
  std::vector<mrs_msgs::msg::Reference> virt_obst_in_octree_frame;

  for (auto& point_ref : req->array.array) {

    mrs_msgs::msg::ReferenceStamped ref_stamped;
    ref_stamped.header    = req->array.header;
    ref_stamped.reference = point_ref;

    RCLCPP_INFO(node_->get_logger(),"Input obstacle point = [%.2f, %.2f, %.2f]", point_ref.position.x, point_ref.position.y, point_ref.position.z);

    auto res_t = transformer_->transformSingle(ref_stamped, octree_frame);

    if (!res_t) {
      res->success = {false};
      res->message = "could not transform virtual obstacle to the map frame";
      RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: %s", res->message.c_str());
      return;
    }

    virt_obst_in_octree_frame.push_back(res_t.value().reference);
    RCLCPP_INFO(node_->get_logger(),"Transformed obstacle point = [%.2f, %.2f, %.2f]", virt_obst_in_octree_frame.back().position.x,
             virt_obst_in_octree_frame.back().position.y, virt_obst_in_octree_frame.back().position.z);
  }

  // Define the 12 edges by point pairs
  const std::vector<std::pair<int, int>> edge_indices = {
      {0, 1}, {0, 2}, {1, 3}, {2, 3},  // bottom face
      {4, 5}, {5, 7}, {7, 6}, {6, 4},  // top face
      {0, 4}, {1, 5}, {2, 6}, {3, 7}   // vertical edges
  };

  VirtualObstacle_t obst;
  obst.frame_id = octree_frame;
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
  const auto& pts   = virt_obst_in_octree_frame;
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
  visualization_msgs::msg::Marker edges;
  auto&                      marker = obst.vis_marker;
  marker.header.frame_id            = octree_frame;
  marker.header.stamp               = clock_->now();
  marker.ns                         = "edges";
  marker.id                         = 0;
  marker.type                       = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action                     = visualization_msgs::msg::Marker::ADD;
  marker.scale.x                    = 0.04;  // line width

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.points.reserve(edge_indices.size());
  for (const auto& e : edge_indices) {
    geometry_msgs::msg::Point p_start;
    geometry_msgs::msg::Point p_end;

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
    std::scoped_lock lock(mutex_virtual_obstacles_);

    virtual_obstacles_.push_back(obst);
  }

  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner] adding virtual obstacle");
  RCLCPP_INFO(node_->get_logger(),"                        p0: %.1f %.1f %.1f", p0.x(), p0.y(), p0.z());
  RCLCPP_INFO(node_->get_logger(),"                        p1: %.1f %.1f %.1f", p1.x(), p1.y(), p1.z());
  RCLCPP_INFO(node_->get_logger(),"                        p2: %.1f %.1f %.1f", p2.x(), p2.y(), p2.z());

  res->success = {true};
  res->message = "obstacle added";

  return;
}

//}

/* callbackRemoveVirtualObstacles() //{ */

void OctomapPlanner::callbackRemoveVirtualObstacles(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  if (!is_initialized_) {
    return;
  }

  size_t virt_obst_size = virtual_obstacles_.size();

  std::stringstream ss;

  if (virt_obst_size > 0) {

    std::scoped_lock lock(mutex_virtual_obstacles_);
    virtual_obstacles_.clear();
    ss << virt_obst_size << " obstacles removed from the virtual obstacles.";

  } else {

    ss << "Virtual obstacles array is empty. No obstacles to be removed.";
  }


  RCLCPP_INFO_STREAM(node_->get_logger(),"[MrsOctomapPlanner]: " << ss.str());
  res->success = true;
  res->message = ss.str();
  return;
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void OctomapPlanner::timerMain() {

  if (!is_initialized_) {
    return;
  }

  /* prerequsities //{ */

  const bool got_octomap              = sh_octomap_.hasMsg() && (clock_->now() - sh_octomap_.lastMsgTime()).seconds() < 2.0;
  const bool got_tracker_cmd          = sh_tracker_cmd_.hasMsg() && (clock_->now() - sh_tracker_cmd_.lastMsgTime()).seconds() < 2.0;
  const bool got_control_manager_diag = sh_control_manager_diag_.hasMsg() && (clock_->now() - sh_control_manager_diag_.lastMsgTime()).seconds() < 2.0;
  const bool got_constraints          = sh_constraints_.hasMsg() && (clock_->now() - sh_constraints_.lastMsgTime()).seconds() < 2.0;

  if (!got_octomap || !got_tracker_cmd || !got_control_manager_diag || !got_constraints) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: waiting for data: octomap = %s, position cmd = %s, ControlManager diag = %s, constraints = %s",
                      got_octomap ? "TRUE" : "FALSE", got_tracker_cmd ? "TRUE" : "FALSE", got_control_manager_diag ? "TRUE" : "FALSE",
                      got_constraints ? "TRUE" : "FALSE");
    return;
  } else {
    ready_to_plan_ = true;
  }

  //}

  RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsOctomapPlanner]: main timer spinning");

  const auto user_goal = mrs_lib::get_mutexed(mutex_user_goal_, user_goal_);
  if (new_user_goal_received_) {
    first_planning_for_current_goal_ = true;
    new_user_goal_received_          = false;
  }

  const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diag = sh_control_manager_diag_.getMsg();
  const mrs_msgs::msg::TrackerCommand::ConstSharedPtr            tracker_cmd          = sh_tracker_cmd_.getMsg();

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

    diagnostics_.header.stamp    = clock_->now();
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

      mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(node_, "timerMain - STATE_PLANNING", scope_timer_logger_, _scope_timer_enabled_);

      if (!octree->getRoot()) {

        RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: don't have a map");

        changeState(STATE_IDLE);

        break;
      }

      {
        std::scoped_lock lock(mutex_diagnostics_);

        diagnostics_.idle = false;
      }

      if (replanning_counter_ >= _max_attempts_to_replan_) {

        RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: planning failed, the uav is stuck");

        changeState(STATE_IDLE);

        break;
      }

      /* get the initial condition (predicted pose of UAV at a specific time) */ /*//{*/
      double time_for_planning_s = 0.0;
      if (control_manager_diag->tracker_status.have_goal) {
        time_for_planning_s = _timeout_threshold_;
      } else {
        time_for_planning_s = _timeout_threshold_ + pow(1.5, float(replanning_counter_));
      }

      RCLCPP_INFO(node_->get_logger(), "[MrsOctomapPlanner]: planning timeout %.2f s", time_for_planning_s);
      
      /*
      rclcpp::Time time_for_planning;

      if (control_manager_diag->tracker_status.have_goal) {
        time_for_planning = rclcpp::Time(_timeout_threshold_);
      } else {
        time_for_planning = _timeout_threshold_ + pow(1.5, float(replanning_counter_));
      }

      RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: planning timeout %.2f s", time_for_planning);
      */
      
      rclcpp::Time init_cond_time = clock_->now() + rclcpp::Duration::from_seconds(time_for_planning_s + _time_for_trajectory_generator_);

      RCLCPP_INFO(node_->get_logger(), "[MrsOctomapPlanner]: init cond time %.2f s", init_cond_time.seconds());

      timer.checkpoint("before getInitialCondition");

      int  iter              = 0;
      auto initial_condition = getInitialCondition(init_cond_time);

      while (!initial_condition && iter < 20) {

        initial_condition = getInitialCondition(init_cond_time);
        RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: Trying to get initial condition updated with the last sent path.");
        rclcpp::sleep_for(std::chrono::milliseconds(5));
        iter++;
      }

      timer.checkpoint("after getInitialCondition()");

      if (!initial_condition) {

        RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: could not obtain initial condition for planning");
        hover();
        changeState(STATE_IDLE);

        break;
      }

      //RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: init cond time stamp %.2f", (initial_condition.value().header.stamp.seconds());

      octomap::point3d plan_from;
      plan_from.x() = initial_condition.value().reference.position.x;
      plan_from.y() = initial_condition.value().reference.position.y;
      plan_from.z() = initial_condition.value().reference.position.z;
      /*//}*/

      /* check if goal was reached */ /*//{*/
      if ((plan_from - user_goal_octpoint).norm() <= _min_path_length_ &&
          mrs_lib::geometry::radians::dist(initial_condition.value().reference.heading, user_goal_.heading) < _min_path_heading_change_) {

        RCLCPP_INFO_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: we reached the target");
        changeState(STATE_IDLE);
        break;
      }
      /*//}*/

      /* plan the path to goal */ /*//{*/
      std::pair<std::vector<octomap::point3d>, bool> waypoints;
      /* rclcpp::Time                                      mct_start = clock_->now(); */

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
          RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,100, "[MrsOctomapPlanner]: collision check of short direct path failed, could not raytrace!");
          is_direct_path_collision_free = false;
          break;
        }

        if (is_direct_path_collision_free) {
          RCLCPP_INFO(node_->get_logger(),"Using direct_collision_free_path.");
          waypoints.second = true;
          waypoints.first.push_back(plan_from);

        } else {

          RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: direct short path does not exist.");
          changeState(STATE_IDLE);
          break;
        }

      } else {

        if (_use_subt_planner_) {

          // | -------------------- MRS SubT planner -------------------- |
          mrs_subt_planning::AstarPlanner subt_planner = mrs_subt_planning::AstarPlanner(node_, "MrsOctomapPlanner SubT Astar Planner");  //to change

          subt_planner.initialize(true, time_for_planning_s - _subt_processing_timeout_, _subt_processing_timeout_, safe_obstacle_distance, _subt_clearing_dist_,
                                  _min_altitude_, max_altitude, _subt_debug_info_, bv_planner_, false);
          subt_planner.setAstarAdmissibility(_subt_admissibility_);

          RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Calling find path method.");

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = clock_->now();
          }

          waypoints = subt_planner.findPath(plan_from, user_goal_octpoint, octree, _subt_make_path_straight_, _subt_apply_postprocessing_,
                                            _subt_bbx_horizontal_, _subt_bbx_vertical_, _subt_processing_safe_dist_, _subt_processing_max_iterations_,
                                            _subt_processing_horizontal_neighbors_only_, _subt_processing_z_diff_tolerance_, _subt_processing_path_length_,
                                            _subt_shortening_window_size_, _subt_shortening_distance_, _subt_apply_pruning_, _subt_pruning_dist_, false, 2.0,
                                            _subt_remove_obsolete_points_, _subt_obsolete_points_tolerance_);

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = rclcpp::Time(0);
          }

        } else {


          mrs_octomap_planner::AstarPlanner planner = mrs_octomap_planner::AstarPlanner(node_, "MrsOctomapPlanner MRS Astar Planner",
              safe_obstacle_distance, _euclidean_distance_cutoff_, _distance_transform_distance_, planning_tree_resolution_, _distance_penalty_,
              _greedy_penalty_, _timeout_threshold_, _max_waypoint_distance_, _min_altitude_, max_altitude, _unknown_is_occupied_, bv_planner_);

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = clock_->now();
          }

          waypoints = planner.findPath(plan_from, user_goal_octpoint, octree, time_for_planning_s);

          {
            std::scoped_lock lock(mutex_planner_time_flag_);

            planner_time_flag_ = rclcpp::Time(0);
          }
        }
      }

      timer.checkpoint("after findPath()");

      // path is complete
      if (waypoints.second) {

        replanning_counter_ = 0;

        waypoints.first.push_back(user_goal_octpoint);

        RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Path is complete. Path length = %lu", waypoints.first.size());

      } else {

        RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Path is not complete");
        if (waypoints.first.size() < 2) {

          RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: path not found");

          replanning_counter_++;

          break;
        }

        RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Path is not complete but found");

        double front_x = waypoints.first.front().x();
        double front_y = waypoints.first.front().y();
        double front_z = waypoints.first.front().z();

        double back_x = waypoints.first.back().x();
        double back_y = waypoints.first.back().y();
        double back_z = waypoints.first.back().z();

        double path_start_end_dist = sqrt(pow(front_x - back_x, 2) + pow(front_y - back_y, 2) + pow(front_z - back_z, 2));

        if (path_start_end_dist < _min_path_length_) {

          RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: path too short, length: %.3f", path_start_end_dist);

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

            RCLCPP_INFO_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: Insufficient improvement in dist to goal. Aborting the planning to avoid oscillations.");

            replanning_counter_++;

            avoiding_oscillations_ = true;

            changeState(STATE_PLANNING);

            break;
          }
        }
      }

      time_last_plan_                  = clock_->now();
      first_planning_for_current_goal_ = false;
      detected_collision_              = false;
      avoiding_oscillations_           = false;

      {
        std::scoped_lock lock(mutex_diagnostics_);

        diagnostics_.best_goal.x = waypoints.first.back().x();
        diagnostics_.best_goal.y = waypoints.first.back().y();
        diagnostics_.best_goal.z = waypoints.first.back().z();
      }

      /*//}*/

      {
        std::scoped_lock lock(mutex_initial_condition_);

        mrs_msgs::msg::TrackerCommand::ConstSharedPtr tracker_cmd = sh_tracker_cmd_.getMsg();

        auto octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);

        // transform the position cmd to the map frame
        mrs_msgs::msg::ReferenceStamped position_cmd_ref;
        position_cmd_ref.header               = tracker_cmd->header;
        position_cmd_ref.reference.position.x = tracker_cmd->position.x;
        position_cmd_ref.reference.position.y = tracker_cmd->position.y;
        position_cmd_ref.reference.position.z = tracker_cmd->position.z;
        position_cmd_ref.reference.heading    = tracker_cmd->heading;

        auto res = transformer_->transformSingle(position_cmd_ref, octree_frame);

        if (!res) {
          RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: could not transform position cmd to the map frame");
          return;
        }

        initial_pos_.x() = res.value().reference.position.x;
        initial_pos_.y() = res.value().reference.position.y;
        initial_pos_.z() = res.value().reference.position.z;
        initial_heading_ = res.value().reference.heading;
      }

      rclcpp::Time path_stamp = initial_condition.value().header.stamp;

      if (clock_->now() > path_stamp || !control_manager_diag->tracker_status.have_goal) {
        path_stamp = rclcpp::Time(0);
      }

      RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Calling path service with timestamp = %.3f at time %.3f.", path_stamp.seconds(), clock_->now().seconds());
      rclcpp::Time tg_start = clock_->now();

      mrs_msgs::srv::GetPathSrv srv_get_path;

      /*srv_get_path.path.header.frame_id = octree_frame_;
      srv_get_path.path.header.stamp    = path_stamp;
      srv_get_path.path.fly_now         = false;
      srv_get_path.path.relax_heading   = _trajectory_generation_relax_heading_;
      srv_get_path.path.use_heading     = _trajectory_generation_use_heading_;
      */

      std::shared_ptr<mrs_msgs::srv::GetPathSrv::Request> req_path = std::make_shared<mrs_msgs::srv::GetPathSrv::Request>(srv_get_path);
      req_path->path.header.frame_id = octree_frame_;
      req_path->path.header.stamp    = path_stamp;
      req_path->path.fly_now         = false;  
      req_path->path.relax_heading   = _trajectory_generation_relax_heading_;
      req_path->path.use_heading     = _trajectory_generation_use_heading_;
      
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

        mrs_msgs::msg::Reference ref;
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

              if (end_idx < waypoints.first.size() - 1) {
                end_idx++;
              } else {
                break;
              }
            }

            if (fabs(dx) > 1e-3 || fabs(dy) > 1e-3) {
              ref.heading = atan2(dy, dx) + _heading_offset_;
            } else {
              if (i > 0) {
                ref.heading = req_path->path.points.back().heading;
              } else {
                ref.heading = initial_heading_;
              }
            }
          } else {
            if (waypoints.first.size() > 1) {
              ref.heading = req_path->path.points.back().heading;
            } else {
              ref.heading = initial_heading_;
            }
          }
        } else {

          ref.heading = user_goal.heading;
        }
        req_path->path.points.push_back(ref);

        if (i > 0) {
          double waypoint_dist = (waypoints.first[i] - waypoints.first[i - 1]).norm();
          if (waypoint_dist > _max_segment_length_for_heading_sampling_) {
            mrs_msgs::msg::Reference inter_ref;
            inter_ref.position.x =
                waypoints.first[i - 1].x() + (waypoints.first[i].x() - waypoints.first[i - 1].x()) / waypoint_dist * _max_segment_length_for_heading_sampling_;
            inter_ref.position.y =
                waypoints.first[i - 1].y() + (waypoints.first[i].y() - waypoints.first[i - 1].y()) / waypoint_dist * _max_segment_length_for_heading_sampling_;
            inter_ref.position.z =
                waypoints.first[i - 1].z() + (waypoints.first[i].z() - waypoints.first[i - 1].z()) / waypoint_dist * _max_segment_length_for_heading_sampling_;
            inter_ref.heading = ref.heading;
            req_path->path.points.push_back(inter_ref);
            /* RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: TG inter input point %02d: [%.2f, %.2f, %.2f, %.2f]", i, inter_ref.position.x, inter_ref.position.y, */
            /* inter_ref.position.z, inter_ref.heading); */
          }
        }

        RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: TG input point %02d: [%.2f, %.2f, %.2f, %.2f]", i, ref.position.x, ref.position.y, ref.position.z, ref.heading);
        req_path->path.points.push_back(ref);

        cum_time += segment_times[i];

        if (i > 1 && cum_time > _trajectory_generation_input_length_) {
          RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: cutting path in waypoint %d out of %d", i, int(waypoints.first.size()));
          break;
        }
      }

      if (!req_path->path.points.empty()) {
          req_path->path.points.back().heading = user_goal.heading;
    }

      if (interrupted_) {
        RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: planner interrupted, breaking main timer");
        break;
      }

      RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: calling trajectory generation");

      timer.checkpoint("calling trajectory generation");

        req_path->path.points.back().position.x = waypoints.first.back().x();
        
        auto res_path = sc_get_trajectory_.callSync(req_path);

        if (!res_path) {
          RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: service call for trajectory failed");
          break;
        } else {
          if (!res_path.value()->success) {
            RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: service call for trajectory failed: '%s'", res_path.value()->message.c_str());
            break;
          }
        }
    

      RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Trajectory generation took %.2f s", (clock_->now() - tg_start).seconds());

      {
        std::scoped_lock lock(mutex_bv_processed_);

        bv_processed_.clearBuffers();
        for (auto& p : res_path.value()->trajectory.points) {
          auto v = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
          bv_processed_.addPoint(v, 0, 1, 0, 1);
        }
        bv_processed_.publish();
      }

      auto trajectory = res_path.value()->trajectory;

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
              RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,100, "[MrsOctomapPlanner]: trajectory check found collision with prediction horizon between %d and %d, replanning!", i, i + 1);
              ray_is_cool = false;
              break;
            }
          }
        } else {
          RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,100, "[MrsOctomapPlanner]: trajectory check failed, could not raytrace!");
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

      setReplanningPoint(trajectory);
      set_timepoints_ = true;

      // set user goal heading for final point such that it is achieved independently on length of the trajectory
      if (_use_user_heading_for_final_point_ && !res_path.value()->trajectory.points.empty()) {
        res_path.value()->trajectory.points.push_back(res_path.value()->trajectory.points.back());
        res_path.value()->trajectory.points.back().heading = user_goal_.heading;
      }

      RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: publishing trajectory reference");

      mrs_msgs::srv::TrajectoryReferenceSrv srv_trajectory_reference;
      std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request> req_traj = std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Request>(srv_trajectory_reference);
      req_traj->trajectory         = res_path.value()->trajectory;
      req_traj->trajectory.fly_now = true;

      // set id of trajectory
      path_id_++;
      req_traj->trajectory.input_id = path_id_;

      int cb = 0;

      RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: Calling trajectory service with timestamp = %.3f at time %.3f.",
               req_traj->trajectory.header.stamp, clock_->now().seconds());

      
        auto res_traj = sc_trajectory_reference_.callSync(req_traj);

      if (!res_traj) {
        RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: service call for trajectory reference failed");
        break;
      } else {
        if (!res_traj.value()->success) {
          RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: service call for trajectory reference failed: '%s'", res_traj.value()->message.c_str());
          break;
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
      mrs_msgs::msg::TrackerCommand::ConstSharedPtr tracker_cmd  = sh_tracker_cmd_.getMsg();
      auto                             octree_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);

      // transform the position cmd to the map frame
      mrs_msgs::msg::ReferenceStamped position_cmd_ref;
      position_cmd_ref.header               = tracker_cmd->header;
      position_cmd_ref.reference.position.x = tracker_cmd->position.x;
      position_cmd_ref.reference.position.y = tracker_cmd->position.y;
      position_cmd_ref.reference.position.z = tracker_cmd->position.z;
      position_cmd_ref.reference.heading    = tracker_cmd->heading;

      auto res = transformer_->transformSingle(position_cmd_ref, octree_frame);

      if (!res) {
        RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: could not transform position cmd to the map frame");
        return;
      }

      octomap::point3d position_cmd_octomap;
      position_cmd_octomap.x() = res.value().reference.position.x;
      position_cmd_octomap.y() = res.value().reference.position.y;
      position_cmd_octomap.z() = res.value().reference.position.z;

      double dist_to_goal = (position_cmd_octomap - user_goal_octpoint).norm();

      RCLCPP_INFO_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: dist to goal: %.2f m", dist_to_goal);

      if (dist_to_goal < _goal_reached_dist_) {
        RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: user goal reached");
        changeState(STATE_IDLE);
        break;
      }

      if ((clock_->now().seconds() - (time_last_plan_.seconds() + _replan_after_)) > 0,0) {


        RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: triggering replanning");

        changeState(STATE_PLANNING);
      }

      break;
    }

      //}
  }
}

//}

/* timerFutureCheck() //{ */

void OctomapPlanner::timerFutureCheck() {

  if (!is_initialized_) {
    return;
  }

  /* preconditions //{ */

  if (!sh_control_manager_diag_.hasMsg()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(),*clock_,1000, "Timer future: missing control manager");
    return;
  }

  if (!sh_octomap_.hasMsg()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(),*clock_,1000, "Timer future: missing octomap");
    return;
  }

  //}

  if (state_ == STATE_IDLE && !avoiding_oscillations_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(),"[MrsOctomapPlanner]: future check timer spinning");
  const mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer(node_, "timerFutureCheck", scope_timer_logger_, _scope_timer_enabled_);

  std::shared_ptr<OcTree_t> octree;

  {
    std::scoped_lock lock(mutex_octree_);

    octree = std::make_shared<OcTree_t>(*octree_);
  }

  // | ----------- check if the prediction is feasible ---------- |

  if (!octree->getRoot()) {
    RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: cannot check for collision, don't have a map");
    return;
  }

  mrs_msgs::msg::MpcPredictionFullState            prediction           = sh_tracker_cmd_.getMsg()->full_state_prediction;
  mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  if (control_manager_diag->flying_normally && control_manager_diag->tracker_status.have_goal) {

    geometry_msgs::msg::TransformStamped tf;

    auto ret = transformer_->getTransform(prediction.header.frame_id, octree_frame_, prediction.header.stamp);

    if (!ret) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: could not transform position cmd to the map frame! can not check for potential collisions!");
      return;
    }

    tf = ret.value();

    // prepare the potential future trajectory

    mrs_msgs::msg::TrajectoryReference trajectory;
    trajectory.header.stamp    = ret.value().header.stamp;
    trajectory.header.frame_id = transformer_->frame_to(ret.value());
    trajectory.fly_now         = true;
    trajectory.use_heading     = _trajectory_generation_use_heading_;
    trajectory.dt              = 0.2;

    for (int i = 1; i < prediction.position.size(); i++) {

      mrs_msgs::msg::ReferenceStamped pose;
      pose.header               = prediction.header;
      pose.reference.position.x = prediction.position[i].x;
      pose.reference.position.y = prediction.position[i].y;
      pose.reference.position.z = prediction.position[i].z;
      pose.reference.heading    = prediction.heading[i];

      auto transformed_pose = transformer_->transform(pose, tf);

      if (!transformed_pose) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: could not transform position cmd to the map frame! can not check for potential collisions!");
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
              /* RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: Detected UNKNOWN space along the planned trajectory!"); */
            } else if (octree->isNodeOccupied(node)) {
              /* RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: Detected OCCUPIED space along the planned trajectory!"); */
              // shorten the trajectory
              int orig_traj_size = int(trajectory.points.size());
              for (int j = int(trajectory.points.size()) - 1; j >= i - 1 && j > _min_allowed_trajectory_points_after_crop_; j--) {
                trajectory.points.pop_back();
              }

              RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: Detected OCCUPIED space along the planned trajectory! Cropped the trajectory to %d from %d points.",
                       int(trajectory.points.size()), orig_traj_size);

              mrs_msgs::srv::TrajectoryReferenceSrv srv_trajectory_reference;
              std::shared_ptr<mrs_msgs::srv::TrajectoryReferenceSrv::Request> req_traj_ref =
                  std::make_shared<mrs_msgs::srv::TrajectoryReferenceSrv::Request>(srv_trajectory_reference);
              req_traj_ref->trajectory = trajectory;

              auto res_traj_ref = sc_trajectory_reference_.callSync(req_traj_ref);

              cropped_trajectory = true;

              if (!res_traj_ref) {
                RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: service call for trajectory reference failed");
                break;
              } else {
                if (!res_traj_ref.value()->success) {
                  RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: service call for trajectory reference failed: '%s'", res_traj_ref.value()->message.c_str());
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
          RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: Unable to raycast.");
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

          RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,100, "[MrsOctomapPlanner]: future check found collision with prediction horizon between %d and %d, hovering!", i, i + 1);

          // the trajectory directly passes through an obstacle, trigger hovering
          changeState(STATE_IDLE);
          hover();

          break;
        }

      } else {

        RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,100, "[MrsOctomapPlanner]: future check failed, could not raytrace!");
        hover();
        break;
      }
    }
  }
}

//}

/* timerDiagnostics() //{ */

void OctomapPlanner::timerDiagnostics() {

  if (!is_initialized_) {
    return;
  }

  auto diagnostics = mrs_lib::get_mutexed(mutex_diagnostics_, diagnostics_);

  try {
    pub_diagnostics_.publish(diagnostics);
  }
  catch (...) {
    //RCLCPP_ERROR(node_->get_logger(),"exception caught during publishing topic '%s'", diagnostics.c_str());
  }

  auto planner_time_flag = mrs_lib::get_mutexed(mutex_planner_time_flag_, planner_time_flag_);

  if (_restart_planner_on_deadlock_ && planner_time_flag != rclcpp::Time(0)) {
    if ((clock_->now() - planner_time_flag).seconds() > planner_deadlock_timeout_) {
      RCLCPP_ERROR(node_->get_logger(),"[MrsOctomapPlanner]: Planner is deadlocked, restarting!");
      rclcpp::shutdown();
    }
  }
}

//}

/* timerPublishVirtualObstacles() //{ */

void OctomapPlanner::timerPublishVirtualObstacles() {

  if (!is_initialized_) {
    return;
  }

  if (pub_virtual_obstacles_.getNumSubscribers() == 0) {
    return;
  }

  visualization_msgs::msg::MarkerArray ma;
  {
    std::scoped_lock lock(mutex_virtual_obstacles_);

    ma.markers.reserve(virtual_obstacles_.size());

    for (int i = 0; i < virtual_obstacles_.size(); i++) {
      auto& obst = virtual_obstacles_.at(i);

      obst.vis_marker.header.stamp = clock_->now();
      obst.vis_marker.id           = i;
      ma.markers.push_back(obst.vis_marker);
    }
  }

  try {
    pub_virtual_obstacles_.publish(ma);
  }
  catch (...) {
    //RCLCPP_ERROR(node_->get_logger(),"exception caught during publishing topic '%s'", ma);
  }
}

//}

// | ------------------------ routines ------------------------ |

/* setReplanningPoint() //{ */

void OctomapPlanner::setReplanningPoint(const mrs_msgs::msg::TrajectoryReference& traj) {

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
    RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: Planning interrupted, not changing state.");
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

  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

  state_ = new_state;
}

//}

/* getInitialCondition() //{ */

std::optional<mrs_msgs::msg::ReferenceStamped> OctomapPlanner::getInitialCondition(const rclcpp::Time des_time) {

  const mrs_msgs::msg::MpcPredictionFullState prediction_full_state = sh_tracker_cmd_.getMsg()->full_state_prediction;

  if (prediction_full_state.input_id != 0 && prediction_full_state.input_id < path_id_) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: could not obtain initial condition, the input_id (%lu) does not match id of last sent path (%d).",
                       prediction_full_state.input_id, path_id_);
    return {};
  }

  if (prediction_full_state.stamps.size() == 0) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: Could not obtain initial condition, prediction full state is empty.");
    return {};
  }

  if ((des_time - prediction_full_state.stamps.back()).seconds() > 0) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner]: could not obtain initial condition, the desired time is too far in the future");
    return {};
  }

  mrs_msgs::msg::ReferenceStamped orig_reference;
  orig_reference.header = prediction_full_state.header;

  rclcpp::Time future_time_stamp;

  for (int i = 0; i < prediction_full_state.stamps.size(); i++) {

    if ((prediction_full_state.stamps[i] - des_time).seconds() > 0) {
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

    mrs_msgs::msg::ReferenceStamped transformed_reference = result.value();
    transformed_reference.header.stamp               = future_time_stamp;

    return transformed_reference;

  } else {

    std::stringstream ss;
    ss << "could not transform initial condition to the map frame";

    RCLCPP_ERROR_STREAM(node_->get_logger(),"[MrsOctomapPlanner]: " << ss.str());
    return {};
  }
}

//}

/* hover() //{ */

void OctomapPlanner::hover(void) {

  RCLCPP_INFO(node_->get_logger(),"[MrsOctomapPlanner]: triggering hover, interrupting planner");

  interrupted_ = true;

  avoiding_oscillations_ = false;

  std::shared_ptr<std_srvs::srv::Trigger::Request> srv_out = std::make_shared<std_srvs::srv::Trigger::Request>();

  sc_hover_.callSync(srv_out);
}

//}

/* estimateSegmentTimes() //{ */

std::vector<double> OctomapPlanner::estimateSegmentTimes(const std::vector<Eigen::Vector4d>& vertices, const bool use_heading) {

  if (vertices.size() <= 1) {
    return std::vector<double>(0);
  }

  const mrs_msgs::msg::DynamicsConstraints::ConstSharedPtr constraints = sh_constraints_.getMsg();

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
  // for each vertex in the path
  for (size_t i = 0; i < vertices.size() - 1; i++) {

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
  return segment_times;
}

//}

/* msgToMap() //{ */

std::optional<OcTreePtr_t> OctomapPlanner::msgToMap(const octomap_msgs::msg::Octomap::ConstSharedPtr octomap) {

  octomap::AbstractOcTree* abstract_tree;

  if (octomap->binary) {
    abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);
  } else {
    abstract_tree = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!abstract_tree) {

    RCLCPP_WARN(node_->get_logger(),"[MrsOctomapPlanner]: octomap message is empty!");
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

    /* RCLCPP_INFO(node_->get_logger(),"pos: %d", pos); */
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

  for (const auto& obst : virtual_obstacles_) {

    if (obst.frame_id != octree_frame_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*clock_,1000, "[MrsOctomapPlanner] not adding virtual obstacle because it's frame_id (%s) doesn't match octree_frame (%s)",
                        obst.frame_id.c_str(), octree_frame_.c_str());
      continue;
    }

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

//}

}  // namespace mrs_octomap_planner


#include <rclcpp_components/register_node_macro.hpp>
  RCLCPP_COMPONENTS_REGISTER_NODE(mrs_octomap_planner::OctomapPlanner)