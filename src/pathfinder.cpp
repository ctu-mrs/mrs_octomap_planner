/* includes //{ */

#include <memory>
#include <ros/init.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTree.h>

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

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/PathfinderDiagnostics.h>

#include <std_srvs/Trigger.h>

#include <pathfinder/astar_planner.hpp>

//}

namespace pathfinder
{

/* defines //{ */

typedef enum
{
  STATE_IDLE,
  STATE_PLANNING,
  STATE_MOVING,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING"};

using OcTree_t            = octomap::OcTree;
using OcTreePtr_t         = std::shared_ptr<octomap::OcTree>;
using OcTreeMsgConstPtr_t = octomap_msgs::OctomapConstPtr;

//}

/* class Pathfinder //{ */

class Pathfinder : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool              is_initialized_ = false;
  std::atomic<bool> ready_to_plan_  = false;
  std::string       _uav_name_;

  // params
  double _euclidean_distance_cutoff_;
  double _safe_obstacle_distance_;
  double _distance_penalty_;
  double _greedy_penalty_;
  int    _planning_tree_fractor_;
  double _map_resolution_;
  double _timeout_threshold_;
  double _time_for_trajectory_generator_;
  double _max_waypoint_distance_;
  double _min_altitude_;
  double _max_altitude_;
  double _rate_main_timer_;
  double _rate_diagnostics_timer_;
  double _rate_future_check_timer_;
  double _replan_after_;
  bool   _unknown_is_occupied_;

  double planning_tree_resolution_;

  octomap_msgs::OctomapConstPtr octree_msg_ptr_;
  std::string                   octree_frame_;
  std::mutex                    mutex_octree_msg_ptr_;

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
  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>           sh_position_cmd_;
  mrs_lib::SubscribeHandler<octomap_msgs::Octomap>               sh_octomap_;
  mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>    sh_mpc_prediction_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>       sh_constraints_;

  // publishers
  ros::Publisher pub_diagnostics_;

  // subscriber callbacks
  void callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>& wrp);
  void callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp);
  void callbackMpcPrediction(mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>& wrp);

  // service servers
  ros::ServiceServer service_server_goto_;
  ros::ServiceServer service_server_reference_;

  // service server callbacks
  bool callbackGoto([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  bool callbackReference([[maybe_unused]] mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res);

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

  // diagnostics
  mrs_msgs::PathfinderDiagnostics diagnostics_;
  std::mutex                      mutex_diagnostics_;

  // timeouts
  void timeoutOctomap(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs);
  void timeoutPositionCmd(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs);
  void timeoutMpcPrediction(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs);
  void timeoutControlManagerDiag(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs);

  // transformer
  mrs_lib::Transformer transformer_;

  std::string current_control_frame_;
  std::mutex  mutex_current_control_frame_;

  // planning
  std::atomic<int> replanning_counter_ = 0;
  ros::Time        time_last_plan_;

  // state machine
  std::atomic<State_t> state_;
  void                 changeState(const State_t new_state);

  mrs_msgs::Reference user_goal_;
  std::mutex          mutex_user_goal_;

  octomap::point3d internal_goal_;

  std::atomic<bool> set_timepoints_ = false;

  ros::Time replanning_start_timepoint_;
  ros::Time replanning_end_timepoint_;

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

  void hover(void);
};

//}

/* onInit() //{ */

void Pathfinder::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[Pathfinder]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "Pathfinder");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("main_timer/rate", _rate_main_timer_);
  param_loader.loadParam("diagnostics_timer/rate", _rate_diagnostics_timer_);
  param_loader.loadParam("future_check_timer/rate", _rate_future_check_timer_);

  param_loader.loadParam("euclidean_distance_cutoff", _euclidean_distance_cutoff_);
  param_loader.loadParam("safe_obstacle_distance", _safe_obstacle_distance_);
  param_loader.loadParam("distance_penalty", _distance_penalty_);
  param_loader.loadParam("greedy_penalty", _greedy_penalty_);
  param_loader.loadParam("planning_tree_fractor", _planning_tree_fractor_);
  param_loader.loadParam("map_resolution", _map_resolution_);
  param_loader.loadParam("unknown_is_occupied", _unknown_is_occupied_);
  param_loader.loadParam("points_scale", _points_scale_);
  param_loader.loadParam("lines_scale", _lines_scale_);
  param_loader.loadParam("max_waypoint_distance", _max_waypoint_distance_);
  param_loader.loadParam("min_altitude", _min_altitude_);
  param_loader.loadParam("max_altitude", _max_altitude_);
  param_loader.loadParam("timeout_threshold", _timeout_threshold_);
  param_loader.loadParam("replan_after", _replan_after_);
  param_loader.loadParam("time_for_trajectory_generator", _time_for_trajectory_generator_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Pathfinder]: could not load all parameters");
    ros::shutdown();
  }

  planning_tree_resolution_ = _map_resolution_ * pow(2, _planning_tree_fractor_);

  // | ---------------------- state machine --------------------- |

  state_ = STATE_IDLE;

  // | ----------------------- publishers ----------------------- |

  pub_diagnostics_ = nh_.advertise<mrs_msgs::PathfinderDiagnostics>("diagnostics_out", 1);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Pathfinder";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_cmd_in", ros::Duration(3.0), &Pathfinder::timeoutPositionCmd, this,
                                                                          &Pathfinder::callbackPositionCmd, this);
  sh_octomap_      = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", ros::Duration(5.0), &Pathfinder::timeoutOctomap, this,
                                                                 &Pathfinder::callbackOctomap, this);

  sh_mpc_prediction_ = mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>(
      shopts, "mpc_prediction_in", ros::Duration(3.0), &Pathfinder::timeoutMpcPrediction, this, &Pathfinder::callbackMpcPrediction, this);

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", ros::Duration(3.0),
                                                                                            &Pathfinder::timeoutControlManagerDiag, this);

  sh_constraints_ = mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>(shopts, "constraints_in");

  // | --------------------- service clients -------------------- |

  sc_get_trajectory_       = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
  sc_trajectory_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
  sc_hover_                = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "hover_out");

  // | --------------------- service servers -------------------- |

  service_server_goto_      = nh_.advertiseService("goto_in", &Pathfinder::callbackGoto, this);
  service_server_reference_ = nh_.advertiseService("reference_in", &Pathfinder::callbackReference, this);

  // | ----------------------- transformer ---------------------- |

  transformer_ = mrs_lib::Transformer("Pathfinder", _uav_name_);

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

  timer_main_         = nh_.createTimer(ros::Rate(_rate_main_timer_), &Pathfinder::timerMain, this);
  timer_future_check_ = nh_.createTimer(ros::Rate(_rate_future_check_timer_), &Pathfinder::timerFutureCheck, this);
  timer_diagnostics_  = nh_.createTimer(ros::Rate(_rate_diagnostics_timer_), &Pathfinder::timerDiagnostics, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[Pathfinder]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackPositionCmd() //{ */

void Pathfinder::callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Pathfinder]: getting position cmd");
}

//}

/* timeoutPositionCmd() //{ */

void Pathfinder::timeoutPositionCmd(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_position_cmd_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    ROS_WARN_THROTTLE(1.0, "[Pathfinder]: position cmd timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* callbackOctomap() //{ */

void Pathfinder::callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Pathfinder]: getting octomap");

  OcTreeMsgConstPtr_t octomap_ptr = wrp.getMsg();

  {
    std::scoped_lock lock(mutex_octree_msg_ptr_);

    octree_msg_ptr_ = octomap_ptr;
    octree_frame_   = octomap_ptr->header.frame_id;
  }

  if (!bv_planner_frame_set_) {
    bv_planner_->setParentFrame(octomap_ptr->header.frame_id);
    bv_planner_frame_set_ = true;
  }

  {
    std::scoped_lock lock(mutex_bv_input_);

    bv_input_.setParentFrame(octomap_ptr->header.frame_id);
  }

  {
    std::scoped_lock lock(mutex_bv_processed_);

    bv_processed_.setParentFrame(octomap_ptr->header.frame_id);
  }
}

//}

/* timeoutOctomap() //{ */

void Pathfinder::timeoutOctomap(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_octomap_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    ROS_WARN_THROTTLE(1.0, "[Pathfinder]: octomap timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* callbackMpcPrediction() //{ */

void Pathfinder::callbackMpcPrediction(mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Pathfinder]: getting mpc prediction");
}

//}

/* timeoutMpcPrediction() //{ */

void Pathfinder::timeoutMpcPrediction(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_mpc_prediction_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    ROS_WARN_THROTTLE(1.0, "[Pathfinder]: MPC prediction timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* timeoutControlManagerDiag() //{ */

void Pathfinder::timeoutControlManagerDiag(const std::string& topic, const ros::Time& last_msg, [[maybe_unused]] const int n_pubs) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_mpc_prediction_.hasMsg()) {
    return;
  }

  if (state_ != STATE_IDLE) {

    ROS_WARN_THROTTLE(1.0, "[Pathfinder]: Control manager diag timeouted!");

    ready_to_plan_ = false;

    changeState(STATE_IDLE);

    hover();
  }
}

//}

/* callbackGoto() //{ */

bool Pathfinder::callbackGoto([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  /* prerequisities //{ */

  if (!is_initialized_) {
    return false;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    ROS_ERROR_STREAM_THROTTLE(0.5, "[Pathfinder]: " << ss.str());

    res.success = false;
    res.message = ss.str();
    return true;
  }

  //}

  // | -------- transform the reference to the map frame -------- |

  {
    mrs_msgs::PositionCommandConstPtr position_cmd = sh_position_cmd_.getMsg();

    mrs_msgs::ReferenceStamped reference;
    reference.header.frame_id = position_cmd->header.frame_id;

    reference.reference.position.x = req.goal[0];
    reference.reference.position.y = req.goal[1];
    reference.reference.position.z = req.goal[2];
    reference.reference.heading    = req.goal[3];

    auto result = transformer_.transformSingle(octree_frame_, reference);

    if (result) {

      std::scoped_lock lock(mutex_user_goal_);

      user_goal_ = result.value().reference;

    } else {
      std::stringstream ss;
      ss << "could not transform the reference from " << position_cmd->header.frame_id << " to " << octree_frame_;

      ROS_ERROR_STREAM("[Pathfinder]: " << ss.str());

      res.success = false;
      res.message = ss.str();
      return true;
    }
  }

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

bool Pathfinder::callbackReference([[maybe_unused]] mrs_msgs::ReferenceStampedSrv::Request& req, mrs_msgs::ReferenceStampedSrv::Response& res) {

  /* prerequisities //{ */

  if (!is_initialized_) {
    return false;
  }

  if (!ready_to_plan_) {
    std::stringstream ss;
    ss << "not ready to plan, missing data";

    ROS_ERROR_STREAM_THROTTLE(0.5, "[Pathfinder]: " << ss.str());

    res.success = false;
    res.message = ss.str();
    return true;
  }

  //}

  // | -------- transform the reference to the map frame -------- |

  {
    mrs_msgs::PositionCommandConstPtr position_cmd = sh_position_cmd_.getMsg();

    mrs_msgs::ReferenceStamped reference;
    reference.header    = req.header;
    reference.reference = req.reference;

    auto result = transformer_.transformSingle(octree_frame_, reference);

    if (result) {

      std::scoped_lock lock(mutex_user_goal_);

      user_goal_ = result.value().reference;

    } else {
      std::stringstream ss;
      ss << "could not transform the reference from " << req.header.frame_id << " to " << octree_frame_;

      ROS_ERROR_STREAM("[Pathfinder]: " << ss.str());

      res.success = false;
      res.message = ss.str();
      return true;
    }
  }

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

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void Pathfinder::timerMain([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  /* prerequsitioes //{ */

  const bool got_octomap              = sh_octomap_.hasMsg() && (ros::Time::now() - sh_octomap_.lastMsgTime()).toSec() < 2.0;
  const bool got_position_cmd         = sh_position_cmd_.hasMsg() && (ros::Time::now() - sh_position_cmd_.lastMsgTime()).toSec() < 2.0;
  const bool got_mpc_prediction       = sh_mpc_prediction_.hasMsg() && (ros::Time::now() - sh_mpc_prediction_.lastMsgTime()).toSec() < 2.0;
  const bool got_control_manager_diag = sh_control_manager_diag_.hasMsg() && (ros::Time::now() - sh_control_manager_diag_.lastMsgTime()).toSec() < 2.0;
  const bool got_constraints          = sh_constraints_.hasMsg() && (ros::Time::now() - sh_constraints_.lastMsgTime()).toSec() < 2.0;

  if (!got_octomap || !got_position_cmd || !got_mpc_prediction || !got_control_manager_diag || !got_constraints) {
    ROS_INFO_THROTTLE(1.0, "[Pathfinder]: waiting for data: octomap = %s, position cmd = %s, MPC prediction = %s, ControlManager diag = %s, constraints = %s",
                      got_octomap ? "TRUE" : "FALSE", got_position_cmd ? "TRUE" : "FALSE", got_mpc_prediction ? "TRUE" : "FALSE",
                      got_control_manager_diag ? "TRUE" : "FALSE", got_constraints ? "TRUE" : "FALSE");
    return;
  } else {
    ready_to_plan_ = true;
  }

  //}

  ROS_INFO_ONCE("[Pathfinder]: main timer spinning");

  const auto user_goal = mrs_lib::get_mutexed(mutex_user_goal_, user_goal_);

  const mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();
  const mrs_msgs::PositionCommandConstPtr           position_cmd         = sh_position_cmd_.getMsg();

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

      // copy the octomap locally
      OcTreeMsgConstPtr_t octree_msg_ptr;
      std::string         octree_frame;

      {
        std::scoped_lock lock(mutex_octree_msg_ptr_);

        octree_msg_ptr = octree_msg_ptr_;
        octree_frame   = octree_frame_;
      }

      std::optional<OcTreePtr_t> octree = msgToMap(octree_msg_ptr);

      if (!octree) {

        ROS_ERROR("[Pathfinder]: don't have a map");

        break;
      }

      {
        std::scoped_lock lock(mutex_diagnostics_);

        diagnostics_.idle = false;
      }

      if (replanning_counter_ >= 2) {

        ROS_ERROR("[Pathfinder]: planning failed, the uav is stuck");

        changeState(STATE_IDLE);

        break;
      }

      // get the initial condition

      double time_for_planning;

      if (control_manager_diag->tracker_status.have_goal) {
        time_for_planning = _timeout_threshold_;
      } else {
        time_for_planning = _timeout_threshold_ + pow(1.5, float(replanning_counter_));
      }

      ROS_INFO("[Pathfinder]: planning timeout %.2f s", time_for_planning);

      ros::Time init_cond_time = ros::Time::now() + ros::Duration(time_for_planning + _time_for_trajectory_generator_);

      ROS_INFO("[Pathfinder]: init cond time %.2f s", init_cond_time.toSec());

      auto initial_condition = getInitialCondition(init_cond_time);

      if (!initial_condition) {

        ROS_ERROR_THROTTLE(1.0, "[Pathfinder]: could not obtain initial condition for planning");
        hover();
        changeState(STATE_IDLE);

        break;
      }

      ROS_INFO("[Pathfinder]: init cond time stamp %.2f", initial_condition.value().header.stamp.toSec());

      octomap::point3d plan_from;
      plan_from.x() = initial_condition.value().reference.position.x;
      plan_from.y() = initial_condition.value().reference.position.y;
      plan_from.z() = initial_condition.value().reference.position.z;

      pathfinder::AstarPlanner planner = pathfinder::AstarPlanner(_safe_obstacle_distance_, _euclidean_distance_cutoff_, planning_tree_resolution_,
                                                                  _planning_tree_fractor_, _distance_penalty_, _greedy_penalty_, _timeout_threshold_,
                                                                  _max_waypoint_distance_, _min_altitude_, _max_altitude_, _unknown_is_occupied_, bv_planner_);

      std::pair<std::vector<octomap::point3d>, bool> waypoints = planner.findPath(plan_from, user_goal_octpoint, octree.value(), time_for_planning);

      // path is complete
      if (waypoints.second) {

        replanning_counter_ = 0;

        waypoints.first.push_back(user_goal_octpoint);

      } else {

        if (waypoints.first.size() < 2) {

          ROS_WARN("[Pathfinder]: path not found");

          replanning_counter_++;

          break;
        }

        double front_x = waypoints.first.front().x();
        double front_y = waypoints.first.front().y();
        double front_z = waypoints.first.front().z();

        double back_x = waypoints.first.back().x();
        double back_y = waypoints.first.back().y();
        double back_z = waypoints.first.back().z();

        double path_start_end_dist = sqrt(pow(front_x - back_x, 2) + pow(front_y - back_y, 2) + pow(front_z - back_z, 2));

        if (path_start_end_dist < 0.1) {

          ROS_WARN("[Pathfinder]: path too short");

          replanning_counter_++;

          changeState(STATE_PLANNING);

          break;
        }
      }

      time_last_plan_ = ros::Time::now();

      diagnostics_.best_goal.x = waypoints.first.back().x();
      diagnostics_.best_goal.y = waypoints.first.back().y();
      diagnostics_.best_goal.z = waypoints.first.back().x();

      {
        std::scoped_lock lock(mutex_initial_condition_);

        mrs_msgs::PositionCommandConstPtr position_cmd = sh_position_cmd_.getMsg();
        auto                              octree_frame = mrs_lib::get_mutexed(mutex_octree_msg_ptr_, octree_frame_);

        // transform the position cmd to the map frame
        mrs_msgs::ReferenceStamped position_cmd_ref;
        position_cmd_ref.header               = position_cmd->header;
        position_cmd_ref.reference.position.x = position_cmd->position.x;
        position_cmd_ref.reference.position.y = position_cmd->position.y;
        position_cmd_ref.reference.position.z = position_cmd->position.z;
        position_cmd_ref.reference.heading    = position_cmd->heading;

        auto res = transformer_.transformSingle(octree_frame, position_cmd_ref);

        if (!res) {
          ROS_ERROR("[Pathfinder]: could not transform position cmd to the map frame");
          return;
        }

        initial_pos_.x() = res.value().reference.position.x;
        initial_pos_.y() = res.value().reference.position.y;
        initial_pos_.z() = res.value().reference.position.z;
        initial_heading_ = res.value().reference.heading;
      }

      ros::Time path_stamp = initial_condition.value().header.stamp;

      if (ros::Time::now() > path_stamp || !control_manager_diag->tracker_status.have_goal) {
        path_stamp = ros::Time(0);
      }

      mrs_msgs::GetPathSrv srv_get_path;
      srv_get_path.request.path.header.frame_id = octree_frame_;
      srv_get_path.request.path.header.stamp    = path_stamp;
      srv_get_path.request.path.fly_now         = false;
      srv_get_path.request.path.relax_heading   = true;
      srv_get_path.request.path.use_heading     = true;

      std::vector<Eigen::Vector4d> eig_waypoints;

      // create an array of Eigen waypoints
      for (auto& w : waypoints.first) {

        Eigen::Vector4d eig_waypoint;
        eig_waypoint[0] = w.x();
        eig_waypoint[1] = w.y();
        eig_waypoint[2] = w.z();
        eig_waypoint[4] = user_goal.heading;

        eig_waypoints.push_back(eig_waypoint);
      }

      std::vector<double> segment_times = estimateSegmentTimes(eig_waypoints, false);

      double cum_time = 0;

      for (int i = 0; i < waypoints.first.size(); i++) {

        mrs_msgs::Reference ref;
        ref.position.x = waypoints.first[i].x();
        ref.position.y = waypoints.first[i].y();
        ref.position.z = waypoints.first[i].z();
        ref.heading    = user_goal.heading;

        srv_get_path.request.path.points.push_back(ref);

        cum_time += segment_times[i];

        if (i > 1 && cum_time > 15.0) {
          ROS_INFO("[Pathfinder]: cutting path in waypoint %d out of %d", i, int(waypoints.first.size()));
          break;
        }
      }

      ROS_INFO("[Pathfinder]: calling trajectory generation");

      {
        bool success = sc_get_trajectory_.call(srv_get_path);

        if (!success) {
          ROS_ERROR("[Pathfinder]: service call for trajectory failed");
          break;
        } else {
          if (!srv_get_path.response.success) {
            ROS_ERROR("[Pathfinder]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
            break;
          }
        }
      }

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
      ROS_INFO("[Pathfinder]: Setting replanning point");
      setReplanningPoint(trajectory);
      set_timepoints_ = true;

      ROS_INFO("[Pathfinder]: publishing trajectory reference");

      mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
      srv_trajectory_reference.request.trajectory         = srv_get_path.response.trajectory;
      srv_trajectory_reference.request.trajectory.fly_now = true;

      {
        bool success = sc_trajectory_reference_.call(srv_trajectory_reference);

        if (!success) {
          ROS_ERROR("[Pathfinder]: service call for trajectory reference failed");
          break;
        } else {
          if (!srv_trajectory_reference.response.success) {
            ROS_ERROR("[Pathfinder]: service call for trajectory reference failed: '%s'", srv_get_path.response.message.c_str());
            break;
          } else {
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

      auto initial_pos = mrs_lib::get_mutexed(mutex_initial_condition_, initial_pos_);

      double dist_to_goal = (initial_pos - user_goal_octpoint).norm();

      ROS_INFO_THROTTLE(1.0, "[Pathfinder]: dist to goal: %.2f m", dist_to_goal);

      if (dist_to_goal < 2 * planning_tree_resolution_) {
        ROS_INFO("[Pathfinder]: user goal reached");
        changeState(STATE_IDLE);
        break;
      }

      if ((ros::Time::now() - (time_last_plan_ + ros::Duration(_replan_after_))).toSec() > 0) {

        ROS_INFO("[Pathfinder]: triggering replanning");

        changeState(STATE_PLANNING);
      }

      break;
    }

      //}
  }
}

//}

/* timerFutureCheck() //{ */

void Pathfinder::timerFutureCheck([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  /* preconditions //{ */

  if (!sh_control_manager_diag_.hasMsg()) {
    return;
  }

  if (!sh_octomap_.hasMsg()) {
    return;
  }

  if (!sh_mpc_prediction_.hasMsg()) {
    return;
  }

  //}

  if (state_ == STATE_IDLE) {
    return;
  }

  ROS_INFO_ONCE("[Pathfinder]: future check timer spinning");

  // | ----------- check if the prediction is feasible ---------- |

  // copy the octomap locally
  OcTreeMsgConstPtr_t octree_msg_ptr;
  std::string         octree_frame;

  {
    std::scoped_lock lock(mutex_octree_msg_ptr_);

    octree_msg_ptr = octree_msg_ptr_;
    octree_frame   = octree_frame_;
  }

  auto octree_opt = msgToMap(octree_msg_ptr);

  if (!octree_opt) {
    ROS_ERROR("[Pathfinder]: cannot check for collision, don't have a map");
    return;
  }

  OcTreePtr_t octree = octree_opt.value();

  mrs_msgs::MpcPredictionFullStateConstPtr    prediction           = sh_mpc_prediction_.getMsg();
  mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  if (control_manager_diag->flying_normally && control_manager_diag->tracker_status.have_goal) {

    mrs_lib::TransformStamped tf;

    auto ret = transformer_.getTransform(prediction->header.frame_id, octree_frame, prediction->header.stamp);

    if (!ret) {
      ROS_ERROR_THROTTLE(1.0, "[Pathfinder]: could not transform position cmd to the map frame! can not check for potential collisions!");
      return;
    }

    // prepare the potential future trajectory

    mrs_msgs::TrajectoryReference trajectory;
    trajectory.header.stamp    = ret.value().stamp();
    trajectory.header.frame_id = ret.value().to();
    trajectory.fly_now         = true;
    trajectory.use_heading     = true;
    trajectory.dt              = 0.2;

    for (int i = 1; i < prediction->position.size(); i++) {

      mrs_msgs::ReferenceStamped pose;
      pose.header               = prediction->header;
      pose.reference.position.x = prediction->position[i].x;
      pose.reference.position.y = prediction->position[i].y;
      pose.reference.position.z = prediction->position[i].z;
      pose.reference.heading    = prediction->heading[i];

      auto transformed_pose = transformer_.transform(tf, pose);

      if (!transformed_pose) {
        ROS_ERROR_THROTTLE(1.0, "[Pathfinder]: could not transform position cmd to the map frame! can not check for potential collisions!");
        return;
      }

      trajectory.points.push_back(transformed_pose->reference);
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

          ROS_ERROR_THROTTLE(0.1, "[Pathfinder]: future check found collision with prediction horizon between %d and %d, hovering!", i, i + 1);

          // shorten the trajectory
          for (int j = int(trajectory.points.size()) - 1; j >= floor(i / 2.0); j--) {
            trajectory.points.pop_back();
          }

          mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
          srv_trajectory_reference.request.trajectory = trajectory;

          bool success = sc_trajectory_reference_.call(srv_trajectory_reference);

          if (!success) {
            ROS_ERROR("[Pathfinder]: service call for trajectory reference failed");
            break;
          } else {
            if (!srv_trajectory_reference.response.success) {
              ROS_ERROR("[Pathfinder]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
              break;
            }
          }

          break;
        }

      } else {

        ROS_ERROR_THROTTLE(0.1, "[Pathfinder]: future check failed, could not raytrace!");
        hover();
        break;
      }
    }
  }
}

//}

/* timerDiagnostics() //{ */

void Pathfinder::timerDiagnostics([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  auto diagnostics = mrs_lib::get_mutexed(mutex_diagnostics_, diagnostics_);

  try {
    pub_diagnostics_.publish(diagnostics);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_diagnostics_.getTopic().c_str());
  }
}

//}

// | ------------------------ routines ------------------------ |

/* setReplanningPoint() //{ */

void Pathfinder::setReplanningPoint(const mrs_msgs::TrajectoryReference& traj) {

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

void Pathfinder::changeState(const State_t new_state) {

  const State_t old_state = state_;

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

  ROS_INFO("[Pathfinder]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

  state_ = new_state;
}

//}

/* getInitialCondition() //{ */

std::optional<mrs_msgs::ReferenceStamped> Pathfinder::getInitialCondition(const ros::Time des_time) {

  const mrs_msgs::MpcPredictionFullStateConstPtr prediction_full_state = sh_mpc_prediction_.getMsg();

  if ((des_time - prediction_full_state->stamps.back()).toSec() > 0) {
    ROS_ERROR_THROTTLE(1.0, "[Pathfinder]: could not obtain initial condition, the desired time is too far in the future");
    return {};
  }

  mrs_msgs::ReferenceStamped orig_reference;
  orig_reference.header = prediction_full_state->header;

  ros::Time future_time_stamp;

  for (int i = 0; i < prediction_full_state->stamps.size(); i++) {

    if ((prediction_full_state->stamps[i] - des_time).toSec() > 0) {
      orig_reference.reference.position.x = prediction_full_state->position[i].x;
      orig_reference.reference.position.y = prediction_full_state->position[i].y;
      orig_reference.reference.position.z = prediction_full_state->position[i].z;
      orig_reference.reference.heading    = prediction_full_state->heading[i];
      future_time_stamp                   = prediction_full_state->stamps[i];
      break;
    }
  }

  // transform the initial condition to the current map frame

  auto result = transformer_.transformSingle(octree_frame_, orig_reference);

  if (result) {

    mrs_msgs::ReferenceStamped transfomed_reference = result.value();
    transfomed_reference.header.stamp               = future_time_stamp;

    return transfomed_reference;

  } else {

    std::stringstream ss;
    ss << "could not transform initial condition to the map frame";

    ROS_ERROR_STREAM("[Pathfinder]: " << ss.str());
    return {};
  }
}

//}

/* hover() //{ */

void Pathfinder::hover(void) {

  ROS_INFO("[Pathfinder]: triggering hover");

  std_srvs::Trigger srv_out;

  sc_hover_.call(srv_out);
}

//}

/* estimateSegmentTimes() //{ */

std::vector<double> Pathfinder::estimateSegmentTimes(const std::vector<Eigen::Vector4d>& vertices, const bool use_heading) {

  if (vertices.size() <= 1) {
    return std::vector<double>(0);
  }

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

std::optional<OcTreePtr_t> Pathfinder::msgToMap(const octomap_msgs::OctomapConstPtr octomap) {

  octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*octomap);

  if (!abstract_tree) {

    ROS_WARN("[Pathfinder]: octomap message is empty!");
    return {};

  } else {

    OcTreePtr_t octree_out = OcTreePtr_t(dynamic_cast<OcTree_t*>(abstract_tree));
    return {octree_out};
  }
}

//}

}  // namespace pathfinder

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pathfinder::Pathfinder, nodelet::Nodelet)
