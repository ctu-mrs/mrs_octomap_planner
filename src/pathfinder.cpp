/* includes //{ */

#include "octomap/OcTree.h"
#include <memory>
#include <ros/init.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

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
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/geometry/misc.h>

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

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
  double _planning_tree_resolution_;
  double _timeout_threshold_;
  double _time_for_trajectory_generator_;
  double _max_waypoint_distance_;
  double _endpoint_tolerance_;
  double _min_altitude_;
  double _rate_main_timer_;
  double _rate_future_check_timer_;
  double _replan_after_;
  bool   _unknown_is_occupied_ = false;

  std::shared_ptr<octomap::OcTree> octree_;
  std::string                      octree_frame_;
  std::mutex                       mutex_octree_;

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

  // subscriber callbacks
  void callbackPositionCmd(mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>& wrp);
  void callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp);
  void callbackMpcPrediction(mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>& wrp);

  // service servers
  ros::ServiceServer service_server_goto_;

  // service server callbacks
  bool callbackGoto([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);

  // service clients
  mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>             sc_get_trajectory_;
  mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference_;
  mrs_lib::ServiceClientHandler<std_srvs::Trigger>                sc_hover_;

  // timers
  ros::Timer timer_main_;
  void       timerMain([[maybe_unused]] const ros::TimerEvent& evt);

  ros::Timer timer_future_check_;
  void       timerFutureCheck([[maybe_unused]] const ros::TimerEvent& evt);

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
  param_loader.loadParam("future_check_timer/rate", _rate_future_check_timer_);

  param_loader.loadParam("euclidean_distance_cutoff", _euclidean_distance_cutoff_);
  param_loader.loadParam("safe_obstacle_distance", _safe_obstacle_distance_);
  param_loader.loadParam("distance_penalty", _distance_penalty_);
  param_loader.loadParam("greedy_penalty", _greedy_penalty_);
  param_loader.loadParam("planning_tree_resolution", _planning_tree_resolution_);
  param_loader.loadParam("unknown_is_occupied", _unknown_is_occupied_);
  param_loader.loadParam("points_scale", _points_scale_);
  param_loader.loadParam("endpoint_tolerance", _endpoint_tolerance_);
  param_loader.loadParam("lines_scale", _lines_scale_);
  param_loader.loadParam("max_waypoint_distance", _max_waypoint_distance_);
  param_loader.loadParam("min_altitude", _min_altitude_);
  param_loader.loadParam("timeout_threshold", _timeout_threshold_);
  param_loader.loadParam("replan_after", _replan_after_);
  param_loader.loadParam("time_for_trajectory_generator", _time_for_trajectory_generator_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Pathfinder]: could not load all parameters");
    ros::requestShutdown();
  }

  // | ---------------------- state machine --------------------- |

  state_ = STATE_IDLE;

  // | ------------------------- timers ------------------------- |

  timer_main_         = nh_.createTimer(ros::Rate(_rate_main_timer_), &Pathfinder::timerMain, this);
  timer_future_check_ = nh_.createTimer(ros::Rate(_rate_future_check_timer_), &Pathfinder::timerFutureCheck, this);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Pathfinder";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_position_cmd_ = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_cmd_in", ros::Duration(3.0), &Pathfinder::timeoutPositionCmd, this,
                                                                          &Pathfinder::callbackPositionCmd, this);
  sh_octomap_      = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", ros::Duration(3.0), &Pathfinder::timeoutOctomap, this,
                                                                 &Pathfinder::callbackOctomap, this);

  sh_mpc_prediction_ = mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>(
      shopts, "mpc_prediction_in", ros::Duration(3.0), &Pathfinder::timeoutMpcPrediction, this, &Pathfinder::callbackMpcPrediction, this);

  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", ros::Duration(3.0),
                                                                                            &Pathfinder::timeoutControlManagerDiag, this);

  // | --------------------- service clients -------------------- |

  sc_get_trajectory_       = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
  sc_trajectory_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");
  sc_hover_                = mrs_lib::ServiceClientHandler<std_srvs::Trigger>(nh_, "hover_out");

  // | --------------------- service servers -------------------- |

  service_server_goto_ = nh_.advertiseService("goto_in", &Pathfinder::callbackGoto, this);

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

  octomap_msgs::OctomapConstPtr octomap = wrp.getMsg();

  auto tree_ptr = octomap_msgs::fullMsgToMap(*octomap);

  if (!tree_ptr) {

    ROS_WARN("[Pathfinder]: octomap message is empty!");

  } else {

    std::scoped_lock lock(mutex_octree_);

    octree_       = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));
    octree_frame_ = octomap->header.frame_id;
  }

  if (!bv_planner_frame_set_) {
    bv_planner_->setParentFrame(octomap->header.frame_id);
    bv_planner_frame_set_ = true;
  }

  {
    std::scoped_lock lock(mutex_bv_input_);

    bv_input_.setParentFrame(octomap->header.frame_id);
  }

  {
    std::scoped_lock lock(mutex_bv_processed_);

    bv_processed_.setParentFrame(octomap->header.frame_id);
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

    ROS_ERROR_STREAM("[Pathfinder]: " << ss.str());

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

  if (!got_octomap || !got_position_cmd || !got_mpc_prediction || !got_control_manager_diag) {
    ROS_INFO_THROTTLE(1.0, "[Pathfinder]: waiting for data: octomap = %s, position cmd = %s, MPC prediction = %s, ControlManager diag = %s",
                      got_octomap ? "TRUE" : "FALSE", got_position_cmd ? "TRUE" : "FALSE", got_mpc_prediction ? "TRUE" : "FALSE",
                      got_control_manager_diag ? "TRUE" : "FALSE");
    return;
  } else {
    ready_to_plan_ = true;
  }

  //}

  ROS_INFO_ONCE("[Pathfinder]: main timer spinning");

  // copy the octomap locally
  std::shared_ptr<octomap::OcTree> octree;
  {
    std::scoped_lock lock(mutex_octree_);
    octree = std::make_shared<octomap::OcTree>(*octree_);
  }

  const auto user_goal = mrs_lib::get_mutexed(mutex_user_goal_, user_goal_);

  const mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  octomap::point3d user_goal_octpoint;
  user_goal_octpoint.x() = user_goal.position.x;
  user_goal_octpoint.y() = user_goal.position.y;
  user_goal_octpoint.z() = user_goal.position.z;

  switch (state_) {

    case STATE_IDLE: {
      break;
    }

      /* STATE_PLANNING //{ */

    case STATE_PLANNING: {

      if (replanning_counter_ >= 4) {

        ROS_ERROR("[Pathfinder]: planning failed, uav is stuck!");

        changeState(STATE_IDLE);
        break;
      }

      // get the initial condition

      double time_for_planning = _timeout_threshold_ + pow(1.5, float(replanning_counter_));

      auto initial_condition = getInitialCondition(ros::Time::now() + ros::Duration(time_for_planning + _time_for_trajectory_generator_));

      if (!initial_condition) {
        ROS_ERROR_THROTTLE(1.0, "[Pathfinder]: could not obtain initial condition for planning");
        break;
      }

      octomap::point3d plan_from;
      plan_from.x() = initial_condition.value().reference.position.x;
      plan_from.y() = initial_condition.value().reference.position.y;
      plan_from.z() = initial_condition.value().reference.position.z;

      pathfinder::AstarPlanner planner =
          pathfinder::AstarPlanner(_safe_obstacle_distance_, _euclidean_distance_cutoff_, _planning_tree_resolution_, _distance_penalty_, _greedy_penalty_,
                                   _timeout_threshold_, _max_waypoint_distance_, _min_altitude_, _unknown_is_occupied_, bv_planner_);

      std::pair<std::vector<octomap::point3d>, bool> waypoints = planner.findPath(plan_from, user_goal_octpoint, octree, time_for_planning);

      // path is complete
      if (waypoints.second) {

        replanning_counter_ = 0;

      } else {

        if (waypoints.first.size() < 2) {

          ROS_ERROR("[Pathfinder]: path not found");

          replanning_counter_++;

          changeState(STATE_PLANNING);

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

          ROS_ERROR("[Pathfinder]: path too short");

          replanning_counter_++;

          changeState(STATE_PLANNING);

          break;
        }
      }

      time_last_plan_ = ros::Time::now();

      {
        std::scoped_lock lock(mutex_initial_condition_);

        // TODO should be transformed into the map frame

        mrs_msgs::PositionCommandConstPtr position_cmd = sh_position_cmd_.getMsg();

        initial_pos_.x() = position_cmd->position.x;
        initial_pos_.y() = position_cmd->position.y;
        initial_pos_.z() = position_cmd->position.z;
        initial_heading_ = position_cmd->heading;
      }

      ros::Time path_stamp = initial_condition->header.stamp;

      if (ros::Time::now() > path_stamp || !control_manager_diag->tracker_status.have_goal) {
        path_stamp = ros::Time(0);
      }

      mrs_msgs::GetPathSrv srv_get_path;
      srv_get_path.request.path.header.frame_id = octree_frame_;
      srv_get_path.request.path.header.stamp    = path_stamp;
      srv_get_path.request.path.fly_now         = false;
      srv_get_path.request.path.relax_heading   = true;
      srv_get_path.request.path.use_heading     = true;

      for (auto& w : waypoints.first) {

        mrs_msgs::Reference ref;
        ref.position.x = w.x();
        ref.position.y = w.y();
        ref.position.z = w.z();
        ref.heading    = user_goal.heading;

        srv_get_path.request.path.points.push_back(ref);
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
          if (!srv_get_path.response.success) {
            ROS_ERROR("[Pathfinder]: service call for trajectory reference failed: '%s'", srv_get_path.response.message.c_str());
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

      auto initial_pos = mrs_lib::get_mutexed(mutex_initial_condition_, initial_pos_);

      double dist_to_goal = (initial_pos - user_goal_octpoint).norm();

      ROS_INFO_THROTTLE(1.0, "[Pathfinder]: dist to goal: %.2f m", dist_to_goal);

      if (dist_to_goal < _endpoint_tolerance_) {
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

  ROS_INFO_ONCE("[Pathfinder]: future check timer spinning");

  // | ----------- check if the prediction is feasible ---------- |

  // copy the octomap locally
  std::shared_ptr<octomap::OcTree> octree;
  {
    std::scoped_lock lock(mutex_octree_);
    octree = std::make_shared<octomap::OcTree>(*octree_);
  }

  mrs_msgs::MpcPredictionFullStateConstPtr    prediction           = sh_mpc_prediction_.getMsg();
  mrs_msgs::ControlManagerDiagnosticsConstPtr control_manager_diag = sh_control_manager_diag_.getMsg();

  if (control_manager_diag->flying_normally && control_manager_diag->tracker_status.have_goal) {

    for (int i = 0; i < prediction->position.size(); i++) {

      octomap::point3d   point(prediction->position[i].x, prediction->position[i].y, prediction->position[i].z);
      octomap::OcTreeKey key  = octree->coordToKey(point);
      auto               node = octree->search(key);

      if (!node || octree->isNodeOccupied(node)) {

        ROS_ERROR("[Pathfinder]: future check failed, hovering!");
        hover();
        break;
      }
    }
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

std::optional<mrs_msgs::ReferenceStamped> Pathfinder::getInitialCondition(const ros::Time time) {

  const mrs_msgs::MpcPredictionFullStateConstPtr prediction_full_state = sh_mpc_prediction_.getMsg();

  if (time > prediction_full_state->stamps.back()) {
    ROS_ERROR_THROTTLE(1.0, "[Pathfinder]: could not obtain initial condition, the desired time is too far in the future");
    return {};
  }

  mrs_msgs::ReferenceStamped orig_reference;
  orig_reference.header.frame_id = prediction_full_state->header.frame_id;

  for (int i = 0; i < prediction_full_state->stamps.size(); i++) {

    if ((prediction_full_state->stamps[i] - time).toSec() > 0) {
      orig_reference.reference.position.x = prediction_full_state->position[i].x;
      orig_reference.reference.position.y = prediction_full_state->position[i].y;
      orig_reference.reference.position.z = prediction_full_state->position[i].z;
      orig_reference.reference.heading    = prediction_full_state->heading[i];
      orig_reference.header.stamp         = prediction_full_state->stamps[i];
      break;
    }
  }

  // transform the initial condition to the current map frame

  auto result = transformer_.transformSingle(octree_frame_, orig_reference);

  if (result) {

    return result.value();

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

}  // namespace pathfinder

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pathfinder::Pathfinder, nodelet::Nodelet)
