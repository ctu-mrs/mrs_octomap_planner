/* includes //{ */

#include "mrs_msgs/TrajectoryReference.h"
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

#include <mrs_msgs/PositionCommand.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>

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

  bool        is_initialized_ = false;
  std::string _uav_name_;

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
  double _replan_after_;
  double _rate_main_timer_;
  bool   _unknown_is_occupied_ = false;

  std::shared_ptr<octomap::OcTree> octomap_;
  std::string                      octomap_frame_;
  std::mutex                       mutex_octomap_;

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

  mrs_lib::BatchVisualizer bv_planner_;
  mrs_lib::BatchVisualizer bv_processed_;

  // subscribers
  mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>        sh_position_cmd_;
  mrs_lib::SubscribeHandler<octomap_msgs::Octomap>            sh_octomap_;
  mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState> sh_mpc_prediction_;

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

  // timers
  ros::Timer timer_main_;
  void       timerMain([[maybe_unused]] const ros::TimerEvent& evt);

  // planning
  std::atomic<int> replanning_without_motion_ = 0;

  // state machine
  std::atomic<State_t> state_;
  void                 changeState(const State_t new_state);

  octomap::point3d user_goal_;
  std::mutex       mutex_user_goal_;

  octomap::point3d internal_goal_;

  octomap::point3d plan_from_;
  std::mutex       mutex_plan_from_;

  std::atomic<bool> set_timepoints_ = false;

  ros::Time replanning_start_timepoint_;
  ros::Time replanning_end_timepoint_;

  octomap::point3d replanning_point_;
  std::mutex       mutex_replanning_point_;

  // routines

  void setReplanningPoint(const mrs_msgs::TrajectoryReference& traj);
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
  param_loader.loadParam("time_for_trajectory_generator", _time_for_trajectory_generator_);
  param_loader.loadParam("replan_after", _replan_after_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Pathfinder]: could not load all parameters");
    ros::requestShutdown();
  }

  // | ---------------------- state machine --------------------- |

  state_ = STATE_IDLE;

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_rate_main_timer_), &Pathfinder::timerMain, this);

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "Pathfinder";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_position_cmd_   = mrs_lib::SubscribeHandler<mrs_msgs::PositionCommand>(shopts, "position_cmd_in", &Pathfinder::callbackPositionCmd, this);
  sh_octomap_        = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", &Pathfinder::callbackOctomap, this);
  sh_mpc_prediction_ = mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>(shopts, "mpc_prediction_in", &Pathfinder::callbackMpcPrediction, this);

  // | --------------------- service clients -------------------- |

  sc_get_trajectory_       = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_, "trajectory_generation_out");
  sc_trajectory_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_, "trajectory_reference_out");

  // | --------------------- service servers -------------------- |

  service_server_goto_ = nh_.advertiseService("goto_in", &Pathfinder::callbackGoto, this);

  // | -------------------- batch visualiuzer ------------------- |

  bv_input_ = mrs_lib::BatchVisualizer(nh_, "visualize_input", "");
  bv_input_.setPointsScale(_points_scale_);
  bv_input_.setLinesScale(_lines_scale_);

  bv_planner_ = mrs_lib::BatchVisualizer(nh_, "visualize_planner", "");
  bv_planner_.setPointsScale(_points_scale_);
  bv_planner_.setLinesScale(_lines_scale_);

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

  mrs_msgs::PositionCommandConstPtr position_cmd = wrp.getMsg();

  ROS_INFO_ONCE("[Pathfinder]: getting position cmd");

  {
    std::scoped_lock lock(mutex_initial_condition_);

    initial_pos_.x() = position_cmd->position.x;
    initial_pos_.y() = position_cmd->position.y;
    initial_pos_.z() = position_cmd->position.z;

    initial_heading_ = position_cmd->heading;
  }

  got_initial_pos_ = true;
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

    std::scoped_lock lock(mutex_octomap_);

    octomap_       = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));
    octomap_frame_ = octomap->header.frame_id;
  }
}

//}

/* callbackMpcPrediction() //{ */

void Pathfinder::callbackMpcPrediction(mrs_lib::SubscribeHandler<mrs_msgs::MpcPredictionFullState>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[Pathfinder]: getting mpc prediction");

  mrs_msgs::MpcPredictionFullStateConstPtr prediction = wrp.getMsg();

  if (set_timepoints_) {

    auto replanning_point = mrs_lib::get_mutexed(mutex_replanning_point_, replanning_point_);

    for (size_t i = 0; i < prediction->position.size(); i++) {

      octomap::point3d p;
      p.x() = prediction->position[i].x;
      p.y() = prediction->position[i].y;
      p.z() = prediction->position[i].z;

      if ((p - replanning_point_).norm() < 0.08) {

        {
          std::scoped_lock lock(mutex_replanning_point_);

          replanning_end_timepoint_   = prediction->stamps[i];
          replanning_start_timepoint_ = replanning_end_timepoint_ - ros::Duration(_timeout_threshold_ + _time_for_trajectory_generator_);
        }

        ROS_INFO("[Pathfinder]: replanning will start in %.3f seconds!", (replanning_start_timepoint_ - ros::Time::now()).toSec());
        set_timepoints_ = false;

        break;
      }
    }
  }
}

//}

/* callbackGoto() //{ */

bool Pathfinder::callbackGoto([[maybe_unused]] mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {
    return false;
  }

  const double x   = req.goal[0];
  const double y   = req.goal[1];
  const double z   = req.goal[2];
  const double hdg = req.goal[3];

  {
    std::scoped_lock lock(mutex_user_goal_);

    user_goal_.x() = float(x);
    user_goal_.y() = float(y);
    user_goal_.z() = float(z);
  }

  {
    std::scoped_lock lock(mutex_plan_from_);

    plan_from_ = initial_pos_;
  }

  changeState(STATE_PLANNING);

  {
    std::scoped_lock lock(mutex_bv_input_);

    bv_input_.clearBuffers();
    bv_input_.addPoint(Eigen::Vector3d(x, y, z));
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

  const bool got_octomap        = sh_octomap_.hasMsg();
  const bool got_position_cmd   = sh_position_cmd_.hasMsg();
  const bool got_mpc_prediction = sh_mpc_prediction_.hasMsg();

  if (!got_octomap || !got_position_cmd || !got_mpc_prediction) {
    ROS_INFO_THROTTLE(1.0, "[Pathfinder]: waiting for data: octomap = %s, position cmd = %s, MPC prediction = %s", got_octomap ? "TRUE" : "FALSE",
                      got_position_cmd ? "TRUE" : "FALSE", got_mpc_prediction ? "TRUE" : "FALSE");
    return;
  }

  //}

  const auto [octomap, map_frame] = mrs_lib::get_mutexed(mutex_octomap_, octomap_, octomap_frame_);
  const auto user_goal            = mrs_lib::get_mutexed(mutex_user_goal_, user_goal_);
  const auto plan_from            = mrs_lib::get_mutexed(mutex_plan_from_, plan_from_);

  switch (state_) {

    case STATE_IDLE: {
      break;
    }

    case STATE_PLANNING: {

      pathfinder::AstarPlanner planner =
          pathfinder::AstarPlanner(_safe_obstacle_distance_, _euclidean_distance_cutoff_, _planning_tree_resolution_, _distance_penalty_, _greedy_penalty_,
                                   _timeout_threshold_, _max_waypoint_distance_, _min_altitude_, _unknown_is_occupied_, bv_planner_);

      auto waypoints = planner.findPath(plan_from, user_goal, octomap);

      if (waypoints.size() < 2) {

        ROS_ERROR("[Pathfinder]: path not found");

        replanning_without_motion_++;

        if (replanning_without_motion_ > 10) {
          ROS_ERROR("[Pathfinder]: planning failed, uav is stuck!");
          changeState(STATE_IDLE);
          break;
        }

        changeState(STATE_PLANNING);
        break;
      }

      mrs_msgs::GetPathSrv srv_get_path;
      srv_get_path.request.path.header.frame_id = octomap_frame_;
      srv_get_path.request.path.header.stamp    = ros::Time::now();
      srv_get_path.request.path.fly_now         = false;

      for (auto& w : waypoints) {
        mrs_msgs::Reference ref;
        ref.position.x = w.x();
        ref.position.y = w.y();
        ref.position.z = w.z();
        ref.heading    = 0;
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

      bv_processed_.clearBuffers();
      for (auto& p : srv_get_path.response.trajectory.points) {
        auto v = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        bv_processed_.addPoint(v, 0, 1, 0, 1);
      }
      bv_processed_.publish();

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

    case STATE_MOVING: {

      replanning_without_motion_ = 0;

      auto initial_pos = mrs_lib::get_mutexed(mutex_initial_condition_, initial_pos_);

      if ((initial_pos - user_goal).norm() < _endpoint_tolerance_) {
        ROS_INFO("[Pathfinder]: user goal reached");
        changeState(STATE_IDLE);
        break;
      }

      if ((replanning_start_timepoint_ - ros::Time::now()).toSec() <= 0) {

        ROS_INFO("[Pathfinder]: triggering replanning");

        {
          std::scoped_lock lock(mutex_replanning_point_, mutex_plan_from_);

          plan_from_ = replanning_point_;
        }

        changeState(STATE_PLANNING);
      }

      break;
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

  auto user_goal = mrs_lib::get_mutexed(mutex_user_goal_, user_goal_);

  {
    std::scoped_lock lock(mutex_bv_input_);

    bv_input_.clearBuffers();
    bv_input_.addPoint(Eigen::Vector3d(user_goal.x(), user_goal.y(), user_goal.z()), 0, 1, 0, 1);
    bv_input_.addCuboid(c, 0.5, 1.0, 1.0, 0.8, true);
    bv_input_.publish();
  }
}

//}

/* changeState() //{ */

void Pathfinder::changeState(const State_t new_state) {

  const State_t old_state = state_;

  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_PLANNING: {

      break;
    }

    case STATE_MOVING: {

      break;
    }

    default: {

      ROS_ERROR("[Pathfinder]: swtiching to unchecked state!");
      break;
    }
  }

  ROS_INFO("[Pathfinder]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

  state_ = new_state;
}

//}

}  // namespace pathfinder

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pathfinder::Pathfinder, nodelet::Nodelet)
