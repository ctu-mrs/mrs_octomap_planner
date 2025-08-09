#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_octomap_planner/Path.h>
#include <mrs_octomap_tools/octomap_methods.h>
#include <nodelet/nodelet.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <ros/ros.h>
#include <astar_planner.hpp>
#include <iostream>
#include <memory>

namespace mrs_octomap_planner
{

  using OcTree_t          = octomap::OcTree;
  using OcTreeSharedPtr_t = std::shared_ptr<octomap::OcTree>;

  class MinimalOctomapPlanner : public nodelet::Nodelet
  {

  public:
    virtual void onInit();

  private:
    ros::NodeHandle nh_;

    bool        is_initialized_ = false;
    std::string _uav_name_;

    // params
    double _safe_obstacle_distance_      = 0.0;
    double _distance_transform_distance_ = 0.0;
    double _planning_tree_resolution_    = 0.0;
    double _distance_penalty_            = 0.0;
    double _greedy_penalty_              = 0.0;
    double _timeout_threshold_           = 0.0;
    double _max_waypoint_distance_       = 0.0;
    double _min_altitude_                = 0.0;
    double _max_altitude_                = 0.0;
    double _scale_points_                = 0.0;
    double _scale_lines_                 = 0.0;
    double _min_path_length_             = 0.0;
    bool   _unknown_is_occupied_         = false;


    std::mutex                                mutex_octree_;
    std::shared_ptr<OcTree_t>                 octree_ = nullptr;
    std::string                               octree_frame_;
    std::shared_ptr<mrs_lib::BatchVisualizer> bv_planner_;

    mrs_lib::SubscribeHandler<octomap_msgs::Octomap> sh_octomap_;

    void timeoutOctomap(const std::string& topic,
                        const ros::Time&   last_msg);
    void callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);

    ros::ServiceServer service_server_get_path_;

    bool callbackGetPath(mrs_octomap_planner::Path::Request&  req,
                         mrs_octomap_planner::Path::Response& res);

    std::unique_ptr<mrs_lib::Transformer> transformer_;

    std::optional<OcTreeSharedPtr_t> msgToMap(const octomap_msgs::OctomapConstPtr octomap);
  };

  void MinimalOctomapPlanner::onInit()
  {
    nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

    ros::Time::waitForValid();

    ROS_INFO("[MrsMinimalOctomapPlanner]: initializing");

    mrs_lib::ParamLoader param_loader(nh_, "MrsMinimalOctomapPlanner");

    param_loader.loadParam("uav_name", _uav_name_);

    param_loader.loadParam("safe_obstacle_distance", _safe_obstacle_distance_);
    param_loader.loadParam("distance_penalty", _distance_penalty_);
    param_loader.loadParam("greedy_penalty", _greedy_penalty_);
    param_loader.loadParam("planning_tree_resolution", _planning_tree_resolution_);
    param_loader.loadParam("unknown_is_occupied", _unknown_is_occupied_);
    param_loader.loadParam("distance_transform_distance", _distance_transform_distance_);
    param_loader.loadParam("max_waypoint_distance", _max_waypoint_distance_);
    param_loader.loadParam("min_altitude", _min_altitude_);
    param_loader.loadParam("max_altitude", _max_altitude_);
    param_loader.loadParam("timeout_threshold", _timeout_threshold_);
    param_loader.loadParam("min_path_length", _min_path_length_);

    param_loader.loadParam("viz/scale/points", _scale_points_);
    param_loader.loadParam("viz/scale/lines", _scale_lines_);

    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[MrsMinimalOctomapPlanner]: Could not load all parameters");
      ros::shutdown();
    }

    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_;
    shopts.node_name          = "MrsMinimalOctomapPlanner";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 1;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts,
                                                                   "octomap_in",
                                                                   ros::Duration(5.0),
                                                                   &MinimalOctomapPlanner::timeoutOctomap,
                                                                   this,
                                                                   &MinimalOctomapPlanner::callbackOctomap,
                                                                   this);

    service_server_get_path_ = nh_.advertiseService("get_path_in", &MinimalOctomapPlanner::callbackGetPath, this);

    transformer_ = std::make_unique<mrs_lib::Transformer>("MrsMinimalOctomapPlanner");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);

    bv_planner_ = std::make_shared<mrs_lib::BatchVisualizer>(nh_, "visualize_planner", "");
    bv_planner_->setPointsScale(_scale_points_);
    bv_planner_->setLinesScale(_scale_lines_);

    is_initialized_ = true;

    ROS_INFO("[MrsMinimalOctomapPlanner]: initialized");
  }


  void MinimalOctomapPlanner::callbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
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
  }

  std::optional<OcTreeSharedPtr_t> MinimalOctomapPlanner::msgToMap(const octomap_msgs::OctomapConstPtr octomap)
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

      OcTreeSharedPtr_t octree_out = OcTreeSharedPtr_t(dynamic_cast<OcTree_t*>(abstract_tree));
      return { octree_out };
    }
  }

  void MinimalOctomapPlanner::timeoutOctomap(const std::string& topic,
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

  bool MinimalOctomapPlanner::callbackGetPath(mrs_octomap_planner::Path::Request&  req,
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

    bv_planner_->setParentFrame(mrs_lib::get_mutexed(mutex_octree_, octree_frame_));
    mrs_octomap_planner::AstarPlanner planner = mrs_octomap_planner::AstarPlanner(_safe_obstacle_distance_,
                                                                                  _safe_obstacle_distance_,
                                                                                  _distance_transform_distance_,
                                                                                  _planning_tree_resolution_,
                                                                                  _distance_penalty_,
                                                                                  _greedy_penalty_,
                                                                                  _timeout_threshold_,
                                                                                  _max_waypoint_distance_,
                                                                                  _min_altitude_,
                                                                                  _max_altitude_,
                                                                                  _unknown_is_occupied_,
                                                                                  bv_planner_);

    octomap::point3d plan_from, plan_to;
    plan_from.x() = req.start.x;
    plan_from.y() = req.start.y;
    plan_from.z() = req.start.z;

    plan_to.x() = req.end.x;
    plan_to.y() = req.end.y;
    plan_to.z() = req.end.z;

    OcTreeSharedPtr_t octree = mrs_lib::get_mutexed(mutex_octree_, octree_);

    auto path = planner.findPath(plan_from, plan_to, octree, _timeout_threshold_);

    // path is complete
    if (path.second) {
      path.first.push_back(plan_to);
      std::stringstream ss;
      ss << "Found complete path of length = " << path.first.size();
      ROS_INFO_STREAM("[MrsMinimalOctomapPlanner]: " << ss.str());
      res.message = ss.str();
    }
    else {
      if (path.first.size() < 2) {
        ROS_WARN("[MrsMinimalOctomapPlanner]: No path found");
        res.success = false;
        res.message = "No path found";
        res.path    = std::vector<geometry_msgs::Point>();
        return true;
      }
      
      std::stringstream ss;
      ss << "Incomplete path found of length = " << path.first.size();
      ROS_INFO_STREAM("[MrsMinimalOctomapPlanner]: " << ss.str());
      res.message = ss.str();

      double front_x = path.first.front().x();
      double front_y = path.first.front().y();
      double front_z = path.first.front().z();

      double back_x = path.first.back().x();
      double back_y = path.first.back().y();
      double back_z = path.first.back().z();

      double dist_path_start_to_end =
          sqrt(pow(front_x - back_x, 2) + pow(front_y - back_y, 2) + pow(front_z - back_z, 2));

      if (dist_path_start_to_end < _min_path_length_) {
        std::stringstream ss;
        ss << "Path too short, length: " << dist_path_start_to_end;
        ROS_WARN_STREAM("[MrsMinimalOctomapPlanner]: " << ss.str());
        res.message = ss.str();
      }
    }

    std::vector<geometry_msgs::Point> tf_path;
    auto                              from_frame = mrs_lib::get_mutexed(mutex_octree_, octree_frame_);
    auto                              to_frame   = req.header.frame_id;
    auto                              ret        = transformer_->getTransform(from_frame, to_frame, ros::Time::now());

    if (!ret) {
      ROS_ERROR_STREAM_THROTTLE(1.0,
                                "[MrsMinimalOctomapPlanner]: Failed to transform path from " << from_frame << " to "
                                                                                             << to_frame);
      return false;
    }

    auto tf = ret.value();

    for (auto& point : path.first) {

      geometry_msgs::PointStamped tmp_pt;
      tmp_pt.header.stamp    = ros::Time::now();
      tmp_pt.header.frame_id = from_frame;
      tmp_pt.point.x         = point.x();
      tmp_pt.point.y         = point.y();
      tmp_pt.point.z         = point.z();

      auto transformed_point = transformer_->transform(tmp_pt, tf);

      if (!transformed_point) {
        ROS_ERROR_STREAM_THROTTLE(1.0,
                                  "[MrsMinimalOctomapPlanner]: Failed to transform path point from "
                                      << from_frame << " to " << to_frame << " even when TF exists");
        return false;
      }

      tf_path.push_back(transformed_point->point);
    }

    res.success         = true;
    res.header.stamp    = ros::Time::now();
    res.header.frame_id = to_frame;
    res.path            = tf_path;
    return true;
  }

}  // namespace mrs_octomap_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_octomap_planner::MinimalOctomapPlanner,
                       nodelet::Nodelet)
